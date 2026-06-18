#!/usr/bin/env python3
"""
===============================================================
Raxzion Robot — Pi Data Router v0.1
Project: AI Robotic System — Leonardo Control Architecture
Author:  Leonardo (AI Superagent) for Aaron Edgley / Raxzion Systems, LLC.
Date:    2026-06-18
Device:  Raspberry Pi 4 (aboard EPIB)
===============================================================

OVERVIEW:
  This script is the central nervous system of the robot stack.
  It runs as a daemon on the Raspberry Pi and does three things:

  1. READS sensor JSON packets from the SIB via /dev/ttyUSB0
  2. PUBLISHES sensor data upstream to MQTT broker (Leonardo / AI layer)
  3. SUBSCRIBES to AI control commands from MQTT and WRITES them
     to the MDB via /dev/ttyUSB1

DATA FLOW:
  SIB ESP32 ──UART──► /dev/ttyUSB0 ──► [THIS SCRIPT] ──MQTT──► Leonardo (AI)
  Leonardo (AI) ──MQTT──► [THIS SCRIPT] ──UART──► /dev/ttyUSB1 ──► MDB ESP32

MQTT TOPICS:
  raxzion/robot/sensors   — sensor data published upstream (SIB → AI)
  raxzion/robot/control   — control commands received downstream (AI → MDB)
  raxzion/robot/status    — router health, battery, fault state

DEPENDENCIES:
  pip install pyserial paho-mqtt

USAGE:
  python3 data_router.py
  python3 data_router.py --sib-port /dev/ttyUSB0 --mdb-port /dev/ttyUSB1
  python3 data_router.py --broker 192.168.1.100 --broker-port 1883
===============================================================
"""

import argparse
import json
import logging
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from queue import Queue, Empty

import paho.mqtt.client as mqtt
import serial
import serial.tools.list_ports

# ─── Logging Setup ───────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(name)s — %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("/var/log/raxzion_router.log", mode="a"),
    ],
)
log = logging.getLogger("DataRouter")

# ─── Default Configuration ───────────────────────────────────
DEFAULT_SIB_PORT    = "/dev/ttyUSB0"
DEFAULT_MDB_PORT    = "/dev/ttyUSB1"
DEFAULT_BAUD        = 115200
DEFAULT_BROKER      = "localhost"       # Mosquitto runs on Pi
DEFAULT_BROKER_PORT = 1883
DEFAULT_KEEPALIVE   = 60
TOPIC_SENSORS       = "raxzion/robot/sensors"
TOPIC_CONTROL       = "raxzion/robot/control"
TOPIC_STATUS        = "raxzion/robot/status"
RECONNECT_DELAY_S   = 3
SERIAL_TIMEOUT_S    = 1.0
STATUS_INTERVAL_S   = 5.0              # Publish router status every 5s
MAX_PACKET_AGE_MS   = 500             # Drop sensor packets older than 500ms


# ════════════════════════════════════════════════════════════
# RAXZION DATA ROUTER
# ════════════════════════════════════════════════════════════
class RaxzionDataRouter:
    def __init__(self, config: dict):
        self.cfg = config
        self.running = False

        # Serial connections
        self.sib_serial: serial.Serial | None = None
        self.mdb_serial: serial.Serial | None = None

        # MQTT client
        self.mqtt_client: mqtt.Client | None = None
        self.mqtt_connected = False

        # Thread-safe queues
        self.sensor_queue  = Queue(maxsize=100)   # SIB → MQTT
        self.control_queue = Queue(maxsize=100)   # MQTT → MDB

        # Stats
        self.stats = {
            "packets_received":  0,
            "packets_published": 0,
            "commands_received": 0,
            "commands_sent":     0,
            "parse_errors":      0,
            "serial_errors":     0,
            "start_time":        time.time(),
        }

        # Threads
        self._threads = []

    # ── Startup ──────────────────────────────────────────────
    def start(self):
        log.info("=" * 55)
        log.info("  Raxzion Data Router v0.1 starting up")
        log.info("=" * 55)
        self.running = True

        self._connect_serial()
        self._connect_mqtt()

        # Launch worker threads
        threads = [
            threading.Thread(target=self._sib_reader_thread,    name="SIBReader",    daemon=True),
            threading.Thread(target=self._sensor_publisher_thread, name="SensorPub", daemon=True),
            threading.Thread(target=self._mdb_writer_thread,    name="MDBWriter",    daemon=True),
            threading.Thread(target=self._status_thread,        name="StatusPub",    daemon=True),
        ]
        for t in threads:
            t.start()
            self._threads.append(t)
            log.info(f"Thread started: {t.name}")

        log.info("Router is live. Waiting for sensor data...")

        # Block main thread — wait for shutdown signal
        try:
            while self.running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        log.info("Shutting down Data Router...")
        self.running = False
        time.sleep(1)
        if self.sib_serial and self.sib_serial.is_open:
            self.sib_serial.close()
        if self.mdb_serial and self.mdb_serial.is_open:
            self.mdb_serial.close()
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        log.info("Router stopped cleanly.")

    # ── Serial Connection ─────────────────────────────────────
    def _connect_serial(self):
        # SIB
        while self.running:
            try:
                self.sib_serial = serial.Serial(
                    port=self.cfg["sib_port"],
                    baudrate=self.cfg["baud"],
                    timeout=SERIAL_TIMEOUT_S
                )
                log.info(f"SIB serial open: {self.cfg['sib_port']} @ {self.cfg['baud']} baud")
                break
            except serial.SerialException as e:
                log.warning(f"SIB serial not available ({e}) — retrying in {RECONNECT_DELAY_S}s")
                time.sleep(RECONNECT_DELAY_S)

        # MDB
        while self.running:
            try:
                self.mdb_serial = serial.Serial(
                    port=self.cfg["mdb_port"],
                    baudrate=self.cfg["baud"],
                    timeout=SERIAL_TIMEOUT_S
                )
                log.info(f"MDB serial open: {self.cfg['mdb_port']} @ {self.cfg['baud']} baud")
                break
            except serial.SerialException as e:
                log.warning(f"MDB serial not available ({e}) — retrying in {RECONNECT_DELAY_S}s")
                time.sleep(RECONNECT_DELAY_S)

    # ── MQTT Connection ───────────────────────────────────────
    def _connect_mqtt(self):
        self.mqtt_client = mqtt.Client(client_id="raxzion_router", clean_session=True)
        self.mqtt_client.on_connect    = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message    = self._on_mqtt_message

        while self.running:
            try:
                self.mqtt_client.connect(
                    self.cfg["broker"],
                    self.cfg["broker_port"],
                    DEFAULT_KEEPALIVE
                )
                self.mqtt_client.loop_start()
                time.sleep(1)  # Allow connect callback to fire
                log.info(f"MQTT connecting to {self.cfg['broker']}:{self.cfg['broker_port']}")
                break
            except Exception as e:
                log.warning(f"MQTT broker unavailable ({e}) — retrying in {RECONNECT_DELAY_S}s")
                time.sleep(RECONNECT_DELAY_S)

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            log.info("MQTT connected to broker.")
            # Subscribe to AI control commands
            client.subscribe(TOPIC_CONTROL, qos=1)
            client.subscribe(TOPIC_STATUS + "/request", qos=0)
            log.info(f"Subscribed to: {TOPIC_CONTROL}")
        else:
            log.error(f"MQTT connect failed — code {rc}")

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.mqtt_connected = False
        if rc != 0:
            log.warning(f"MQTT unexpected disconnect (rc={rc}) — auto-reconnecting...")

    def _on_mqtt_message(self, client, userdata, msg):
        """Receives control commands from Leonardo / AI agent."""
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except json.JSONDecodeError as e:
            log.warning(f"MQTT message parse error on {topic}: {e}")
            return

        if topic == TOPIC_CONTROL:
            # Validate required fields
            if not self._validate_control_packet(payload):
                log.warning(f"Invalid control packet dropped: {payload}")
                return
            # Enqueue for MDB writer thread
            if not self.control_queue.full():
                self.control_queue.put(payload)
                self.stats["commands_received"] += 1
            else:
                log.warning("Control queue full — dropping command packet")

    # ── Thread: SIB Reader ────────────────────────────────────
    def _sib_reader_thread(self):
        """Continuously reads newline-delimited JSON from SIB UART."""
        log.info("SIB reader thread running.")
        while self.running:
            if not self.sib_serial or not self.sib_serial.is_open:
                time.sleep(0.1)
                continue
            try:
                raw = self.sib_serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # Parse JSON
                packet = json.loads(line)

                # Attach router-side timestamp (UTC ISO8601)
                packet["router_timestamp"] = datetime.now(timezone.utc).isoformat()

                self.stats["packets_received"] += 1

                if not self.sensor_queue.full():
                    self.sensor_queue.put(packet)
                else:
                    log.warning("Sensor queue full — dropping packet")

            except json.JSONDecodeError:
                self.stats["parse_errors"] += 1
                # Silently drop — SIB may send debug lines during boot
            except serial.SerialException as e:
                self.stats["serial_errors"] += 1
                log.error(f"SIB serial error: {e}")
                time.sleep(RECONNECT_DELAY_S)
            except Exception as e:
                log.error(f"SIB reader unexpected error: {e}")
                time.sleep(0.1)

    # ── Thread: Sensor Publisher ──────────────────────────────
    def _sensor_publisher_thread(self):
        """Publishes sensor packets from queue to MQTT broker."""
        log.info("Sensor publisher thread running.")
        while self.running:
            try:
                packet = self.sensor_queue.get(timeout=1.0)
            except Empty:
                continue

            if not self.mqtt_connected:
                # Re-queue and wait for connection
                self.sensor_queue.put(packet)
                time.sleep(0.1)
                continue

            # Drop stale packets
            pkt_age_ms = time.time() * 1000 - packet.get("timestamp", 0)
            if pkt_age_ms > MAX_PACKET_AGE_MS:
                log.debug(f"Dropping stale packet (age={pkt_age_ms:.0f}ms)")
                continue

            try:
                payload_str = json.dumps(packet, separators=(",", ":"))
                result = self.mqtt_client.publish(
                    TOPIC_SENSORS,
                    payload=payload_str,
                    qos=0,          # QoS 0 — fire and forget for high-rate sensor data
                    retain=False
                )
                if result.rc == mqtt.MQTT_ERR_SUCCESS:
                    self.stats["packets_published"] += 1
                else:
                    log.warning(f"MQTT publish failed: rc={result.rc}")
            except Exception as e:
                log.error(f"Sensor publish error: {e}")

    # ── Thread: MDB Writer ────────────────────────────────────
    def _mdb_writer_thread(self):
        """Writes control command packets to MDB via UART."""
        log.info("MDB writer thread running.")
        while self.running:
            try:
                cmd = self.control_queue.get(timeout=1.0)
            except Empty:
                continue

            if not self.mdb_serial or not self.mdb_serial.is_open:
                log.warning("MDB serial not open — dropping command")
                continue

            try:
                # Serialize to single-line JSON + newline (MDB parser expects this)
                cmd_str = json.dumps(cmd, separators=(",", ":")) + "\n"
                self.mdb_serial.write(cmd_str.encode("utf-8"))
                self.mdb_serial.flush()
                self.stats["commands_sent"] += 1
                log.debug(f"→ MDB: {cmd_str.strip()}")
            except serial.SerialException as e:
                self.stats["serial_errors"] += 1
                log.error(f"MDB serial write error: {e}")
            except Exception as e:
                log.error(f"MDB writer unexpected error: {e}")

    # ── Thread: Status Publisher ──────────────────────────────
    def _status_thread(self):
        """Publishes router health status to MQTT every 5 seconds."""
        log.info("Status publisher thread running.")
        while self.running:
            time.sleep(STATUS_INTERVAL_S)
            if not self.mqtt_connected:
                continue
            try:
                uptime_s = int(time.time() - self.stats["start_time"])
                status = {
                    "timestamp":          datetime.now(timezone.utc).isoformat(),
                    "uptime_s":           uptime_s,
                    "packets_received":   self.stats["packets_received"],
                    "packets_published":  self.stats["packets_published"],
                    "commands_received":  self.stats["commands_received"],
                    "commands_sent":      self.stats["commands_sent"],
                    "parse_errors":       self.stats["parse_errors"],
                    "serial_errors":      self.stats["serial_errors"],
                    "sib_connected":      self.sib_serial is not None and self.sib_serial.is_open,
                    "mdb_connected":      self.mdb_serial is not None and self.mdb_serial.is_open,
                    "mqtt_connected":     self.mqtt_connected,
                    "sensor_queue_depth": self.sensor_queue.qsize(),
                    "control_queue_depth":self.control_queue.qsize(),
                }
                self.mqtt_client.publish(
                    TOPIC_STATUS,
                    payload=json.dumps(status, separators=(",", ":")),
                    qos=0,
                    retain=True   # Retain so AI agent always has latest status
                )
            except Exception as e:
                log.error(f"Status publish error: {e}")

    # ── Validation ────────────────────────────────────────────
    def _validate_control_packet(self, packet: dict) -> bool:
        """Validates incoming control command structure from AI agent."""
        required = ["timestamp"]
        for field in required:
            if field not in packet:
                return False
        # motor_a and motor_b are optional — robot may only move one axis
        if "motor_a" in packet:
            if "speed" not in packet["motor_a"] or "direction" not in packet["motor_a"]:
                return False
            if packet["motor_a"]["direction"] not in ("forward", "reverse", "stop"):
                return False
        if "motor_b" in packet:
            if "speed" not in packet["motor_b"] or "direction" not in packet["motor_b"]:
                return False
        return True


# ════════════════════════════════════════════════════════════
# ENTRY POINT
# ════════════════════════════════════════════════════════════
def parse_args():
    parser = argparse.ArgumentParser(description="Raxzion Robot Pi Data Router v0.1")
    parser.add_argument("--sib-port",    default=DEFAULT_SIB_PORT,    help="SIB serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--mdb-port",    default=DEFAULT_MDB_PORT,    help="MDB serial port (default: /dev/ttyUSB1)")
    parser.add_argument("--baud",        default=DEFAULT_BAUD,        type=int, help="UART baud rate (default: 115200)")
    parser.add_argument("--broker",      default=DEFAULT_BROKER,      help="MQTT broker address (default: localhost)")
    parser.add_argument("--broker-port", default=DEFAULT_BROKER_PORT, type=int, help="MQTT broker port (default: 1883)")
    parser.add_argument("--debug",       action="store_true",         help="Enable debug logging")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    config = {
        "sib_port":    args.sib_port,
        "mdb_port":    args.mdb_port,
        "baud":        args.baud,
        "broker":      args.broker,
        "broker_port": args.broker_port,
    }

    router = RaxzionDataRouter(config)

    # Graceful shutdown on SIGINT / SIGTERM
    def handle_signal(sig, frame):
        log.info(f"Signal {sig} received — shutting down.")
        router.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    router.start()


if __name__ == "__main__":
    main()
