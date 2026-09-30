"""Phone GPS over Bluetooth SPP, used by qcomgpsd only while the modem has no fix.

Phones advertise several serial ports at once (other GPS apps, Android's Nearby Share), and BlueZ's
ConnectProfile connects to whichever it finds first, so every port is probed for real NMEA instead.
"""
import signal
import socket
import subprocess
import threading
import time

from jeepney import DBusAddress, new_method_call
from jeepney.io.threading import DBusRouter, open_dbus_connection
from jeepney.low_level import HeaderFields, MessageType

from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.system.bluetooth.bluez import ADAPTER_IFACE, BLUEZ, DEVICE_IFACE, OBJECT_MANAGER, unwrap_variant
from openpilot.starpilot.system.bluetooth.phone_gps_fix import (PHONE_GPS_STATUS_PATH, NmeaAccumulator, clear_phone_fix, contains_nmea,
                                                                parse_serial_ports, write_phone_fix, write_phone_status)
from openpilot.starpilot.system.bluetooth.protocol import is_phone

SPP_UUID = "00001101-0000-1000-8000-00805f9b34fb"

CONNECT_POLL_S = 2.0
RETRY_MIN_S = 15.0
RETRY_MAX_S = 120.0
SDP_TIMEOUT_S = 20.0
CONNECT_TIMEOUT_S = 15.0
# NMEA apps send at ~1Hz even without a fix.
PROBE_TIMEOUT_S = 6.0
STALL_TIMEOUT_S = 10.0


def is_phone_candidate(props: dict) -> bool:
  if not props.get("Paired", False) or props.get("Blocked", False):
    return False
  uuids = {str(uuid).lower() for uuid in props.get("UUIDs", [])}
  return SPP_UUID in uuids or is_phone(int(props.get("Class", 0)), str(props.get("Icon", "")))


def browse_serial_ports(address: str) -> list[tuple[str, int]]:
  result = subprocess.run(["sdptool", "browse", address], capture_output=True, text=True, timeout=SDP_TIMEOUT_S, check=False)
  return parse_serial_ports(result.stdout)


def probe_nmea(address: str, channel: int) -> tuple[socket.socket | None, bytes, str]:
  sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
  received = b""
  connected = False
  try:
    # Connecting includes paging the phone and encrypting the link, so it gets its own, longer timeout.
    sock.settimeout(CONNECT_TIMEOUT_S)
    sock.connect((address, channel))
    connected = True
    sock.settimeout(PROBE_TIMEOUT_S)
    deadline = time.monotonic() + PROBE_TIMEOUT_S
    while time.monotonic() < deadline:
      data = sock.recv(1024)
      if not data:
        break
      received += data
      if contains_nmea(received):
        return sock, received, ""
    reason = "no NMEA"
  except TimeoutError:
    reason = "no NMEA" if connected else "connect timed out"
  except OSError as error:
    reason = error.strerror or str(error)
  sock.close()
  return None, b"", reason


class PhoneGpsDaemon:
  def __init__(self):
    self.router = DBusRouter(open_dbus_connection(bus="SYSTEM"))
    self._state_lock = threading.Lock()
    self._stop = threading.Event()
    self._stopped = False
    self._sock: socket.socket | None = None
    self._retry_after: dict[str, tuple[float, float]] = {}

  def _managed_objects(self) -> dict:
    message = new_method_call(DBusAddress("/", bus_name=BLUEZ, interface=OBJECT_MANAGER), "GetManagedObjects")
    reply = self.router.send_and_get_reply(message, timeout=15.0)
    if reply.header.message_type == MessageType.error:
      raise RuntimeError(str(reply.body[0] if reply.body else reply.header.fields.get(HeaderFields.error_name, "failed")))
    return unwrap_variant(reply.body[0]) if reply.body else {}

  @staticmethod
  def _publish_status(address: str, last_data: float | None, last_fix: float | None) -> None:
    # Only feeds the Bluetooth settings screen; never let it break the GPS link.
    try:
      write_phone_status(address, last_data, last_fix)
    except OSError:
      pass

  def _close_connection(self, reason: str) -> None:
    with self._state_lock:
      sock, self._sock = self._sock, None
    if sock is None:
      return
    try:
      sock.close()
    except OSError:
      pass
    clear_phone_fix()
    clear_phone_fix(PHONE_GPS_STATUS_PATH)
    cloudlog.warning(f"phone_gpsd: disconnected ({reason})")

  def _read_loop(self, sock: socket.socket, address: str, initial: bytes) -> None:
    accumulator = NmeaAccumulator()
    sock.settimeout(1.0)
    last_data = time.monotonic()
    last_fix: float | None = None
    last_status = 0.0
    first_fix = True
    reason = "stopped"
    data = initial
    try:
      while not self._stop.is_set():
        with self._state_lock:
          if self._sock is not sock:
            return
        now = time.monotonic()
        publish_now = False
        if data:
          last_data = now
          for fix in accumulator.feed_bytes(data):
            write_phone_fix(fix)
            last_fix = now
            if first_fix:
              first_fix = False
              publish_now = True
              cloudlog.warning(f"phone_gpsd: first fix {fix['latitude']:.5f},{fix['longitude']:.5f} sats={fix['satellites']}")
        elif now - last_data > STALL_TIMEOUT_S:
          reason = "no data"
          break
        if publish_now or now - last_status >= 1.0:
          last_status = now
          self._publish_status(address, last_data, last_fix)
        try:
          data = sock.recv(4096)
        except TimeoutError:
          data = b""
          continue
        if not data:
          reason = "closed by phone"
          break
    except OSError as error:
      reason = error.strerror or str(error)
    self._close_connection(reason)

  def _connect_phone(self, address: str, name: str) -> bool:
    ports = browse_serial_ports(address)
    if not ports:
      raise RuntimeError("phone offers no serial port - is the GPS app's Bluetooth stream running?")
    tried = []
    for service, channel in ports:
      sock, initial, failure = probe_nmea(address, channel)
      if sock is None:
        tried.append(f"{service or '?'} ch{channel}: {failure}")
        continue
      with self._state_lock:
        self._sock = sock
      cloudlog.warning(f"phone_gpsd: connected to {name} via \"{service}\" (channel {channel})")
      self._publish_status(address, time.monotonic(), None)
      threading.Thread(target=self._read_loop, args=(sock, address, initial), daemon=True).start()
      return True
    raise RuntimeError("no serial port sent NMEA (" + "; ".join(tried) + ")")

  def _connect_candidates(self) -> None:
    objects = self._managed_objects()
    # Runs offroad too, so stay off the radio while bluetooth_managerd is scanning for devices to pair.
    if any(interfaces.get(ADAPTER_IFACE, {}).get("Discovering", False) for interfaces in objects.values()):
      return
    now = time.monotonic()
    for path, interfaces in objects.items():
      props = interfaces.get(DEVICE_IFACE)
      if props is None or not is_phone_candidate(props):
        continue
      delay, retry_at = self._retry_after.get(path, (0.0, 0.0))
      if now < retry_at:
        continue
      name = str(props.get("Alias") or props.get("Address") or path)
      try:
        if self._connect_phone(str(props["Address"]), name):
          self._retry_after.pop(path, None)
          return
      except Exception as error:
        delay = min(RETRY_MAX_S, max(RETRY_MIN_S, delay * 2))
        self._retry_after[path] = (delay, time.monotonic() + delay)
        cloudlog.warning(f"phone_gpsd: {name}: {error}; retry in {delay:.0f}s")

  def run(self) -> None:
    clear_phone_fix()
    clear_phone_fix(PHONE_GPS_STATUS_PATH)
    while not self._stop.is_set():
      try:
        with self._state_lock:
          connected = self._sock is not None
        if not connected:
          self._connect_candidates()
      except Exception:
        # BlueZ restarting or the adapter powering down; keep trying rather than exiting.
        cloudlog.exception("phone_gpsd: loop error")
      self._stop.wait(CONNECT_POLL_S)

  def stop(self) -> None:
    self._stop.set()
    if self._stopped:
      return
    self._stopped = True
    self._close_connection("shutting down")
    self.router.close()


def main() -> None:
  daemon = PhoneGpsDaemon()

  def handle_signal(_signum, _frame):
    # Only flag it here; run() returns within CONNECT_POLL_S and stop() then cleans up exactly once.
    daemon._stop.set()

  signal.signal(signal.SIGTERM, handle_signal)
  signal.signal(signal.SIGINT, handle_signal)
  daemon.run()
  daemon.stop()


if __name__ == "__main__":
  main()
