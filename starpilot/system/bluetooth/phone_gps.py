"""phone_gpsd: read a phone's GPS over Bluetooth SPP so qcomgpsd can fall back to it.

Apps like "GPS NMEA Tether" run an SPP (RFCOMM serial) server on the phone that streams NMEA 0183. BlueZ's
Profile API does the SDP lookup and RFCOMM connect for us and hands over the connected socket as a file
descriptor, so this needs no AF_BLUETOOTH support in Python. Each parsed fix goes to a small file in
/dev/shm (see phone_gps_fix.py); qcomgpsd publishes it on gpsLocation only while the modem has no fix.
"""
import os
import select
import signal
import threading
import time

from jeepney import DBusAddress, MatchRule, new_error, new_method_call, new_method_return
from jeepney.io.threading import DBusRouter, open_dbus_connection
from jeepney.low_level import HeaderFields, MessageType

from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.system.bluetooth.bluez import BLUEZ, DEVICE_IFACE, OBJECT_MANAGER, unwrap_variant
from openpilot.starpilot.system.bluetooth.phone_gps_fix import NmeaAccumulator, clear_phone_fix, write_phone_fix
from openpilot.starpilot.system.bluetooth.protocol import is_phone

SPP_UUID = "00001101-0000-1000-8000-00805f9b34fb"
PROFILE_PATH = "/link/firestar/starpilot/phone_gps"
PROFILE_IFACE = "org.bluez.Profile1"
PROFILE_MANAGER_IFACE = "org.bluez.ProfileManager1"

CONNECT_POLL_S = 2.0
RETRY_MIN_S = 15.0
RETRY_MAX_S = 120.0
# The app streams at ~1Hz, so this long with no bytes means the link or the app has died.
STALL_TIMEOUT_S = 10.0


def is_phone_candidate(props: dict) -> bool:
  if not props.get("Paired", False) or props.get("Blocked", False):
    return False
  uuids = {str(uuid).lower() for uuid in props.get("UUIDs", [])}
  return SPP_UUID in uuids or is_phone(int(props.get("Class", 0)), str(props.get("Icon", "")))


class PhoneGpsDaemon:
  def __init__(self):
    # enable_fds: BlueZ passes the connected RFCOMM socket to NewConnection as a unix fd.
    self.router = DBusRouter(open_dbus_connection(bus="SYSTEM", enable_fds=True))
    self._call_lock = threading.Lock()
    self._state_lock = threading.Lock()
    self._stop = threading.Event()
    self._stopped = False
    self._registered = False
    self._fd: int | None = None
    self._device_path = ""
    self._retry_after: dict[str, tuple[float, float]] = {}

    self._profile_filter = self.router.filter(MatchRule(type="method_call", interface=PROFILE_IFACE, path=PROFILE_PATH), bufsize=10)
    self._profile_queue = self._profile_filter.__enter__()
    self._profile_thread = threading.Thread(target=self._profile_loop, daemon=True)
    self._profile_thread.start()

  def _call(self, path: str, interface: str, member: str, signature: str | None = None, body: tuple = (), timeout: float = 15.0):
    address = DBusAddress(path, bus_name=BLUEZ, interface=interface)
    message = new_method_call(address, member, signature, body) if signature is not None else new_method_call(address, member)
    with self._call_lock:
      reply = self.router.send_and_get_reply(message, timeout=timeout)
    if reply.header.message_type == MessageType.error:
      raise RuntimeError(str(reply.body[0] if reply.body else reply.header.fields.get(HeaderFields.error_name, "failed")))
    return reply.body

  def _register_profile(self) -> None:
    options = {
      "Name": ("s", "StarPilot Phone GPS"),
      "Role": ("s", "client"),
      "AutoConnect": ("b", False),
    }
    try:
      self._call("/org/bluez", PROFILE_MANAGER_IFACE, "RegisterProfile", "osa{sv}", (PROFILE_PATH, SPP_UUID, options))
    except RuntimeError as error:
      if "alreadyexists" not in str(error).replace(" ", "").lower():
        raise
    self._registered = True
    cloudlog.warning("phone_gpsd: SPP client profile registered")

  def _profile_loop(self) -> None:
    while not self._stop.is_set():
      message = self._profile_queue.get()
      if message is None:
        break
      member = message.header.fields.get(HeaderFields.member, "")
      try:
        if member == "NewConnection":
          self._on_new_connection(str(message.body[0]), message.body[1])
        elif member == "RequestDisconnection":
          self._close_connection("phone requested disconnection")
        elif member == "Release":
          self._registered = False
        else:
          raise RuntimeError(f"Unsupported profile call: {member}")
        self.router.send(new_method_return(message))
      except Exception as error:
        cloudlog.exception(f"phone_gpsd: profile call {member} failed")
        try:
          self.router.send(new_error(message, "org.bluez.Error.Rejected", "s", (str(error),)))
        except Exception:
          pass

  def _on_new_connection(self, device_path: str, fd_obj) -> None:
    fd = fd_obj.to_raw_fd()
    with self._state_lock:
      if self._fd is not None:
        os.close(fd)
        return
      self._fd = fd
      self._device_path = device_path
    cloudlog.warning(f"phone_gpsd: connected to {device_path}")
    threading.Thread(target=self._read_loop, args=(fd,), daemon=True).start()

  def _close_connection(self, reason: str) -> None:
    with self._state_lock:
      fd, self._fd = self._fd, None
      device_path, self._device_path = self._device_path, ""
    if fd is None:
      return
    try:
      os.close(fd)
    except OSError:
      pass
    clear_phone_fix()
    cloudlog.warning(f"phone_gpsd: disconnected from {device_path} ({reason})")

  def _read_loop(self, fd: int) -> None:
    accumulator = NmeaAccumulator()
    last_data = time.monotonic()
    first_fix = True
    reason = "stopped"
    try:
      while not self._stop.is_set():
        with self._state_lock:
          if self._fd != fd:
            return
        readable, _, _ = select.select([fd], [], [], 1.0)
        now = time.monotonic()
        if not readable:
          if now - last_data > STALL_TIMEOUT_S:
            reason = "no data"
            break
          continue
        data = os.read(fd, 4096)
        if not data:
          reason = "closed by phone"
          break
        last_data = now
        for fix in accumulator.feed_bytes(data):
          write_phone_fix(fix)
          if first_fix:
            first_fix = False
            cloudlog.warning(f"phone_gpsd: first fix {fix['latitude']:.5f},{fix['longitude']:.5f} sats={fix['satellites']}")
    except OSError as error:
      reason = str(error)
    self._close_connection(reason)

  def _connect_candidates(self) -> None:
    body = self._call("/", OBJECT_MANAGER, "GetManagedObjects")
    objects = unwrap_variant(body[0]) if body else {}
    now = time.monotonic()
    for path, interfaces in objects.items():
      props = interfaces.get(DEVICE_IFACE)
      if props is None or not is_phone_candidate(props):
        continue
      delay, retry_at = self._retry_after.get(path, (0.0, 0.0))
      if now < retry_at:
        continue
      try:
        self._call(path, DEVICE_IFACE, "ConnectProfile", "s", (SPP_UUID,), timeout=25.0)
        self._retry_after.pop(path, None)
        return
      except Exception as error:
        delay = min(RETRY_MAX_S, max(RETRY_MIN_S, delay * 2))
        self._retry_after[path] = (delay, time.monotonic() + delay)
        cloudlog.warning(f"phone_gpsd: SPP connect to {props.get('Alias') or path} failed ({error}); retry in {delay:.0f}s")

  def run(self) -> None:
    clear_phone_fix()
    while not self._stop.is_set():
      try:
        if not self._registered:
          self._register_profile()
        with self._state_lock:
          connected = self._fd is not None
        if not connected:
          self._connect_candidates()
      except Exception:
        # BlueZ restarting or the adapter powering down; keep trying rather than exiting.
        self._registered = False
        cloudlog.exception("phone_gpsd: loop error")
      self._stop.wait(CONNECT_POLL_S)

  def stop(self) -> None:
    self._stop.set()
    if self._stopped:
      return
    self._stopped = True
    self._close_connection("shutting down")
    try:
      self._call("/org/bluez", PROFILE_MANAGER_IFACE, "UnregisterProfile", "o", (PROFILE_PATH,), timeout=5.0)
    except Exception:
      pass
    try:
      self._profile_queue.put_nowait(None)
    except Exception:
      pass
    self._profile_filter.__exit__(None, None, None)
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
