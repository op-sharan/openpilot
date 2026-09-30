import functools
import socket
import threading

import pytest

import openpilot.starpilot.system.bluetooth.phone_gps as phone_gps
import openpilot.starpilot.system.bluetooth.phone_gps_fix as fix_module


def sentence(body: str) -> bytes:
  checksum = 0
  for char in body:
    checksum ^= ord(char)
  return f"${body}*{checksum:02X}\r\n".encode()


GGA = sentence("GNGGA,153012.00,2613.5790,N,09817.4880,W,1,11,0.8,34.2,M,-24.1,M,,")
RMC = sentence("GNRMC,153012.00,A,2613.5790,N,09817.4880,W,30.5,84.4,280926,,,A")


@pytest.fixture
def daemon(monkeypatch, tmp_path):
  fix_path, status_path = str(tmp_path / "fix.json"), str(tmp_path / "status.json")
  monkeypatch.setattr(phone_gps, "write_phone_fix", functools.partial(fix_module.write_phone_fix, path=fix_path))
  monkeypatch.setattr(phone_gps, "write_phone_status", functools.partial(fix_module.write_phone_status, path=status_path))
  monkeypatch.setattr(phone_gps, "PHONE_GPS_STATUS_PATH", status_path)
  monkeypatch.setattr(phone_gps, "clear_phone_fix", lambda path=fix_path: fix_module.clear_phone_fix(path))
  # No D-Bus: _connect_phone and _read_loop don't touch the router.
  d = object.__new__(phone_gps.PhoneGpsDaemon)
  d._state_lock = threading.Lock()
  d._stop = threading.Event()
  d._sock = None
  d._retry_after = {}
  d.fix_path, d.status_path = fix_path, status_path
  return d


def test_connect_skips_ports_that_do_not_speak_nmea(daemon, monkeypatch):
  monkeypatch.setattr(phone_gps, "browse_serial_ports", lambda _address: [("GPS NMEA Tether", 16), ("BT1", 21)])
  ours, theirs = socket.socketpair()
  probed = []

  def probe(_address, channel):
    probed.append(channel)
    return (None, b"", "Device or resource busy") if channel == 16 else (ours, GGA, "")
  monkeypatch.setattr(phone_gps, "probe_nmea", probe)
  monkeypatch.setattr(threading, "Thread", lambda **_kwargs: type("T", (), {"start": lambda self: None})())

  assert daemon._connect_phone("D4:3A:2C:63:2A:50", "Pixel 8 Pro")
  assert probed == [16, 21]
  assert daemon._sock is ours
  theirs.close()
  ours.close()


def test_connect_reports_every_failed_port(daemon, monkeypatch):
  monkeypatch.setattr(phone_gps, "browse_serial_ports", lambda _address: [("GPS NMEA Tether", 16), ("BT1", 21)])
  monkeypatch.setattr(phone_gps, "probe_nmea", lambda _address, channel: (None, b"", "Connection refused"))
  with pytest.raises(RuntimeError, match="GPS NMEA Tether ch16: Connection refused; BT1 ch21: Connection refused"):
    daemon._connect_phone("D4:3A:2C:63:2A:50", "Pixel 8 Pro")

  monkeypatch.setattr(phone_gps, "browse_serial_ports", lambda _address: [])
  with pytest.raises(RuntimeError, match="no serial port"):
    daemon._connect_phone("D4:3A:2C:63:2A:50", "Pixel 8 Pro")


def test_read_loop_writes_fixes_and_cleans_up_when_phone_closes(daemon):
  ours, theirs = socket.socketpair()
  daemon._sock = ours
  reader = threading.Thread(target=daemon._read_loop, args=(ours, "D4:3A:2C:63:2A:50", GGA))
  reader.start()
  theirs.sendall(RMC)

  for _ in range(50):
    if fix_module.read_phone_fix(daemon.fix_path) is not None:
      break
    threading.Event().wait(0.05)
  assert fix_module.read_phone_fix(daemon.fix_path)["satellites"] == 11
  assert fix_module.read_phone_status(daemon.status_path)[1] == "streaming"

  theirs.close()
  reader.join(timeout=5)
  assert not reader.is_alive()
  assert daemon._sock is None
  assert fix_module.read_phone_fix(daemon.fix_path) is None
  assert fix_module.read_phone_status(daemon.status_path) == ("", "")
