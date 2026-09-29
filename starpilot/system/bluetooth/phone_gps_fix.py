"""Phone GPS fallback: NMEA parsing and the fix hand-off between phone_gpsd and qcomgpsd.

The comma's own GNSS receiver is badly desensed by the eGPU's USB 3 link and can take minutes to get a
fix (or never get one). A phone running an NMEA-over-Bluetooth app almost always already has a fix, so
phone_gpsd reads its stream and qcomgpsd publishes it on gpsLocation whenever the modem has nothing.

This module is deliberately stdlib-only: qcomgpsd imports it, and nothing here may be able to take
qcomgpsd down. Every reader path swallows errors and returns None.
"""
import datetime
import json
import math
import os
import time

PHONE_GPS_FIX_PATH = "/dev/shm/starpilot_phone_gps.json"
# Link state for the Bluetooth settings screen, written by phone_gpsd, read by bluetooth_managerd.
PHONE_GPS_STATUS_PATH = "/dev/shm/starpilot_phone_gps_status.json"

# The phone app sends NMEA at ~1Hz; this long without bytes (or without a valid fix) counts as stale.
PHONE_STATUS_STALE_S = 5.0

# qcomgpsd only substitutes a phone fix this recent. Phones emit at 1Hz, so this tolerates a couple of
# dropped epochs without ever publishing a position the car has already driven away from.
PHONE_FIX_MAX_AGE_S = 3.0

KNOTS_TO_MS = 0.514444


def nmea_checksum_ok(sentence: str) -> bool:
  sentence = sentence.strip()
  if not sentence.startswith("$") or "*" not in sentence:
    return False
  body, _, checksum = sentence[1:].partition("*")
  if len(checksum) < 2:
    return False
  calculated = 0
  for char in body:
    calculated ^= ord(char)
  try:
    return calculated == int(checksum[:2], 16)
  except ValueError:
    return False


def _coordinate(value: str, hemisphere: str) -> float | None:
  # NMEA packs coordinates as (d)ddmm.mmmm; the degree digits are everything before the last two
  # integer digits.
  if not value or hemisphere not in ("N", "S", "E", "W"):
    return None
  try:
    dot = value.index(".") if "." in value else len(value)
    degrees = float(value[:dot - 2])
    minutes = float(value[dot - 2:])
  except ValueError:
    return None
  if not 0.0 <= minutes < 60.0:
    return None
  result = degrees + minutes / 60.0
  return -result if hemisphere in ("S", "W") else result


def _float(value: str) -> float | None:
  try:
    result = float(value)
  except (TypeError, ValueError):
    return None
  return result if math.isfinite(result) else None


def _unix_ms(date: str, clock: str) -> int | None:
  # RMC carries ddmmyy + hhmmss(.ss) in UTC.
  try:
    day, month, year = int(date[0:2]), int(date[2:4]), 2000 + int(date[4:6])
    hour, minute = int(clock[0:2]), int(clock[2:4])
    seconds = float(clock[4:])
    whole = int(seconds)
    stamp = datetime.datetime(year, month, day, hour, minute, whole, int(round((seconds - whole) * 1e6)),
                              tzinfo=datetime.UTC)
  except (ValueError, IndexError):
    return None
  return int(stamp.timestamp() * 1000)


class NmeaAccumulator:
  """Combines per-epoch RMC (position/speed/course/date) and GGA (fix quality/sats/HDOP/altitude).

  A fix is emitted on each valid RMC. GGA is merged in when it belongs to the same epoch, matched on
  the UTC time-of-day field, since apps differ on which of the two they send first.
  """

  def __init__(self):
    self._gga: dict | None = None
    self._gga_time = ""
    self._buffer = ""

  def feed_bytes(self, data: bytes) -> list[dict]:
    self._buffer += data.decode("ascii", errors="ignore")
    # Bound the buffer in case the stream is not NMEA at all.
    if len(self._buffer) > 8192:
      self._buffer = self._buffer[-1024:]
    fixes = []
    while "\n" in self._buffer:
      line, self._buffer = self._buffer.split("\n", 1)
      fix = self.feed_line(line)
      if fix is not None:
        fixes.append(fix)
    return fixes

  def feed_line(self, line: str) -> dict | None:
    line = line.strip()
    if not nmea_checksum_ok(line):
      return None
    fields = line[1:line.index("*")].split(",")
    kind = fields[0][-3:]
    if kind == "GGA":
      self._parse_gga(fields)
      return None
    if kind == "RMC":
      return self._parse_rmc(fields)
    return None

  def _parse_gga(self, fields: list[str]) -> None:
    if len(fields) < 10:
      return
    try:
      quality = int(fields[6] or 0)
    except ValueError:
      return
    try:
      satellites = int(fields[7] or 0)
    except ValueError:
      satellites = 0
    self._gga = {
      "quality": quality,
      "satellites": satellites,
      "hdop": _float(fields[8]),
      "altitude": _float(fields[9]),
    }
    self._gga_time = fields[1]

  def _parse_rmc(self, fields: list[str]) -> dict | None:
    if len(fields) < 10 or fields[2] != "A":
      return None
    latitude = _coordinate(fields[3], fields[4])
    longitude = _coordinate(fields[5], fields[6])
    unix_ms = _unix_ms(fields[9], fields[1])
    if latitude is None or longitude is None or unix_ms is None:
      return None
    if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
      return None

    speed_knots = _float(fields[7])
    course = _float(fields[8])
    gga = self._gga if self._gga is not None and self._gga_time == fields[1] else None
    if gga is not None and gga["quality"] <= 0:
      return None

    return {
      "latitude": latitude,
      "longitude": longitude,
      "altitude": gga["altitude"] if gga and gga["altitude"] is not None else 0.0,
      "speed": max(0.0, speed_knots * KNOTS_TO_MS) if speed_knots is not None else 0.0,
      "bearing_deg": course % 360.0 if course is not None else 0.0,
      "bearing_valid": course is not None,
      "unix_ms": unix_ms,
      "satellites": gga["satellites"] if gga else 0,
      "hdop": gga["hdop"] if gga else None,
    }


def write_phone_fix(fix: dict, path: str = PHONE_GPS_FIX_PATH, now: float | None = None) -> None:
  payload = dict(fix)
  # CLOCK_MONOTONIC is system-wide on Linux, so qcomgpsd can age this against its own clock.
  payload["mono"] = time.monotonic() if now is None else now
  tmp_path = f"{path}.tmp"
  with open(tmp_path, "w") as f:
    json.dump(payload, f)
  os.replace(tmp_path, path)


def clear_phone_fix(path: str = PHONE_GPS_FIX_PATH) -> None:
  try:
    os.remove(path)
  except OSError:
    pass


def read_phone_fix(path: str = PHONE_GPS_FIX_PATH, max_age: float = PHONE_FIX_MAX_AGE_S,
                   now: float | None = None) -> dict | None:
  try:
    with open(path) as f:
      fix = json.load(f)
    now = time.monotonic() if now is None else now
    age = now - float(fix["mono"])
    if not 0.0 <= age <= max_age:
      return None
    float(fix["latitude"])
    float(fix["longitude"])
    return fix
  except Exception:
    return None


def address_from_device_path(device_path: str) -> str:
  # /org/bluez/hci0/dev_D4_3A_2C_63_2A_50 -> D4:3A:2C:63:2A:50
  return device_path.rsplit("/", 1)[-1].removeprefix("dev_").replace("_", ":").upper()


def write_phone_status(address: str, last_data: float | None, last_fix: float | None,
                       path: str = PHONE_GPS_STATUS_PATH) -> None:
  tmp_path = f"{path}.tmp"
  with open(tmp_path, "w") as f:
    json.dump({"address": address.upper(), "last_data": last_data, "last_fix": last_fix}, f)
  os.replace(tmp_path, path)


def read_phone_status(path: str = PHONE_GPS_STATUS_PATH, now: float | None = None) -> tuple[str, str]:
  """Returns (address, state) where state is "streaming", "no_fix", "connected", or "" when not connected.

  "no_fix" means NMEA is arriving but the phone has no GPS lock yet (e.g. indoors), which is still proof
  the Bluetooth link works.
  """
  try:
    with open(path) as f:
      status = json.load(f)
    now = time.monotonic() if now is None else now
    address = str(status["address"]).upper()
    last_fix, last_data = status.get("last_fix"), status.get("last_data")
    if last_fix is not None and now - float(last_fix) <= PHONE_STATUS_STALE_S:
      return address, "streaming"
    if last_data is not None and now - float(last_data) <= PHONE_STATUS_STALE_S:
      return address, "no_fix"
    return address, "connected"
  except Exception:
    return "", ""


def phone_fix_fields(fix: dict) -> dict:
  """gpsLocation field values for a phone fix. Accuracies are conservative estimates from HDOP."""
  hdop = fix.get("hdop")
  horizontal = max(1.0, float(hdop) * 5.0) if hdop else 10.0
  speed = float(fix.get("speed", 0.0))
  bearing = float(fix.get("bearing_deg", 0.0))
  bearing_rad = math.radians(bearing)
  moving = speed > 1.0 and bool(fix.get("bearing_valid", False))
  return {
    "latitude": float(fix["latitude"]),
    "longitude": float(fix["longitude"]),
    "altitude": float(fix.get("altitude", 0.0)),
    "speed": speed,
    "bearingDeg": bearing,
    "unixTimestampMillis": int(fix["unix_ms"]),
    "vNED": [speed * math.cos(bearing_rad), speed * math.sin(bearing_rad), 0.0],
    "horizontalAccuracy": horizontal,
    "verticalAccuracy": horizontal * 1.5,
    "bearingAccuracyDeg": max(5.0, horizontal) if moving else 180.0,
    "speedAccuracy": 0.5,
    "hasFix": True,
    "satelliteCount": int(fix.get("satellites", 0)),
  }
