import datetime
import json

import pytest

from openpilot.starpilot.system.bluetooth.phone_gps_fix import (KNOTS_TO_MS, NmeaAccumulator, address_from_device_path, clear_phone_fix,
                                                                nmea_checksum_ok, phone_fix_fields, read_phone_fix, read_phone_status,
                                                                write_phone_fix, write_phone_status)


def sentence(body: str) -> str:
  checksum = 0
  for char in body:
    checksum ^= ord(char)
  return f"${body}*{checksum:02X}"


RMC = sentence("GNRMC,153012.00,A,2613.5790,N,09817.4880,W,30.5,84.4,280926,,,A")
GGA = sentence("GNGGA,153012.00,2613.5790,N,09817.4880,W,1,11,0.8,34.2,M,-24.1,M,,")


def test_checksum():
  assert nmea_checksum_ok(RMC)
  assert nmea_checksum_ok(RMC + "\r\n")
  assert not nmea_checksum_ok(RMC[:-2] + "00")
  assert not nmea_checksum_ok(RMC.replace("*", ""))
  assert not nmea_checksum_ok("GNRMC,no,dollar*00")


def test_rmc_with_matching_gga():
  acc = NmeaAccumulator()
  assert acc.feed_line(GGA) is None
  fix = acc.feed_line(RMC)
  assert fix is not None
  assert fix["latitude"] == pytest.approx(26 + 13.5790 / 60)
  assert fix["longitude"] == pytest.approx(-(98 + 17.4880 / 60))
  assert fix["speed"] == pytest.approx(30.5 * KNOTS_TO_MS)
  assert fix["bearing_deg"] == pytest.approx(84.4)
  assert fix["satellites"] == 11
  assert fix["hdop"] == pytest.approx(0.8)
  assert fix["altitude"] == pytest.approx(34.2)
  expected = datetime.datetime(2026, 9, 28, 15, 30, 12, tzinfo=datetime.UTC)
  assert fix["unix_ms"] == int(expected.timestamp() * 1000)


def test_rmc_alone_still_gives_a_fix():
  fix = NmeaAccumulator().feed_line(RMC)
  assert fix is not None
  assert fix["satellites"] == 0 and fix["hdop"] is None


def test_gga_from_another_epoch_is_not_merged():
  acc = NmeaAccumulator()
  acc.feed_line(sentence("GNGGA,153011.00,2613.5790,N,09817.4880,W,1,4,9.9,1.0,M,,M,,"))
  fix = acc.feed_line(RMC)
  assert fix["satellites"] == 0


def test_void_rmc_and_no_fix_gga_rejected():
  assert NmeaAccumulator().feed_line(sentence("GPRMC,153012.00,V,,,,,,,280926,,,N")) is None
  acc = NmeaAccumulator()
  acc.feed_line(sentence("GNGGA,153012.00,2613.5790,N,09817.4880,W,0,0,,,M,,M,,"))
  assert acc.feed_line(RMC) is None


def test_bad_checksum_and_other_sentences_ignored():
  acc = NmeaAccumulator()
  assert acc.feed_line(RMC[:-2] + "00") is None
  assert acc.feed_line(sentence("GPGSV,3,1,11,01,40,083,46")) is None


def test_feed_bytes_across_chunk_boundaries():
  acc = NmeaAccumulator()
  data = (GGA + "\r\n" + RMC + "\r\n").encode()
  fixes = []
  for i in range(0, len(data), 7):
    fixes += acc.feed_bytes(data[i:i + 7])
  assert len(fixes) == 1
  assert fixes[0]["satellites"] == 11


def test_fix_file_roundtrip_and_staleness(tmp_path):
  path = str(tmp_path / "phone_gps.json")
  fix = NmeaAccumulator().feed_line(RMC)
  write_phone_fix(fix, path, now=100.0)
  assert read_phone_fix(path, now=101.0)["latitude"] == pytest.approx(fix["latitude"])
  assert read_phone_fix(path, now=104.0) is None
  assert read_phone_fix(path, now=99.0) is None
  clear_phone_fix(path)
  assert read_phone_fix(path, now=101.0) is None


def test_read_never_raises(tmp_path):
  path = tmp_path / "phone_gps.json"
  path.write_text("{not json")
  assert read_phone_fix(str(path), now=0.0) is None
  path.write_text(json.dumps({"mono": 0.0}))
  assert read_phone_fix(str(path), now=0.0) is None


def test_phone_status_states(tmp_path):
  path = str(tmp_path / "status.json")
  assert read_phone_status(path, now=0.0) == ("", "")

  assert address_from_device_path("/org/bluez/hci0/dev_d4_3a_2c_63_2a_50") == "D4:3A:2C:63:2A:50"
  write_phone_status("d4:3a:2c:63:2a:50", None, None, path)
  assert read_phone_status(path, now=100.0) == ("D4:3A:2C:63:2A:50", "connected")

  write_phone_status("D4:3A:2C:63:2A:50", 100.0, None, path)
  assert read_phone_status(path, now=101.0)[1] == "no_fix"

  write_phone_status("D4:3A:2C:63:2A:50", 100.0, 100.0, path)
  assert read_phone_status(path, now=101.0)[1] == "streaming"
  # Data stopped long ago but the link file is still there: connected, not streaming.
  assert read_phone_status(path, now=200.0)[1] == "connected"

  clear_phone_fix(path)
  assert read_phone_status(path, now=101.0) == ("", "")


def test_phone_fix_fields():
  fields = phone_fix_fields(NmeaAccumulator().feed_line(RMC))
  assert fields["hasFix"]
  assert fields["horizontalAccuracy"] == pytest.approx(10.0)
  assert fields["satelliteCount"] == 0
  assert fields["bearingAccuracyDeg"] < 180.0

  stopped = phone_fix_fields(NmeaAccumulator().feed_line(sentence("GNRMC,153012.00,A,2613.5790,N,09817.4880,W,0.0,,280926,,,A")))
  assert stopped["bearingAccuracyDeg"] == 180.0
  assert stopped["vNED"] == pytest.approx([0.0, 0.0, 0.0])
