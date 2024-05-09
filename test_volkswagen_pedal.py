#!/usr/bin/env python3
import time
import argparse
from panda import Panda, CanHandle, McuType
from typing import Callable, Dict, List, Optional, Tuple
from opendbc.can.packer import CANPacker
#from panda.tests.libpanda import libpanda_py
class CANPackerPanda(CANPacker):
  def make_can_msg_panda(self, name_or_addr, bus, values, fix_checksum=None):
    msg = self.make_can_msg(name_or_addr, bus, values)
    if fix_checksum is not None:
      msg = fix_checksum(msg)
    addr, _, dat, bus = msg
    return libpanda_py.make_CANPacket(addr, bus, dat)

def crc8_pedal(data):
  crc = 0xFF    # standard init value
  poly = 0xD5   # standard crc8: x8+x7+x6+x4+x2+1
  size = len(data)
  for i in range(size - 1, -1, -1):
    crc ^= data[i]
    for _ in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc


def create_gas_interceptor_command(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  dat = packer.make_can_msg("GAS_COMMAND", 0, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  #return packer.make_can_msg("GAS_COMMAND", 0, values)
  return packer.pack(512, values)
def interceptor_gas_cmd(packer, gas: int):
    #packer: CANPackerPanda
    global cnt_gas_cmd
    values: dict[str, float | int] = {"ENABLE": True, "COUNTER_PEDAL": cnt_gas_cmd & 0xF}
    if gas > 0:
      values["GAS_COMMAND"] = gas * 255.
      values["GAS_COMMAND2"] = gas * 255.
    cnt_gas_cmd += 1
    dat = packer.make_can_msg("GAS_COMMAND", 0, values)[2]
    checksum = crc8_pedal(dat[:-1])
    values["CHECKSUM_PEDAL"] = checksum
    #return packer.make_can_msg("GAS_COMMAND", 0, values)
    return packer.pack(512, values)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Send a gas command to the gas interceptor')
  parser.add_argument('--gas', action='store_true')
  parser.add_argument("fn", type=str, nargs='?', help="flash file")
  args = parser.parse_args()

  p = Panda()
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  cnt_gas_cmd=0
  packer1 = CANPacker("vw_golf_mk4")
  #packer1 = CANPackerPanda("vw_golf_mk4")

  while 1:
    if len(p.can_recv()) == 0:
      break

  #if args.gas:
    #interceptor_gas_cmd1(3)
    #interceptor_gas_cmd(3)
    #addr, _, dat, bus = create_gas_interceptor_command(packer,1.02,cnt_gas_cmd)
    #addr, _, dat, bus = interceptor_gas_cmd(packer1,3)
    #print(addr,_, dat,bus)
    print(create_gas_interceptor_command(packer1,2,cnt_gas_cmd))
    #p.send_heartbeat()
    #p.can_send(0x200, dat, 0)
    #p.can_send_many(create_gas_interceptor_command(packer1,2,cnt_gas_cmd))
    cnt_gas_cmd+=1
    time.sleep(0.02)

    #p.can_send(0x200, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x02", 0)
    #exit(0)
  #else:
    #p.can_send(0x200, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)

  #if args.fn:
    #time.sleep(0.1)
    #print("flashing", args.fn)
    #code = open(args.fn, "rb").read()
    #Panda.flash_static(CanHandle(p, 0), code, mcu_type=McuType.F2)

  print("send gas command done")
