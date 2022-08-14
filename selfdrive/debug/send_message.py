#!/usr/bin/env python
from time import sleep
import zmq
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car import make_can_msg

from cereal import car
import cereal.messaging as messaging

#set panda safety
def set_panda_safety():
  ret = car.CarParams.new_message()
  ret.safetyModel = car.CarParams.SafetyModel.wuling
  return ret

#context = zmq.Context()
sendcan = messaging.pub_sock('sendcan')

command = []

def lock_door(sendcan):
  print("locking door")
  command.append(make_can_msg(0x373, b"\x82\x01\x00\x00\xac\x90\x02\xc1", 0))
  sendcan.send(can_list_to_can_capnp(command, msgtype='sendcan'))
  del command[:]
  
# set_panda_safety()
while True:
  lock_door(sendcan)
  sleep(1)