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

set_panda_safety()
