from selfdrive.car import make_can_msg
from panda import Panda

# panda = Panda()

# dat = b"\x20\x00\xe0\x00\x00\x00\x2e"
# panda.can_send(0x225, dat, 0)

# dat = b"\xa0\x12\xe0\x00\x00\x00\xb2"
# panda.can_send(0x225, dat, 0)

# panda.can_send(0x225,  bytearray(b"\xa0\x12\xe0\x00\x00\x00\xb2"), 0)

# 200a00000000002a

#  bytearray(b"\xa0\x12\xe0\x00\x00\x00\xb2")
 
# data =b"\x43\xb0\x00\x1e\x7b\x00\x60\xec"


# while 1 :
#   panda.can_send(0x370, b"\x43\xb0\x00\x1e\x7b\x00\x60", 0)
#   print('okay')
# panda.can_send(0x370, b"\x43\xb0\x00\x1e\x70\x00\x60", 0)
# panda.can_send(0x370, b"\x43\xb0\x00\x1e\x60\x00\x20", 0)


# panda.can_send(0x225,  bytearray(b"\xa0\x12\xe0\x00\x00\x00\xb2"), 0)

# onkan adas 
#43b0000002002015

# result = bytes.fromhex("43b0000002002015")

# 0x370
#   dat = b"\x00\x00\x00\x00\x00\x00\x00"

# panda.can_send(0x370, b"\x43\xb0\x00\x00\x02\x00\x20\x15", 0)

# c3 b0 00 00 01 00 20 94
panda = Panda()
# panda.can_recv()
panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
panda.set_safety_mode(Panda.SAFETY_GM)
panda.set_safety_mode(Panda.SAFETY_WULING)
panda.set_safety_mode(Panda.SAFETY_NOOUTPUT)
# panda.set_power_save(0)
while 1:  
  panda.can_send(0x370, bytes([0xc3,0xb0,0x00,0x00,0x01,0x00,0x20,0x94]), 0)
  print("ok")

# 43b000000c0000ff

# d5782000400000ad
# 43b0000001002014

# 957820004000006d
# f5 78 60 00 40 00 00 0d
# 83 b0 00 00 02 00 20 55
# set cruise on
panda.can_send(0x263, bytes([0xf5,0x78,0x60,0x40,0x00,0x00,0x00,0x0d]), 0)
panda.can_send(0x370, bytes([0x83,0x00,0x00,0x00,0x02,0x00,0x20,0x55]), 0)

03 b0 00 1e 1b 00 c0 ac
panda.can_send(0x370, bytes([0x03,0xb0,0x00,0x1e,0x1b,0x00,0xc0,0xac]), 0)


# set cruise off
# 83 b0 00 00 0c 00 00 3f
panda.can_send(0x370, bytes([0x83,0xb0,0x00,0x00,0x0c,0x00,0x00,0x3f]), 0)

#steer
# a7 e3 00 00 00 00 00 8a
# e0 09 00 00 00 00 00 e9
panda.can_send(0x225, bytes([0xa7,0xe3,0x00,0x00,0x00,0x00,0x00,0x8a]), 0)


# lkd state enable 
# 06 0d 02 00 ac 90 02 53
panda.can_send(0x373, b"\x86\x0d\x02\x00\xac\x90\x02\x53", 0)
dumpsafety = panda.health()
print(f"\nsafety_mode: {dumpsafety['safety_mode']}")
print("If safety mode == 0? so ALLOUTPUT don't worked OR FLAG ALLOWDEBUG is missing\non Panda flash")