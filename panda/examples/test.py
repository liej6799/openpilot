from selfdrive.car import make_can_msg
from panda import Panda

panda = Panda()
panda.set_safety_mode(Panda.SAFETY_WULING)
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
panda.set_safety_mode(Panda.SAFETY_SILENT)
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
# 83 b0 00 1e 0b 00 60 bc
# set cruise on
panda.can_send(0x370, bytes([0xc3,0xb0,0x00,0x00,0x01,0x00,0x20,0x94]), 0)
panda.can_send(0x263, bytes([0xf5,0x78,0x60,0x40,0x00,0x00,0x00,0x0d]), 0)
panda.can_send(0x370, bytes([0x83,0x00,0x00,0x00,0x02,0x00,0x20,0x55]), 0)
panda.can_send(0x370, bytes([0x83,0xb0,0x00,0x1e,0x0b,0x00,0x60,0xbc]), 0)

03 b0 00 1e 1b 00 c0 ac
# work for set cruise aktif
panda.can_send(0x370, bytes([0x03,0xb0,0x00,0x1e,0x1b,0x00,0xc0,0xac]), 0)
c2 01 00 00 ac 90 02 01

03 b0 00 00 00 00 20 d3
panda.can_send(0x370, bytes([0x03,0xb0,0x00,0x00,0x00,0x00,0x20,0xd3]), 0)
panda.can_send(0x373, bytes([0xc1,0x01,0x00,0x00,0xac,0x90,0x02,0x01]), 0)

# set cruise off
# 83 b0 00 00 0c 00 00 3f
panda.can_send(0x370, bytes([0x83,0xb0,0x00,0x00,0x0c,0x00,0x00,0x3f]), 0)

#steer
# a7 e3 00 00 00 00 00 8a
# e0 09 00 00 00 00 00 e9
panda.can_send(0x225, bytes([0xa7,0xe3,0x00,0x00,0x00,0x00,0x00,0x8a]), 0)
# e0 19 00 00 00 00 00 f9
panda.can_send(0x225, bytes([0xe0,0x19,0x00,0x00,0x00,0x00,0x00,0xf9]), 0)
# e0 0f 00 00 00 00 00 ef
panda.can_send(0x225, bytes([0xe0,0x0f,0x00,0x00,0x00,0x00,0x00,0xef]), 0)

# lkd state enable 
# 06 0d 02 00 ac 90 02 53
# c6 3d 01 00 ac 90 02 42
panda.can_send(0x373, b"\x86\x0d\x02\x00\xac\x90\x02\x53", 0)
panda.can_send(0x373, b"\xc6\x3d\x01\x00\xac\x90\x02\x42", 0)
dumpsafety = panda.health()
print(f"\nsafety_mode: {dumpsafety['safety_mode']}")
print("If safety mode == 0? so ALLOUTPUT don't worked OR FLAG ALLOWDEBUG is missing\non Panda flash")


# steering control
panda.can_send(0x373, b"\xc6\x3d\x01\x00\xac\x90\x02\x42", 0)
panda.can_send(0x225, bytes([0xe0,0x0f,0x00,0x00,0x00,0x00,0x00,0xef]), 0)


46 f6 60 80 e2 40 c2 00
panda.can_send(0x1e5, bytes([0x46,0xf6,0x60,0x80,0xe2,0x40,0xc2,0x00]), 0)
panda.can_send(0x225, bytes([0xe0,0x0f,0x00,0x00,0x00,0x00,0x00,0xef]), 0)
panda.can_send(0x373, b"\xc6\x3d\x01\x00\xac\x90\x02\x42", 0)
panda.can_send(0x429, b"\x00\x08\x00\x00\x00\x00\x00\x00", 0)

0080000000000000
#work for set lka aktif
while 1:  
  panda.can_send(0x373, b"\xc6\x3d\x01\x00\xac\x90\x02\x42", 0)
  print("ok")

# set lka aktif
panda.can_send(0x373, b"\xc6\x3d\x01\x00\xac\x90\x02\x42", 0)

#work for lka grey
panda.can_send(0x373, b"\x82\x01\x00\x00\xac\x90\x02\xc1", 0)

#works lka green
panda.can_send(0x373, b"\x84\x3d\x01\x00\xac\x90\x02\x00", 0)
panda.can_send(0x373, b"\x44\x3d\x00\x00\xac\x90\x02\xbf", 0)

# matikan lkas indikator
panda.can_send(0x373, b"\x40\x01\x00\x00\xac\x90\x02\x7f", 0)


---
c6 3d 02 00 ac 90 02 43


0x4c1 
05 5c 74 4a 80 00 46 00
panda.can_send(0x370, bytes([0x83,0xb0,0x00,0x00,0x0c,0x00,0x00,0x3f]), 0)

panda.can_send(0x4c1, bytes([0x05,0x5c,0x74,0x4a,0x80,0x00,0x46,0x00]), 0)
panda.can_send(0x263, bytes([0xd5,0x78,0x00,0x40,0x00,0x00,0x00,0x4d]), 0)

820100 00 ac 90 02 c1
panda.can_send(0x373, b"\x82\x01\x00\x00\xac\x90\x02\xc1", 0)

# 26 b6 35 50 0f f9 40 a9
panda.can_send(0x269, b"\x26\xb6\x35\x50\x0f\xf9\x40\xa9", 0)

8f bb 00 00 00 14 f4 52
panda.can_send(0x260, b"\x8f\xbb\x00\x00\x00\x14\xf4\x52", 0)

# 43 b0 00 1e 70 00 60 e1
panda.can_send(0x370, b"\x43\xb0\x00\x1e\x70\x00\x60\xe1", 0)


#warning stiir work after adas enable
# 85 3d 01 00 ac 90 02 01
panda.can_send(0x373, b"\x85\x3d\x01\x00\xac\x90\x02\x01", 0)

-----
#set panda safety
def set_panda_safety():
  ret = car.CarParams.new_message()
  ret.safetyModel = car.CarParams.SafetyModel.toyota
  return ret




319.94  0x1e1 (481)     +b'80 20 00 00 00 00 00 a0'
320.16  0x373 (883)     -b'40 01 00 00 ac 90 02 7f'
panda.can_send(0x1e1, b"\x80\x20\x00\x00\x00\x00\x00\xa0", 0)



c2 b0 00 3c 30 00 60 3e
panda.can_send(0x370, b"\xc2\xb0\x00\x3c\x30\x00\x06\x3e", 0)
