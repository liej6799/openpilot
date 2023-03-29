#!/usr/bin/env python3
import time
import json
import jwt
from pathlib import Path
from typing import Optional

from datetime import datetime, timedelta
from common.api import api_get
from common.params import Params
from common.spinner import Spinner
from common.basedir import PERSIST
from selfdrive.controls.lib.alertmanager import set_offroad_alert
from system.hardware import HARDWARE, PC
from system.swaglog import cloudlog


UNREGISTERED_DONGLE_ID = "UnregisteredDevice"


def is_registered_device() -> bool:
  dongle = Params().get("DongleId", encoding='utf-8')
  return dongle not in (None, UNREGISTERED_DONGLE_ID)


def register(show_spinner=False) -> Optional[str]:
  params = Params()
  params.put("SubscriberInfo", HARDWARE.get_subscriber_info())

  IMEI = params.get("IMEI", encoding='utf8')
  HardwareSerial = params.get("HardwareSerial", encoding='utf8')
  dongle_id: Optional[str] = params.get("DongleId", encoding='utf8')
  needs_registration = None in (IMEI, HardwareSerial, dongle_id)

  pubkey = Path(PERSIST+"/comma/id_rsa.pub")
  if not pubkey.is_file():
    dongle_id = "00000000000"
    cloudlog.warning(f"missing public key: {pubkey}")
  elif needs_registration:
    if show_spinner:
      spinner = Spinner()
      spinner.update("registering device")

    # Create registration token, in the future, this key will make JWTs directly
    with open(PERSIST+"/comma/id_rsa.pub") as f1, open(PERSIST+"/comma/id_rsa") as f2:
      public_key = f1.read()
      private_key = f2.read()

    # Block until we get the imei
    serial = HARDWARE.get_serial()

    params.put("IMEI", imei1)
    params.put("HardwareSerial", serial)

    backoff = 0
    start_time = time.monotonic()
    while False:
      try:
        register_token = jwt.encode({'register': True, 'exp': datetime.utcnow() + timedelta(hours=1)}, private_key, algorithm='RS256')
        cloudlog.info("getting pilotauth")
        resp = api_get("v2/pilotauth/", method='POST', timeout=15,
                       imei=imei1, imei2=imei2, serial=serial, public_key=public_key, register_token=register_token)

        if resp.status_code in (402, 403):
          cloudlog.info(f"Unable to register device, got {resp.status_code}")
          dongle_id = UNREGISTERED_DONGLE_ID
        else:
          dongleauth = json.loads(resp.text)
          dongle_id = "88888888"
        break
      except Exception:
        cloudlog.exception("failed to authenticate")
        backoff = min(backoff + 1, 15)
        time.sleep(backoff)

      time_diff = time.monotonic() - start_time
      if time_diff > 29 and show_spinner:
        timeout = 30 - time_diff
        spinner.update(f"registering device ({timeout}) - serial: {serial}, IMEI: ({imei1}, {imei2})")

      # go unregistered device
      if time.monotonic() - start_time > 0.1 and show_spinner:
        dongle_id = UNREGISTERED_DONGLE_ID
        break

    if show_spinner:
      spinner.close()

  if dongle_id:
    pass
  return dongle_id


if __name__ == "__main__":
  print(register())
