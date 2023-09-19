import os, sys, io
import M5
from M5 import *
from bleuart import *


title_APP = None
Label_RX = None
label_TX = None
rect0 = None
ble_periph = None


def setup():
  global title_APP, Label_RX, label_TX, rect0, ble_periph

  M5.begin()
  Widgets.fillScreen(0x222222)
  title_APP = Widgets.Title("CoreS3 UART demo", 3, 0xFFFFFF, 0x0000FF, Widgets.FONTS.DejaVu18)
  Label_RX = Widgets.Label("RX:", 5, 52, 1.0, 0xffffff, 0x222222, Widgets.FONTS.DejaVu18)
  label_TX = Widgets.Label("TX:", 6, 171, 1.0, 0xf51111, 0x222222, Widgets.FONTS.DejaVu18)
  rect0 = Widgets.Rectangle(200, 40, 100, 100, 0x9fe63d, 0xf4d06c)

  print('BLE UART Demo')
  ble_periph = BLEUARTServer(name='S3BLE-uart')


def loop():
  global title_APP, Label_RX, label_TX, rect0, ble_periph
  M5.update()
  try:
    print(str((ble_periph.any())))
    ble_periph.write('hello M5'+'\r\n')
  except:
    print('BLE UART write error!')


if __name__ == '__main__':
  try:
    setup()
    while True:
      loop()
  except (Exception, KeyboardInterrupt) as e:
    try:
      from utility import print_error_msg
      print_error_msg(e)
    except ImportError:
      print("please update to latest firmware")
