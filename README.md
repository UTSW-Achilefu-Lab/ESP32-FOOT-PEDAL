# CVG Footpedal

To ensure hands-free usage of the cvg augment reality surgical aid, we fabricated a cheap (~$80), wireless footpedal to control the system's application. Many surgeons are familiar with footpedal controls. USB connected footpedals are cheap (~$40) and available from many retailors. However, using a wired device in the operating theater creates a tripping hazard. To transform a wired footpedal into a wireless one, we utilized the esp32-s3-otg-usb development board. This specific developement board was choosen because several integrated peripherals additional to the esp32s3 system on a chip's integrated bluetooth and usb transceiver. Specifically, the battery charge controller, buck converter (battery to 3.3V), and boost converter (battery to 5V). The LCD Screen that comes with this dev kit is removed.

Software written for the esp32s3 first enables bluetooth (Espressif's port of Apache NimBLE) then waits for connection to a previously paired or new device.
Once wirelessly connected, the esp32 identifies itself as a HID-compliant mouse and power to the connected usb footpedal is enabled. This system on a chip communicates with the footpedal converting any inputs (keys: 'a', 'b', and 'c') into mouse clicks (left, middle, and right) which are wireless sent to the Raspberry Pi CM4 running the cancer vision goggles. 

No software modification is required for the Cancer Vision Goggle application.

|   what   |   why   |   $$$   |   link   |
| --- | --- | --- | --- | 
| usb footpedal | hands free input device medical stafff are familar with | ~$40 | [https://www.amazon.com/Programmable-Multifunctional-Ergonomic-Equipment-photoelectric/dp/B0B4SP6KCF](https://www.amazon.com/Programmable-Multifunctional-Ergonomic-Equipment-photoelectric/dp/B0B4SP6KCF) |
| microcontroller | integrates required peripherals, cheap | $32.45 | [https://www.digikey.com/en/products/detail/espressif-systems/ESP32-S3-USB-OTG/15822449](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-S3-USB-OTG/15822449) |
| Single cell Li-Ion battery | device power | $9.95 | [https://www.adafruit.com/product/258](https://www.adafruit.com/product/258) | 
| 3D printed enclosure | device protection | - | n/a |

> Note: Most usb input devices available today do not require a specific driver, but conform to the Human Interface Device (HID) Standard introduced in the mid 1990s.
The usb footpedal used in this project is no different and identifies itself as an HID-compliant keyboard (but it only has three keys ('a', 'b', and 'c')).


## Device Setup

### Software Required

- **TODO: link to esp-idf setup**

> See the [Getting Started Guide](https://idf.espressif.com/) for full
steps to configure and use ESP-IDF to build projects.

### Hardware Required

- ESP32 SoC DEV BOARD [ESP32-S3-USB-OTG](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-usb-otg/index.html)
- A micro-usb cable for power supply and programming
- **TODO: all the rest of the components required**

**How much does it cost?**
bill of materials

### TODO: Assembly

1. assembly instructions
2. picture of assembled device

### Build and Flash

Build the project and flash it to the board, then run monitor tool to
view serial output.

```sh
idf.py -p PORT flash monitor
```

> (To exit the serial monitor, type ``Ctrl-]``.)

### TODO: pairing instructions

## Troubleshooting

### TODO: any tips if the device is not connecting

### From Espressif

1. When using NimBLE stack, some iOS devices do not show the volume pop up.
To fix this, please set CONFIG_BT_NIMBLE_SM_LVL to value 2.
iOS needs Authenticated Pairing with Encryption to show up the pop ups.

## Example Output

```
I (353) BLE_INIT: Using main XTAL as clock source
I (363) BLE_INIT: Feature Config, ADV:1, BLE_50:1, DTM:1, SCAN:1, CCA:0, SMP:1, CONNECT:1
I (373) BLE_INIT: Bluetooth MAC: f4:12:fa:9b:64:2a
I (373) phy_init: phy_version 701,f4f1da3a,Mar  3 2025,15:50:10
I (413) HID_DEV_DEMO: setting ble device
I (423) HID_DEV_DEMO: BLE Host Task Started
I (423) NimBLE: GAP procedure initiated: stop advertising.

I (423) HID_DEV_BLE: START
I (423) main_task: Returned from app_main()
I (423) NimBLE: GAP procedure initiated: advertise; 
I (433) NimBLE: disc_mode=2
I (433) NimBLE:  adv_channel_map=0 own_addr_type=0 adv_filter_policy=0 adv_itvl_min=48 adv_itvl_max=80
I (443) NimBLE: 

I (5293) HID_DEV_BLE: CONNECT
I (5293) ESP_HID_GAP: connection established; status=0
I (5483) NimBLE: encryption change event; status=0 
########################################################################
BT hid mouse demo usage:
This auto clicks the left, right, and mouse wheel buttons
########################################################################

I (5503) main task: LEFT CLICK

I (5503) NimBLE: GATT procedure initiated: notify; 
I (5513) NimBLE: att_handle=37

I (5513) NimBLE: notify_tx event; conn_handle=1 attr_handle=37 status=0 is_indication=0
I (5523) ESP_HID_GAP: subscribe event; conn_handle=1 attr_handle=8 reason=3 prevn=0 curn=0 previ=0 curi=1

I (5533) main task: RELEASE
I (5533) NimBLE: GATT procedure initiated: notify; 
I (5533) NimBLE: att_handle=37

I (5543) NimBLE: notify_tx event; conn_handle=1 attr_handle=37 status=0 is_indication=0
I (5543) ESP_HID_GAP: subscribe event; conn_handle=1 attr_handle=18 reason=3 prevn=0 curn=1 previ=0 curi=0

I (5553) ESP_HID_GAP: subscribe event; conn_handle=1 attr_handle=37 reason=3 prevn=0 curn=1 previ=0 curi=0

I (5583) ESP_HID_GAP: mtu update event; conn_handle=1 cid=4 mtu=256
I (6543) main task: RIGHT CLICK

I (6543) NimBLE: GATT procedure initiated: notify; 
I (6543) NimBLE: att_handle=37

I (6543) NimBLE: notify_tx event; conn_handle=1 attr_handle=37 status=0 is_indication=0
I (6553) main task: RELEASE
I (6553) NimBLE: GATT procedure initiated: notify; 
I (6553) NimBLE: att_handle=37

I (6553) NimBLE: notify_tx event; conn_handle=1 attr_handle=37 status=0 is_indication=0
I (7563) main task: MIDDLE CLICK

I (7563) NimBLE: GATT procedure initiated: notify; 
I (7563) NimBLE: att_handle=37

I (7563) NimBLE: notify_tx event; conn_handle=1 attr_handle=37 status=0 is_indication=0
I (7573) main task: RELEASE
I (7573) NimBLE: GATT procedure initiated: notify; 
I (7573) NimBLE: att_handle=37
...

I (14713) NimBLE: notify_tx event; conn_handle=1 attr_handle=37 status=0 is_indication=0
I (15093) ESP_HID_GAP: subscribe event; conn_handle=1 attr_handle=8 reason=2 prevn=0 curn=0 previ=1 curi=0

I (15093) ESP_HID_GAP: subscribe event; conn_handle=1 attr_handle=18 reason=2 prevn=1 curn=0 previ=0 curi=0

I (15103) ESP_HID_GAP: subscribe event; conn_handle=1 attr_handle=37 reason=2 prevn=1 curn=0 previ=0 curi=0

I (15113) ESP_HID_GAP: disconnect; reason=531
I (15113) HID_DEV_BLE: DISCONNECT: UNKNOWN
I (15113) NimBLE: GAP procedure initiated: advertise; 
I (15123) NimBLE: disc_mode=2
I (15123) NimBLE:  adv_channel_map=0 own_addr_type=0 adv_filter_policy=0 adv_itvl_min=48 adv_itvl_max=80
I (15133) NimBLE: 
```


