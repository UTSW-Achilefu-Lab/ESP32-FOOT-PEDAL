# CVG Footpedal

This device transforms a usb footpedal into a wireless (bluetooth) mouse.
The usb footpedal reports as a keyboard with keys: ["a", "b", "c"]).
The device communicates with the keyboard and when connected to a
bluetooth host identifies as an HID compliant mouse sends pedal presses
as mouse clicks (left, middle, and right).

## Device Setup

### Software Required

> See the [Getting Started Guide](https://idf.espressif.com/) for full
steps to configure and use ESP-IDF to build projects.

### Hardware Required

- ESP32 SoC DEV BOARD [ESP32-S3-USB-OTG](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-usb-otg/index.html)
- A micro-usb cable for power supply and programming

| Components             | Functions                                                                            | Cost   | Purchase Link                                                                                                                                                                                              |
| ---------------------- | ------------------------------------------------------------------------------------ | ------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| USB Foot Pedal         | A hands-free input device that medical staff are familiar with                       | ~$40   | [https://www.amazon.com/Programmable-Multifunctional-Ergonomic-Equipment-photoelectric/dp/B0B4SP6KCF](https://www.amazon.com/Programmable-Multifunctional-Ergonomic-Equipment-photoelectric/dp/B0B4SP6KCF) |
| ESP32-S3-USB-OTG           | Microcontroller with integrated peripherals for USB host, BLE, and battery charging. | $32.45 | [https://www.digikey.com/en/products/detail/espressif-systems/ESP32-S3-USB-OTG/15822449](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-S3-USB-OTG/15822449)                           |
| Li-Ion Polymer battery | 3.7V, 1200mAh battery to power the device                                            | $9.95  | [https://www.adafruit.com/product/258](https://www.adafruit.com/product/258)                                                                                                                               |
| 3D printed enclosure   | device protection                                                                    | \-     | n/a                                                                                                                                                                                                        |

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
### TODO: update with current debug log

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


