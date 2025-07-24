/* footpedal_main.c
 *
 * Modified and combined from example programs provided by espressif
 *
 * See:
 * - esp-idf/examples/bluetooth/esp_hid_device esp_hid_device example
 * - esp-idf/examples/peripherals/usb/host/hid
 *
 * Copyright (C) 2025 Ian Zurutuza, Brian Blasi 
 
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "esp_hid_gap.h"
#include "esp_hidd.h"
#include "portmacro.h"

#include <stdbool.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "errno.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"

#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"

static const char *TAG_HID_HOST = "USB_HID_HOST";
static const char *TAG_BT_HID_DEVICE = "USB_BT_HID_DEVICE";
/* GPIO Pin number for quit from example logic */
#define APP_QUIT_PIN GPIO_NUM_0
void send_mouse(uint8_t, char, char, char);
QueueHandle_t app_event_queue = NULL;

/**
 * @brief APP event group
 *
 * Application logic can be different. There is a one among other ways to
 * distingiush the event by application event group. In this example we have two
 * event groups: APP_EVENT            - General event, which is APP_QUIT_PIN
 * press event (Generally, it is IO0). APP_EVENT_HID_HOST   - HID Host Driver
 * event, such as device connection/disconnection or input report.
 */
typedef enum { APP_EVENT = 0, APP_EVENT_HID_HOST } app_event_group_t;

/**
 * @brief APP event queue
 *
 * This event is used for delivering the HID Host event from callback to a task.
 */
typedef struct {
  app_event_group_t event_group;
  /* HID Host - Device related info */
  struct {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t event;
    void *arg;
  } hid_host_device;
} app_event_queue_t;

/**
 * @brief HID Protocol string names
 */
static const char *hid_proto_name_str[] = {
    "KEYBOARD",
};

/**
 * @brief Key event
 */
typedef struct {
  enum key_state { KEY_STATE_PRESSED = 0x00, KEY_STATE_RELEASED = 0x01 } state;
  uint8_t modifier;
  uint8_t key_code;
} key_event_t;

/* Main char symbol for ENTER key */
#define KEYBOARD_ENTER_MAIN_CHAR '\r'
/* When set to 1 pressing ENTER will be extending with LineFeed during serial
 * debug output */
#define KEYBOARD_ENTER_LF_EXTEND 1

/**
 * @brief Scancode to ascii table
 */
const uint8_t keycode2ascii[57][2] = {
    {0, 0},     /* HID_KEY_NO_PRESS        */
    {0, 0},     /* HID_KEY_ROLLOVER        */
    {0, 0},     /* HID_KEY_POST_FAIL       */
    {0, 0},     /* HID_KEY_ERROR_UNDEFINED */
    {'a', 'A'}, /* HID_KEY_A               */
    {'b', 'B'}, /* HID_KEY_B               */
    {'c', 'C'}, /* HID_KEY_C               */
};

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto) {
  static hid_protocol_t prev_proto_output = -1;

  if (prev_proto_output != proto) {
    prev_proto_output = proto;
    printf("\r\n");
    if (proto == HID_PROTOCOL_MOUSE) {
      printf("Mouse\r\n");
    } else if (proto == HID_PROTOCOL_KEYBOARD) {
      printf("Keyboard\r\n");
    } else {
      printf("Generic\r\n");
    }
    fflush(stdout);
  }
}

/**
 * @brief HID Keyboard modifier verification for capitalization application
 * (right or left shift)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right shift)
 * @return false Modifier was not pressed (left or right shift)
 *
 */
static inline bool hid_keyboard_is_modifier_shift(uint8_t modifier) {
  if (((modifier & HID_LEFT_SHIFT) == HID_LEFT_SHIFT) ||
      ((modifier & HID_RIGHT_SHIFT) == HID_RIGHT_SHIFT)) {
    return true;
  }
  return false;
}

/**
 * @brief HID Keyboard get char symbol from key code
 *
 * @param[in] modifier  Keyboard modifier data
 * @param[in] key_code  Keyboard key code
 * @param[in] key_char  Pointer to key char data
 *
 * @return true  Key scancode converted successfully
 * @return false Key scancode unknown
 */
static inline bool hid_keyboard_get_char(uint8_t modifier, uint8_t key_code,
                                         unsigned char *key_char) {
  uint8_t mod = (hid_keyboard_is_modifier_shift(modifier)) ? 1 : 0;

  if ((key_code >= HID_KEY_A) && (key_code <= HID_KEY_SLASH)) {
    *key_char = keycode2ascii[key_code][mod];
  } else {
    // All other key pressed
    return false;
  }

  return true;
}

/**
 * @brief HID Keyboard print char symbol
 *
 * @param[in] key_char  Keyboard char to stdout
 */
static inline void hid_keyboard_print_char(unsigned int key_char) {
  if (!!key_char) {
    ESP_LOGI(TAG_HID_HOST, "key = \"%c\"\n", key_char);
#if (KEYBOARD_ENTER_LF_EXTEND)
    if (KEYBOARD_ENTER_MAIN_CHAR == key_char) {
      putchar('\n');
    }
#endif // KEYBOARD_ENTER_LF_EXTEND
    fflush(stdout);
  }
}

/**
 * @brief Key Event. Key event with the key code, state and modifier.
 *
 * @param[in] key_event Pointer to Key Event structure
 *
 */
static void key_event_callback(key_event_t *key_event) {
  unsigned char key_char;

  hid_print_new_device_report_header(HID_PROTOCOL_KEYBOARD);

  if (KEY_STATE_PRESSED == key_event->state) {
    if (hid_keyboard_get_char(key_event->modifier, key_event->key_code,
                              &key_char)) {

      hid_keyboard_print_char(key_char);
      switch (key_char) {
      case 'a':
        send_mouse(1, 0, 0, 0);
        break;
      case 'b':
        send_mouse(4, 0, 0, 0);
        break;
      case 'c':
        send_mouse(2, 0, 0, 0);
        break;
      default:
        break;
      }
    }
    // This will make it not loop
  } else if (KEY_STATE_RELEASED == key_event->state) {
    send_mouse(0, 0, 0, 0);
  }
}

/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src, uint8_t key,
                             unsigned int length) {
  for (unsigned int i = 0; i < length; i++) {
    if (src[i] == key) {
      return true;
    }
  }
  return false;
}

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data,
                                              const int length) {
  hid_keyboard_input_report_boot_t *kb_report =
      (hid_keyboard_input_report_boot_t *)data;

  if (length < sizeof(hid_keyboard_input_report_boot_t)) {
    return;
  }

  static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};
  key_event_t key_event;

  for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {

    // key has been released verification
    if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED &&
        !key_found(kb_report->key, prev_keys[i], HID_KEYBOARD_KEY_MAX)) {
      key_event.key_code = prev_keys[i];
      key_event.modifier = 0;
      key_event.state = KEY_STATE_RELEASED;
      key_event_callback(&key_event);
    }

    // key has been pressed verification
    if (kb_report->key[i] > HID_KEY_ERROR_UNDEFINED &&
        !key_found(prev_keys, kb_report->key[i], HID_KEYBOARD_KEY_MAX)) {
      key_event.key_code = kb_report->key[i];
      key_event.modifier = kb_report->modifier.val;
      key_event.state = KEY_STATE_PRESSED;
      key_event_callback(&key_event);
    }
  }

  memcpy(prev_keys, &kb_report->key, HID_KEYBOARD_KEY_MAX);
}

/**
 * @brief USB HID Host Mouse Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_mouse_report_callback(const uint8_t *const data,
                                           const int length) {
  hid_mouse_input_report_boot_t *mouse_report =
      (hid_mouse_input_report_boot_t *)data;

  if (length < sizeof(hid_mouse_input_report_boot_t)) {
    return;
  }

  static int x_pos = 0;
  static int y_pos = 0;

  // Calculate absolute position from displacement

  hid_print_new_device_report_header(HID_PROTOCOL_MOUSE);

  printf("X: %06d\tY: %06d\t|%c|%c|\r", x_pos, y_pos,
         (mouse_report->buttons.button1 ? 'o' : ' '),
         (mouse_report->buttons.button2 ? 'o' : ' '));
  fflush(stdout);
}

/**
 * @brief USB HID Host Generic Interface report callback handler
 * 'generic' means anything else than mouse or keyboard
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_generic_report_callback(const uint8_t *const data,
                                             const int length) {
  hid_print_new_device_report_header(HID_PROTOCOL_NONE);
  for (int i = 0; i < length; i++) {
    printf("%02X", data[i]);
  }
  putchar('\r');
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                 const hid_host_interface_event_t event,
                                 void *arg) {
  uint8_t data[64] = {0};
  size_t data_length = 0;
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
    ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(
        hid_device_handle, data, 64, &data_length));

    if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
      if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
        hid_host_keyboard_report_callback(data, data_length);
      } else if (HID_PROTOCOL_MOUSE == dev_params.proto) {
        hid_host_mouse_report_callback(data, data_length);
      }
    } else {
      hid_host_generic_report_callback(data, data_length);
    }

    break;
  case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
    ESP_LOGI(TAG_HID_HOST, "HID Device, protocol '%s' DISCONNECTED",
             hid_proto_name_str[dev_params.proto]);
    ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
    break;
  case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
    ESP_LOGI(TAG_HID_HOST, "HID Device, protocol '%s' TRANSFER_ERROR",
             hid_proto_name_str[dev_params.proto]);
    break;
  default:
    ESP_LOGE(TAG_HID_HOST, "HID Device, protocol '%s' Unhandled event",
             hid_proto_name_str[dev_params.proto]);
    break;
  }
}

/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                           const hid_host_driver_event_t event, void *arg) {
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_DRIVER_EVENT_CONNECTED:
    ESP_LOGI(TAG_HID_HOST, "HID Device, protocol '%s' CONNECTED",
             hid_proto_name_str[dev_params.proto]);

    const hid_host_device_config_t dev_config = {
        .callback = hid_host_interface_callback, .callback_arg = NULL};

    ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
    if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
      ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle,
                                                     HID_REPORT_PROTOCOL_BOOT));
      if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
        ESP_ERROR_CHECK(hid_class_request_set_idle(hid_device_handle, 0, 0));
      }
    }
    ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
    break;
  default:
    break;
  }
}

/**
 * @brief Start USB Host install and handle common USB host library events while
 * app pin not low
 *
 * @param[in] arg  Not used
 */
static void usb_lib_task(void *arg) {
  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };

  ESP_ERROR_CHECK(usb_host_install(&host_config));
  xTaskNotifyGive(arg);

  while (true) {
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    // In this example, there is only one client registered
    // So, once we deregister the client, this call must succeed with ESP_OK
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      ESP_ERROR_CHECK(usb_host_device_free_all());
      break;
    }
  }

  ESP_LOGI(TAG_HID_HOST, "USB shutdown");
  // Clean up USB Host
  vTaskDelay(10); // Short delay to allow clients clean-up
  ESP_ERROR_CHECK(usb_host_uninstall());
  vTaskDelete(NULL);
}

/**
 * @brief BOOT button pressed callback
 *
 * Signal application to exit the HID Host task
 *
 * @param[in] arg Unused
 */
static void gpio_isr_cb(void *arg) {
  BaseType_t xTaskWoken = pdFALSE;
  const app_event_queue_t evt_queue = {
      .event_group = APP_EVENT,
  };

  if (app_event_queue) {
    xQueueSendFromISR(app_event_queue, &evt_queue, &xTaskWoken);
  }

  if (xTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

/**
 * @brief HID Host Device callback
 *
 * Puts new HID Device event to the queue
 *
 * @param[in] hid_device_handle HID Device handle
 * @param[in] event             HID Device event
 * @param[in] arg               Not used
 */
void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                              const hid_host_driver_event_t event, void *arg) {
  const app_event_queue_t evt_queue = {.event_group = APP_EVENT_HID_HOST,
                                       // HID Host Device related info
                                       .hid_host_device.handle =
                                           hid_device_handle,
                                       .hid_host_device.event = event,
                                       .hid_host_device.arg = arg};

  if (app_event_queue) {
    xQueueSend(app_event_queue, &evt_queue, 0);
  }
}

typedef struct {
  TaskHandle_t task_hdl;
  esp_hidd_dev_t *hid_dev;
  uint8_t protocol_mode;
  uint8_t *buffer;
} local_param_t;

static local_param_t s_ble_hid_param = {0};

const unsigned char mouseReportMap[] = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x02, // USAGE (Mouse)
    0xa1, 0x01, // COLLECTION (Application)

    0x09, 0x01, //   USAGE (Pointer)
    0xa1, 0x00, //   COLLECTION (Physical)

    0x05, 0x09, //     USAGE_PAGE (Button)
    0x19, 0x01, //     USAGE_MINIMUM (Button 1)
    0x29, 0x03, //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00, //     LOGICAL_MINIMUM (0)
    0x25, 0x01, //     LOGICAL_MAXIMUM (1)
    0x95, 0x03, //     REPORT_COUNT (3)
    0x75, 0x01, //     REPORT_SIZE (1)
    0x81, 0x02, //     INPUT (Data,Var,Abs)
    0x95, 0x01, //     REPORT_COUNT (1)
    0x75, 0x05, //     REPORT_SIZE (5)
    0x81, 0x03, //     INPUT (Cnst,Var,Abs)

    0x05, 0x01, //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30, //     USAGE (X)
    0x09, 0x31, //     USAGE (Y)
    0x09, 0x38, //     USAGE (Wheel)
    0x15, 0x81, //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f, //     LOGICAL_MAXIMUM (127)
    0x75, 0x08, //     REPORT_SIZE (8)
    0x95, 0x03, //     REPORT_COUNT (3)
    0x81, 0x06, //     INPUT (Data,Var,Rel)

    0xc0, //   END_COLLECTION
    0xc0  // END_COLLECTION
};

// send the buttons, change in x, and change in y
void send_mouse(uint8_t buttons, char dx, char dy, char wheel) {
  static uint8_t buffer[4] = {0};
  buffer[0] = buttons;
  buffer[1] = dx;
  buffer[2] = dy;
  buffer[3] = wheel;
  esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 0, 0, buffer, 4);
}

void ble_hid_demo_task_mouse(void *pvParameters) {
  BaseType_t task_created;
  app_event_queue_t evt_queue;

  ESP_LOGI(TAG_BT_HID_DEVICE, "Start USBHID task");
  ESP_LOGI(TAG_HID_HOST, "HID Host example");

  // configure pins for enabling usb host data comm and power
  const gpio_config_t boost_pin = {
      .pin_bit_mask = BIT64(GPIO_NUM_13),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&boost_pin));
  gpio_set_level(13, 1);

  const gpio_config_t ilimit_pin = {
      .pin_bit_mask = BIT64(GPIO_NUM_17),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&ilimit_pin));
  gpio_set_level(17, 1);

  const gpio_config_t host_pin = {
      .pin_bit_mask = BIT64(GPIO_NUM_18),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&host_pin));
  gpio_set_level(18, 1);

  // Init BOOT button: Pressing the button simulates app request to exit
  // It will disconnect the USB device and uninstall the HID driver and USB Host
  // Lib
  const gpio_config_t input_pin = {
      .pin_bit_mask = BIT64(APP_QUIT_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
  };
  ESP_ERROR_CHECK(gpio_config(&input_pin));
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
  ESP_ERROR_CHECK(gpio_isr_handler_add(APP_QUIT_PIN, gpio_isr_cb, NULL));

  /*
   * Create usb_lib_task to:
   * - initialize USB Host library
   * - Handle USB Host events while APP pin in in HIGH state
   */
  task_created =
      xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096,
                              xTaskGetCurrentTaskHandle(), 2, NULL, 0);
  assert(task_created == pdTRUE);

  // Wait for notification from usb_lib_task to proceed
  ulTaskNotifyTake(false, 1000);

  /*
   * HID host driver configuration
   * - create background task for handling low level event inside the HID driver
   * - provide the device callback to get new HID Device connection event
   */
  const hid_host_driver_config_t hid_host_driver_config = {
      .create_background_task = true,
      .task_priority = 5,
      .stack_size = 4096,
      .core_id = 0,
      .callback = hid_host_device_callback,
      .callback_arg = NULL};

  ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

  // Create queue
  app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));

  ESP_LOGI(TAG_HID_HOST, "Waiting for HID Device to be connected");

  while (1) {
    // Wait queue
    if (xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY)) {
      if (APP_EVENT == evt_queue.event_group) {
        // User pressed button
        usb_host_lib_info_t lib_info;
        ESP_ERROR_CHECK(usb_host_lib_info(&lib_info));
        if (lib_info.num_devices == 0) {
          // End while cycle
          break;
        } else {
          ESP_LOGW(TAG_HID_HOST, "To shutdown example, remove all USB devices "
                                 "and press button again.");
          // Keep polling
        }
      }

      if (APP_EVENT_HID_HOST == evt_queue.event_group) {
        hid_host_device_event(evt_queue.hid_host_device.handle,
                              evt_queue.hid_host_device.event,
                              evt_queue.hid_host_device.arg);
      }
    }
  }

  ESP_LOGI(TAG_HID_HOST, "HID Driver uninstall");
  ESP_ERROR_CHECK(hid_host_uninstall());
  gpio_isr_handler_remove(APP_QUIT_PIN);
  xQueueReset(app_event_queue);
  vQueueDelete(app_event_queue);
}

static esp_hid_raw_report_map_t ble_report_map[] = {
    {.data = mouseReportMap, .len = sizeof(mouseReportMap)},
};

static esp_hid_device_config_t ble_hid_config = {.vendor_id = 0x1234,
                                                 .product_id = 0x5678,
                                                 .version = 0x0001,
                                                 .device_name = "cvg-mouse",
                                                 .manufacturer_name =
                                                     "UT Southwestern",
                                                 .serial_number = "0000000001",
                                                 .report_maps = ble_report_map,
                                                 .report_maps_len = 1};

void ble_hid_task_start_up(void) {
  if (s_ble_hid_param.task_hdl) {
    // Task already exists
    return;
  }

  /* Nimble Specific */
  xTaskCreate(ble_hid_demo_task_mouse, "ble_hid_demo_task_mouse", 3 * 1024,
              NULL, configMAX_PRIORITIES - 3, &s_ble_hid_param.task_hdl);
}

void ble_hid_task_shut_down(void) {
  if (s_ble_hid_param.task_hdl) {
    vTaskDelete(s_ble_hid_param.task_hdl);
    s_ble_hid_param.task_hdl = NULL;
  }
}

static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base,
                                    int32_t id, void *event_data) {
  esp_hidd_event_t event = (esp_hidd_event_t)id;
  esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

  switch (event) {
  case ESP_HIDD_START_EVENT: {
    ESP_LOGI(TAG_BT_HID_DEVICE, "START");
    esp_hid_ble_gap_adv_start();
    break;
  }
  case ESP_HIDD_CONNECT_EVENT: {
    ESP_LOGI(TAG_BT_HID_DEVICE, "CONNECT");
    break;
  }
  case ESP_HIDD_PROTOCOL_MODE_EVENT: {
    ESP_LOGI(TAG_BT_HID_DEVICE, "PROTOCOL MODE[%u]: %s",
             param->protocol_mode.map_index,
             param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
    break;
  }
  case ESP_HIDD_CONTROL_EVENT: {
    ESP_LOGI(TAG_BT_HID_DEVICE, "CONTROL[%u]: %sSUSPEND",
             param->control.map_index, param->control.control ? "EXIT_" : "");
    if (param->control.control) {
      // exit suspend
      // FIXME: or don't start a task, set a variable
      ble_hid_task_start_up();
    } else {
      // suspend
      ble_hid_task_shut_down();
    }
    break;
  }
  case ESP_HIDD_OUTPUT_EVENT: {
    ESP_LOGI(TAG_BT_HID_DEVICE,
             "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index,
             esp_hid_usage_str(param->output.usage), param->output.report_id,
             param->output.length);
    ESP_LOG_BUFFER_HEX(TAG_BT_HID_DEVICE, param->output.data,
                       param->output.length);
    break;
  }
  case ESP_HIDD_FEATURE_EVENT: {
    ESP_LOGI(TAG_BT_HID_DEVICE, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:",
             param->feature.map_index, esp_hid_usage_str(param->feature.usage),
             param->feature.report_id, param->feature.length);
    ESP_LOG_BUFFER_HEX(TAG_BT_HID_DEVICE, param->feature.data,
                       param->feature.length);
    break;
  }
  case ESP_HIDD_DISCONNECT_EVENT: {
    ESP_LOGI(TAG_BT_HID_DEVICE, "DISCONNECT: %s",
             esp_hid_disconnect_reason_str(
                 esp_hidd_dev_transport_get(param->disconnect.dev),
                 param->disconnect.reason));
    ble_hid_task_shut_down();
    esp_hid_ble_gap_adv_start();
    break;
  }
  case ESP_HIDD_STOP_EVENT: {
    ESP_LOGI(TAG_BT_HID_DEVICE, "STOP");
    break;
  }
  default:
    break;
  }
  return;
}

void ble_hid_device_host_task(void *param) {
  ESP_LOGI(TAG_BT_HID_DEVICE, "BLE Host Task Started");
  /* This function will return only when nimble_port_stop() is executed */
  nimble_port_run();

  nimble_port_freertos_deinit();
}
void ble_store_config_init(void);

void app_main(void) {
  esp_err_t ret;

  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG_BT_HID_DEVICE, "setting hid gap, mode:%d", HID_DEV_MODE);
  ret = esp_hid_gap_init(HID_DEV_MODE);
  ESP_ERROR_CHECK(ret);

  ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE,
                                 ble_hid_config.device_name);
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG_BT_HID_DEVICE, "setting ble device");
  ESP_ERROR_CHECK(esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE,
                                    ble_hidd_event_callback,
                                    &s_ble_hid_param.hid_dev));

  ble_store_config_init();

  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  /* Starting nimble task after gatts is initialized*/
  ret = esp_nimble_enable(ble_hid_device_host_task);
  if (ret) {
    ESP_LOGE(TAG_BT_HID_DEVICE, "esp_nimble_enable failed: %d", ret);
  }
}
