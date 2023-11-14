/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "tusb.h"
#include "bsp/board_api.h"
#include "usb.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void cdc_task(void);

/*------------- MAIN -------------*/
void init_pico_usb_drive() {
    board_init();
    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);
    if (board_init_after_tusb) {
       board_init_after_tusb();
    }
}

void pico_usb_drive_heartbeat() {
    tud_task(); // tinyusb device task
    led_blinking_task();
    cdc_task();
}

void in_flash_drive() {
  init_pico_usb_drive();
  while(!tud_msc_test_ejected()) {
    pico_usb_drive_heartbeat();
  }  
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+
// Invoked when device is mounted
void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

// Invoked to determine max LUN
uint8_t tud_msc_get_maxlun_cb(void) {
  FRESULT result = f_mount(getSDCardFATFSptr(), "", 1);
  //char tmp[80]; sprintf(tmp, "tud_msc_get_maxlun_cb sd card mount: %s (%d)", FRESULT_str(result), result); logMsg(tmp);
  if (result == FR_OK) { logMsg((char*)"Two drives mode"); } else { logMsg((char*)"One drive mode"); }
  return result == FR_OK ? 2 : 1; // dual LUN
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void)
{
  // connected() check for DTR bit
  // Most but not all terminal client set this when making connection
  // if ( tud_cdc_connected() )
  {
    // connected and there are data available
    if ( tud_cdc_available() )
    {
      // read data
      char buf[64];
      uint32_t count = tud_cdc_read(buf, sizeof(buf));
      (void) count;

      // Echo back
      // Note: Skip echo by commenting out write() and write_flush()
      // for throughput test e.g
      //    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
      tud_cdc_write(buf, count);
      tud_cdc_write_flush();
    }
  }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;
  (void) rts;

  // TODO set some indicator
  if ( dtr )
  {
    // Terminal connected
  }else
  {
    // Terminal disconnected
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}


//#include "tusb_config.h"
static volatile bool _disk_busy[CFG_TUH_DEVICE_MAX];

bool msc_app_init(void) {
    for (size_t i = 0; i < CFG_TUH_DEVICE_MAX; ++i)
        _disk_busy[i] = false;
    // disable stdout buffered for echoing typing command
#ifndef __ICCARM__ // TODO IAR doesn't support stream control ?
    setbuf(stdout, NULL);
#endif
  //  cli_init();
    return true;
}
void msc_app_task(void) {
    // TODO:
}

void usb_host() {
    clrScr(1);
    board_init();
    logMsg("TinyUSB Host MassStorage Explorer");
    tuh_init(BOARD_TUH_RHPORT);
    while (1) {
        // tinyusb host task
        tuh_task();
        msc_app_task();
    }
}

static scsi_inquiry_resp_t inquiry_resp;

bool inquiry_complete_cb(uint8_t dev_addr, tuh_msc_complete_data_t const * cb_data) {
  msc_cbw_t const* cbw = cb_data->cbw;
  msc_csw_t const* csw = cb_data->csw;
  if (csw->status != 0) {
    logMsg("Inquiry failed");
    return false;
  }
  // Print out Vendor ID, Product ID and Rev
  char tmp[80];
  sprintf(tmp, "%.8s %.16s rev %.4s", inquiry_resp.vendor_id, inquiry_resp.product_id, inquiry_resp.product_rev); logMsg(tmp);

  // Get capacity of device
  uint32_t const block_count = tuh_msc_get_block_count(dev_addr, cbw->lun);
  uint32_t const block_size = tuh_msc_get_block_size(dev_addr, cbw->lun);

  sprintf(tmp, "Disk Size: %lu MB", block_count / ((1024*1024)/block_size)); logMsg(tmp);
  // printf("Block Count = %lu, Block Size: %lu\r\n", block_count, block_size);

  // For simplicity: we only mount 1 LUN per device
  uint8_t const drive_num = dev_addr-1;
  char drive_path[3] = "0:";
  drive_path[0] += drive_num;
/* TODO:
  if ( f_mount(&fatfs[drive_num], drive_path, 1) != FR_OK )
  {
    puts("mount failed");
  }
*/
  // change to newly mounted drive
  f_chdir(drive_path);

  // print the drive label
//  char label[34];
//  if ( FR_OK == f_getlabel(drive_path, label, NULL) )
//  {
//    puts(label);
//  }

  return true;
}

//------------- IMPLEMENTATION -------------//
void tuh_msc_mount_cb(uint8_t dev_addr) {
  logMsg("A MassStorage device is mounted");
  uint8_t const lun = 0;
  tuh_msc_inquiry(dev_addr, lun, &inquiry_resp, inquiry_complete_cb, 0);
}

void tuh_msc_umount_cb(uint8_t dev_addr) {
  logMsg("A MassStorage device is unmounted");
  uint8_t const drive_num = dev_addr-1;
  char drive_path[3] = "0:";
  drive_path[0] += drive_num;
  f_unmount(drive_path);

//  if ( phy_disk == f_get_current_drive() )
//  { // active drive is unplugged --> change to other drive
//    for(uint8_t i=0; i<CFG_TUH_DEVICE_MAX; i++)
//    {
//      if ( disk_is_ready(i) )
//      {
//        f_chdrive(i);
//        cli_init(); // refractor, rename
//      }
//    }
//  }
}
