/*
 Helium Mapper build for LilyGo TTGO T-Beam v1.1 boards.
 Copyright (C) 2021-2022 by Max-Plastix

 This is a development fork by Max-Plastix hosted here:
 https://github.com/Max-Plastix/tbeam-helium-mapper/

 This code comes from a number of developers and earlier efforts, visible in the
 full lineage on Github, including:  Fizzy, longfi-arduino, Kyle T. Gabriel, and Xose Pérez

 GPL makes this all possible -- continue to modify, extend, and share!
 */

/*
  Main module

  # Modified by Kyle T. Gabriel to fix issue with incorrect GPS data for
  TTNMapper

  Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <axp20x.h>
#include <lmic.h>

#include "configuration.h"
#include "gps.h"
#include "screen.h"
#include "sleep.h"
#include "ttn.h"

#ifndef UPLINK_SHORT
#define FPORT_MAPPER 2  // FPort for Uplink messages -- must match Helium Console Decoder script!
#else
#define FPORT_MAPPER 3  // FPort for Uplink messages -- must match Helium Console Decoder script!
#endif

#define FPORT_STATUS 5
#define FPORT_GPSLOST 6

#define STATUS_BOOT 1
#define STATUS_USB_ON 2
#define STATUS_USB_OFF 3

// Defined in ttn.ino
void ttn_register(void (*callback)(uint8_t message));

bool justSendNow = true;                // Send one at boot, regardless of deadzone?
unsigned long int last_send_ms = 0;     // Time of last uplink
unsigned long int last_moved_ms = 0;    // Time of last movement
unsigned long int last_gpslost_ms = 0;  // Time of last gps-lost packet
double last_send_lat = 0;               // Last known location
double last_send_lon = 0;               //
double dist_moved = 0;                  // Distance in m from last uplink

// Deadzone (no uplink) location and radius
double deadzone_lat = DEADZONE_LAT;
double deadzone_lon = DEADZONE_LON;
double deadzone_radius_m = DEADZONE_RADIUS_M;
boolean in_deadzone = false;

/* Defaults that can be overwritten by downlink messages */
/* (32-bit int seconds allows for 50 days) */
unsigned int stationary_tx_interval_s;  // prefs STATIONARY_TX_INTERVAL
unsigned int rest_wait_s;               // prefs REST_WAIT
unsigned int rest_tx_interval_s;        // prefs REST_TX_INTERVAL

unsigned int tx_interval_s;  // Currently-active time interval

enum activity_state {
  ACTIVITY_MOVING,
  ACTIVITY_REST,
  ACTIVITY_SLEEP,
  ACTIVITY_GPS_LOST,
  ACTIVITY_WOKE,
  ACTIVITY_INVALID
};
enum activity_state active_state = ACTIVITY_INVALID;
boolean never_rest = NEVER_REST;

// Return status from mapper uplink, since we care about the flavor of the failure
enum mapper_uplink_result { MAPPER_UPLINK_SUCCESS, MAPPER_UPLINK_BADFIX, MAPPER_UPLINK_NOLORA, MAPPER_UPLINK_NOTYET };

/* Maybe these moves to prefs eventually? */
unsigned int sleep_wait_s = SLEEP_WAIT;
unsigned int sleep_tx_interval_s = SLEEP_TX_INTERVAL;

unsigned int gps_lost_wait_s = GPS_LOST_WAIT;
unsigned int gps_lost_ping_s = GPS_LOST_PING;
uint32_t last_fix_time = 0;

float battery_low_voltage = BATTERY_LOW_VOLTAGE;
float min_dist_moved = MIN_DIST;

AXP20X_Class axp;
bool pmu_irq = false;  // true when PMU IRQ pending

bool oled_found = false;
bool axp192_found = false;
uint8_t oled_addr = 0;  // i2c address of OLED controller

bool packetQueued;
bool isJoined = false;

bool screen_stay_on = false;
bool screen_stay_off = false;
bool is_screen_on = true;
int screen_idle_off_s = SCREEN_IDLE_OFF_S;
uint32_t screen_last_active_ms = 0;
boolean in_menu = false;
boolean have_usb_power = true;
uint8_t usb_power_count = 0;

// Buffer for Payload frame
static uint8_t txBuffer[11];

// deep sleep support
RTC_DATA_ATTR int bootCount = 0;
esp_sleep_source_t wakeCause;  // the reason we booted this time

char buffer[40];  // Screen buffer

dr_t lorawan_sf;  // prefs LORAWAN_SF
char sf_name[40];

unsigned long int ack_req = 0;
unsigned long int ack_rx = 0;

// Store Lat & Long in six bytes of payload
void pack_lat_lon(double lat, double lon) {
  uint32_t LatitudeBinary;
  uint32_t LongitudeBinary;
  LatitudeBinary = ((lat + 90) / 180.0) * 16777215;
  LongitudeBinary = ((lon + 180) / 360.0) * 16777215;

  txBuffer[0] = (LatitudeBinary >> 16) & 0xFF;
  txBuffer[1] = (LatitudeBinary >> 8) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;
  txBuffer[3] = (LongitudeBinary >> 16) & 0xFF;
  txBuffer[4] = (LongitudeBinary >> 8) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;
}

uint8_t battery_byte(void) {
  uint16_t batteryVoltage = ((float_t)((float_t)(axp.getBattVoltage()) / 10.0) + .5);
  return (uint8_t)((batteryVoltage - 200) & 0xFF);
}

// Prepare a packet for the Mapper
void build_mapper_packet() {
  double lat;
  double lon;
  uint16_t altitudeGps;
  uint8_t sats;
  uint16_t speed;

  lat = tGPS.location.lat();
  lon = tGPS.location.lng();
  pack_lat_lon(lat, lon);
  altitudeGps = (uint16_t)tGPS.altitude.meters();
  speed = (uint16_t)tGPS.speed.kmph();  // convert from double
  if (speed > 255)
    speed = 255;  // don't wrap around.
  sats = tGPS.satellites.value();

  sprintf(buffer, "Lat: %f, ", lat);
  Serial.print(buffer);
  sprintf(buffer, "Long: %f, ", lon);
  Serial.print(buffer);
  sprintf(buffer, "Alt: %f, ", tGPS.altitude.meters());
  Serial.print(buffer);
  sprintf(buffer, "Sats: %d", sats);
  Serial.println(buffer);

  #ifndef UPLINK_SHORT
  txBuffer[6] = (altitudeGps >> 8) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  txBuffer[8] = speed & 0xFF;
  txBuffer[9] = battery_byte();

  txBuffer[10] = sats & 0xFF;
  #else
  uint8_t altitude_hundret = constrain((unsigned int)(round(altitudeGps/150)),0,31);
  uint8_t sattelites_offset = constrain(sats,3,10)-3;
  txBuffer[6] = (altitude_hundret << 3) | sattelites_offset;
  #endif
}

// Helium requires a FCount reset sometime before hitting 0xFFFF
// 50,000 makes it obvious it was intentional
#define MAX_FCOUNT 50000

boolean send_uplink(uint8_t *txBuffer, uint8_t length, uint8_t fport, boolean confirmed) {
  if (confirmed) {
    Serial.println("ACK requested");
    screen_print("? ");
    digitalWrite(RED_LED, LOW);  // Light LED
    ack_req++;
  }

  // send it!
  packetQueued = true;
  if (!ttn_send(txBuffer, length, fport, confirmed)) {
    Serial.println("Surprise send failure!");
    return false;
  }

  // Helium requires a re-join / reset of count to avoid 16bit count rollover
  // Hopefully a device reboot every 50k uplinks is no problem.
  if (ttn_get_count() > MAX_FCOUNT) {
    Serial.println("FCount Rollover!");

    // I don't understand why this doesn't show at all
    screen_print("\n\nRollover Reset!\n");
    screen_update();
    delay(1000);  // Give some time to read the screen

    ttn_erase_prefs();
    ESP.restart();
  }

  return true;
}

bool status_uplink(uint8_t status, uint8_t value) {
  if (!SEND_STATUS_UPLINKS)
    return false;

  pack_lat_lon(last_send_lat, last_send_lon);
  txBuffer[6] = battery_byte();
  txBuffer[7] = status;
  txBuffer[8] = value;
  Serial.printf("Tx: STATUS %d %d\n", status, value);
  screen_print("\nTX STATUS ");
  return send_uplink(txBuffer, 9, FPORT_STATUS, 0);
}

bool gpslost_uplink(void) {
  uint16_t minutes_lost;

  if (!SEND_GPSLOST_UPLINKS)
    return false;

  minutes_lost = (last_fix_time - millis()) / 1000 / 60;
  pack_lat_lon(last_send_lat, last_send_lon);
  txBuffer[6] = battery_byte();
  txBuffer[7] = tGPS.satellites.value();
  txBuffer[8] = (minutes_lost >> 8) & 0xFF;
  txBuffer[9] = minutes_lost & 0xFF;
  Serial.printf("Tx: GPSLOST %d\n", minutes_lost);
  screen_print("\nTX GPSLOST ");
  return send_uplink(txBuffer, 10, FPORT_GPSLOST, 0);
}

// Send a packet, if one is warranted
enum mapper_uplink_result mapper_uplink() {
  double now_lat = tGPS.location.lat();
  double now_lon = tGPS.location.lng();
  unsigned long int now = millis();

  // Here we try to filter out bogus GPS readings.
  if (!(tGPS.location.isValid() && tGPS.time.isValid() && tGPS.satellites.isValid() && tGPS.hdop.isValid() &&
        tGPS.altitude.isValid() && tGPS.speed.isValid()))
    return MAPPER_UPLINK_BADFIX;

  // Filter out any reports while we have low satellite count.  The receiver can old a fix on 3, but it's poor.
  if (tGPS.satellites.value() < 4)
    return MAPPER_UPLINK_BADFIX;

  // HDOP is only a hint as to accuracy, but we can assume very bad HDOP is not worth mapping.
  // https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation) suggests 5 is a good cutoff.
  if (tGPS.hdop.hdop() > 5.0)
    return MAPPER_UPLINK_BADFIX;

  // With the exception of a few places, a perfectly zero lat or long probably means we got a bad reading
  if (now_lat == 0.0 || now_lon == 0.0)
    return MAPPER_UPLINK_BADFIX;

  // Don't attempt to send or update until we join Helium
  if (!isJoined)
    return MAPPER_UPLINK_NOLORA;

  // LoRa is not ready for a new packet, maybe still sending the last one.
  if (!LMIC_queryTxReady())
    return MAPPER_UPLINK_NOLORA;

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
    return MAPPER_UPLINK_NOLORA;

  // distance from last transmitted location
  double dist_moved = tGPS.distanceBetween(last_send_lat, last_send_lon, now_lat, now_lon);
  double deadzone_dist = tGPS.distanceBetween(deadzone_lat, deadzone_lon, now_lat, now_lon);
  in_deadzone = (deadzone_dist <= deadzone_radius_m);

  /*
  Serial.printf("[Time %lu / %us, Moved %dm in %lus %c]\n", (now - last_send_ms) / 1000, tx_interval_s,
  (int32_t)dist_moved, (now - last_moved_ms) / 1000, in_deadzone ? 'D' : '-');
  */

  // Deadzone means we don't send unless asked
  if (in_deadzone && !justSendNow)
    return MAPPER_UPLINK_NOTYET;

  char because = '?';
  if (justSendNow) {
    justSendNow = false;
    Serial.println("** JUST_SEND_NOW");
    because = '>';
  } else if (dist_moved > min_dist_moved) {
    Serial.println("** DIST");
    last_moved_ms = now;
    because = 'D';
  } else if (now - last_send_ms > tx_interval_s * 1000) {
    Serial.println("** TIME");
    because = 'T';
  } else {
    return MAPPER_UPLINK_NOTYET;  // Nothing to do, go home early
  }

  // The first distance-moved is crazy, since has no origin.. don't put it on
  // screen.
  if (dist_moved > 1000000)
    dist_moved = 0;

  snprintf(buffer, sizeof(buffer), "\n%d %c %4lus %4.0fm ", ttn_get_count(), because, (now - last_send_ms) / 1000,
           dist_moved);
  screen_print(buffer);

  // prepare the LoRa frame
  build_mapper_packet();

  // Want an ACK on this one?
  bool confirmed = (LORAWAN_CONFIRMED_EVERY > 0) && (ttn_get_count() % LORAWAN_CONFIRMED_EVERY == 0);

  // Send it!
  #ifndef UPLINK_SHORT
  int transmitsize = 11;
  #else 
  int transmitsize = 7;
  #endif

  if (!send_uplink(txBuffer, transmitsize, FPORT_MAPPER, confirmed))
    return MAPPER_UPLINK_NOLORA;

  last_send_ms = now;
  last_send_lat = now_lat;
  last_send_lon = now_lon;

  screen_last_active_ms = now;
  return MAPPER_UPLINK_SUCCESS;  // We did it!
}

void mapper_restore_prefs(void) {
  Preferences p;
  if (p.begin("mapper", true))  // Read-only
  {
    min_dist_moved = p.getFloat("min_dist", MIN_DIST);
    rest_wait_s = p.getUInt("rest_wait", REST_WAIT);
    rest_tx_interval_s = p.getUInt("rest_tx", REST_TX_INTERVAL);
    stationary_tx_interval_s = p.getUInt("tx_interval", STATIONARY_TX_INTERVAL);
    if (sizeof(lorawan_sf) != sizeof(unsigned char))
      Serial.println("Error!  size mismatch for sf");
    lorawan_sf = p.getUChar("sf", LORAWAN_SF);
    // Close the Preferences
    p.end();
  } else {
    Serial.println("No Mapper prefs -- using defaults.");
    min_dist_moved = MIN_DIST;
    rest_wait_s = REST_WAIT;
    rest_tx_interval_s = REST_TX_INTERVAL;
    stationary_tx_interval_s = STATIONARY_TX_INTERVAL;
    lorawan_sf = LORAWAN_SF;
  }

  tx_interval_s = stationary_tx_interval_s;
}

void mapper_save_prefs(void) {
  Preferences p;

  Serial.println("Saving prefs.");
  if (p.begin("mapper", false)) {
    p.putFloat("min_dist", min_dist_moved);
    p.putUInt("rest_wait", rest_wait_s);
    p.putUInt("rest_tx", rest_tx_interval_s);
    p.putUInt("tx_interval", stationary_tx_interval_s);
    p.putUChar("sf", lorawan_sf);
    p.end();
  }
}

void mapper_erase_prefs(void) {
#if 0 
  nvs_flash_erase(); // erase the NVS partition and...
  nvs_flash_init(); // initialize the NVS partition.
#endif
  Preferences p;
  if (p.begin("mapper", false)) {
    p.clear();
    p.end();
  }
}

// LoRa message event callback
void lora_msg_callback(uint8_t message) {
  static boolean seen_joined = false, seen_joining = false;
#ifdef DEBUG_LORA_MESSAGES
  {
    snprintf(buffer, sizeof(buffer), "## MSG %d\n", message);
    screen_print(buffer);
  }
  if (EV_JOIN_TXCOMPLETE == message)
    Serial.println("# JOIN_TXCOMPLETE");
  if (EV_TXCOMPLETE == message)
    Serial.println("# TXCOMPLETE");
  if (EV_RXCOMPLETE == message)
    Serial.println("# RXCOMPLETE");
  if (EV_RXSTART == message)
    Serial.println("# RXSTART");
  if (EV_TXCANCELED == message)
    Serial.println("# TXCANCELED");
  if (EV_TXSTART == message)
    Serial.println("# TXSTART");
  if (EV_JOINING == message)
    Serial.println("# JOINING");
  if (EV_JOINED == message)
    Serial.println("# JOINED");
  if (EV_JOIN_FAILED == message)
    Serial.println("# JOIN_FAILED");
  if (EV_REJOIN_FAILED == message)
    Serial.println("# REJOIN_FAILED");
  if (EV_RESET == message)
    Serial.println("# RESET");
  if (EV_LINK_DEAD == message)
    Serial.println("# LINK_DEAD");
  if (EV_ACK == message)
    Serial.println("# ACK");
  if (EV_PENDING == message)
    Serial.println("# PENDING");
  if (EV_QUEUED == message)
    Serial.println("# QUEUED");
#endif

  /* This is confusing because JOINED is sometimes spoofed and comes early */
  if (EV_JOINED == message)
    seen_joined = true;
  if (EV_JOINING == message)
    seen_joining = true;
  if (!isJoined && seen_joined && seen_joining) {
    isJoined = true;
    screen_print("Joined Helium!\n");
    ttn_set_sf(lorawan_sf);  // Joining seems to leave it at SF10?
    ttn_get_sf_name(sf_name, sizeof(sf_name));
  }

  if (EV_TXSTART == message) {
    screen_print("+\v");
    screen_update();
  }
  // We only want to say 'packetSent' for our packets (not packets needed for
  // joining)
  if (EV_TXCOMPLETE == message && packetQueued) {
    //    screen_print("sent.\n");
    packetQueued = false;
    if (axp192_found)
      axp.setChgLEDMode(AXP20X_LED_OFF);
  }

  if (EV_ACK == message) {
    digitalWrite(RED_LED, HIGH);
    ack_rx++;
    Serial.printf("ACK! %lu / %lu\n", ack_rx, ack_req);
    screen_print("! ");
  }

  if (EV_RXCOMPLETE == message || EV_RESPONSE == message) {
    size_t len = ttn_response_len();
    uint8_t data[len];
    uint8_t port;
    ttn_response(&port, data, len);

    snprintf(buffer, sizeof(buffer), "\nRx: %d on P%d\n", len, port);
    screen_print(buffer);

    Serial.printf("Downlink on port: %d = ", port);
    for (int i = 0; i < len; i++) {
      if (data[i] < 16)
        Serial.print('0');
      Serial.print(data[i], HEX);
    }
    Serial.println();

    /*
     * Downlink format: FPort 1
     * 2 Bytes: Minimum Distance (1 to 65535) meters, or 0 no-change
     * 2 Bytes: Minimum Time (1 to 65535) seconds (18.2 hours) between pings, or
     * 0 no-change, or 0xFFFF to use default 1 Byte:  Battery voltage (2.0
     * to 4.5) for auto-shutoff, or 0 no-change
     */
    if (port == 1 && len == 5) {
      float new_distance = (float)(data[0] << 8 | data[1]);
      if (new_distance > 0.0) {
        min_dist_moved = new_distance;
        snprintf(buffer, sizeof(buffer), "\nNew Dist: %.0fm\n", new_distance);
        screen_print(buffer);
      }

      unsigned long int new_interval = data[2] << 8 | data[3];
      if (new_interval) {
        if (new_interval == 0xFFFF) {
          stationary_tx_interval_s = STATIONARY_TX_INTERVAL;
        } else {
          stationary_tx_interval_s = new_interval;
        }
        tx_interval_s = stationary_tx_interval_s;
        snprintf(buffer, sizeof(buffer), "\nNew Time: %.0lus\n", new_interval);
        screen_print(buffer);
      }

      if (data[4]) {
        float new_low_voltage = data[4] / 100.00 + 2.00;
        battery_low_voltage = new_low_voltage;
        snprintf(buffer, sizeof(buffer), "\nNew LowBat: %.2fv\n", new_low_voltage);
        screen_print(buffer);
      }
    }
  }
}

void scanI2Cdevice(void) {
  byte err, addr;
  int nDevices = 0;
  for (addr = 1; addr < 0x7F; addr++) {
    Wire.beginTransmission(addr);
    err = Wire.endTransmission();
    if (err == 0) {
#if 0
      Serial.print("I2C device found at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.print(addr, HEX);
      Serial.println(" !");
#endif
      nDevices++;

      if (addr == 0x3C || addr == 0x78 || addr == 0x7E) {
        oled_addr = addr;
        oled_found = true;
        Serial.printf("OLED at %02X\n", oled_addr);
      }
      if (addr == AXP192_SLAVE_ADDRESS) {
        axp192_found = true;
        Serial.println("AXP192 PMU");
      }
    } else if (err == 4) {
      Serial.print("Unknown i2c device at 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found!\n");
  /* else  Serial.println("done\n"); */
}

/* The AXP library computes this incorrectly for AXP192.
   It's just a fixed mapping table from the datasheet */
int axp_charge_to_ma(int set) {
  switch (set) {
    case 0:
      return 100;
    case 1:
      return 190;
    case 2:
      return 280;
    case 3:
      return 360;
    case 4:
      return 450;
    case 5:
      return 550;
    case 6:
      return 630;
    case 7:
      return 700;
    case 8:
      return 780;
    case 9:
      return 880;
    case 10:
      return 960;
    case 11:
      return 1000;
    case 12:
      return 1080;
    case 13:
      return 1160;
    case 14:
      return 1240;
    case 15:
      return 1320;
    default:
      return -1;
  }
}

/**
  Initialize the AXP192 power manager chip.

  DCDC1 0.7-3.5V @ 1200mA max -> OLED
  If you turn the OLED off, it will drag down the I2C lines and block the bus from the AXP192 which shares it.
  Use SSD1306 sleep mode instead

  DCDC3 0.7-3.5V @ 700mA max -> ESP32 (keep this on!)
  LDO1 30mA -> "VCC_RTC" charges GPS tiny J13 backup battery
  LDO2 200mA -> "LORA_VCC"
  LDO3 200mA -> "GPS_VCC"
*/
void axp192Init() {
  if (axp192_found) {
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
      // Serial.println("AXP192 Begin PASS");
    } else {
      Serial.println("axp.begin() FAIL");
      axp192_found = false;
      return;
    }

    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);        // LORA radio 200mA "LORA_VCC"
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);        // GPS power 200mA "GPS_VCC"
    axp.setLDO3Voltage(3300);                          // Voltage for GPS Power.  (Neo-6 can take 2.7v to 3.6v)
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);       // OLED power, 1200mA max "VCC_2.5V"
    axp.setDCDC1Voltage(2500);                         // Voltage or the OLED SSD1306
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);      // Unconnected
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);      // "EXTEN" pin, unused
    axp.setChargeControlCur(AXP1XX_CHARGE_CUR_550MA);  // Default 0x1000 = 780mA, more than we can get from USB

    // Flash the Blue LED until our first packet is transmitted
    axp.setChgLEDMode(AXP20X_LED_BLINK_4HZ);
    // axp.setChgLEDMode(AXP20X_LED_OFF);

#if 0
    Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
    //Serial.printf("LDO1: %s\n", axp.isLDO1Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");
#endif
    // Fire an interrupt on falling edge.  Note that some IRQs repeat/persist.
    pinMode(PMU_IRQ, INPUT);
    gpio_pullup_en((gpio_num_t)PMU_IRQ);
    attachInterrupt(
        PMU_IRQ, [] { pmu_irq = true; }, FALLING);

    // Configure REG 36H: PEK press key parameter set.  Index values for
    // argument!
    axp.setStartupTime(2);        // "Power on time": 512mS
    axp.setlongPressTime(2);      // "Long time key press time": 2S
    axp.setShutdownTime(2);       // "Power off time" = 8S
    axp.setTimeOutShutdown(1);    // "When key press time is longer than power off time, auto power off"
    axp.setVWarningLevel1(2950);  // These warning IRQs do not clear until charged, and inhibit other IRQs!
    axp.setVWarningLevel2(2900);  // We effectively disable them by setting them lower than we'd run

    // Serial.printf("AC IN: %fv\n", axp.getAcinVoltage());
    // Serial.printf("Vbus: %fv\n", axp.getVbusVoltage());
    Serial.printf("PMIC Temp %0.2f°C\n", axp.getTemp());
    // Serial.printf("TSTemp %f°C\n", axp.getTSTemp());
    // Serial.printf("GPIO0 %fv\n", axp.getGPIO0Voltage());
    // Serial.printf("GPIO1 %fv\n", axp.getGPIO1Voltage());
    // Serial.printf("Batt In: %fmW\n", axp.getBattInpower());
    Serial.printf("Battery: %0.3fv\n", axp.getBattVoltage() / 1000.0);
    Serial.printf("SysIPSOut: %0.3fv\n", axp.getSysIPSOUTVoltage() / 1000.0);
    Serial.printf("isVBUSPlug? %s\n", axp.isVBUSPlug() ? "Yes" : "No");
    have_usb_power = axp.isVBUSPlug();
    Serial.printf("isChargingEnable? %s\n", axp.isChargeingEnable() ? "Yes" : "No");
    // Doesn't work on AXP192 because it has a different charge current curve:
    // Serial.printf("ChargeCurrent: %.2fmA\n", axp.getSettingChargeCurrent());
    Serial.printf("ChargeControlCurrent: %d = %dmA\n", axp.getChargeControlCur(),
                  axp_charge_to_ma(axp.getChargeControlCur()));
    Serial.printf("Battery Charge Level: %d%%\n", axp.getBattPercentage());

    Serial.printf("WarningLevel1: %d mV\n", axp.getVWarningLevel1());
    Serial.printf("WarningLevel2: %d mV\n", axp.getVWarningLevel2());
    Serial.printf("PowerDown:     %d mV\n", axp.getPowerDownVoltage());

    Serial.printf("DCDC1Voltage: %d mV\n", axp.getDCDC1Voltage());
    Serial.printf("DCDC2Voltage: %d mV\n", axp.getDCDC2Voltage());
    Serial.printf("DCDC3Voltage: %d mV\n", axp.getDCDC3Voltage());
    Serial.printf("LDO2:         %d mV\n", axp.getLDO2Voltage());
    Serial.printf("LDO3:         %d mV\n", axp.getLDO3Voltage());
    Serial.printf("LDO4:         %d mV\n", axp.getLDO4Voltage());

    // Enable battery current measurements
    axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
    //    axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ |
    //    AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
    axp.enableIRQ(0xFFFFFFFFFF, 1);  // Give me ALL the interrupts you have.

    // @Kenny_PDY discovered that low-battery voltage inhibits detecting the menu button.
    // Disable these two IRQs until we figure out why it blocks the PEK button IRQs.
    // Low battery also seems to inhibit the USB present/lost signal we use to wake up.
    axp.enableIRQ(APX202_APS_LOW_VOL_LEVEL1_IRQ, 0);
    axp.enableIRQ(AXP202_APS_LOW_VOL_LEVEL2_IRQ, 0);

    // The Charging Current available is less than requested for battery charging.
    // Another Persistent IRQ.  Clear it after showing it once?
    // TODO: Show it every X minutes?  Adjust charge current request?
    axp.enableIRQ(AXP202_CHARGE_LOW_CUR_IRQ, 0);

    axp.clearIRQ();
  } else {
    Serial.println("AXP192 not found!");
  }
}

// Perform power on init that we do on each wake from deep sleep
void wakeup() {
  bootCount++;
  wakeCause = esp_sleep_get_wakeup_cause();

  Serial.printf("BOOT #%d!  cause:%d ext1:%08llx\n", bootCount, wakeCause, esp_sleep_get_ext1_wakeup_status());
}

#include <BluetoothSerial.h>
#include <WiFi.h>
#include <esp_bt.h>

void setup() {
  // Debug
#ifdef DEBUG_PORT
  DEBUG_PORT.begin(SERIAL_BAUD);
#endif
  wakeup();

  // Make sure WiFi and BT are off
  // WiFi.disconnect(true);
  WiFi.mode(WIFI_MODE_NULL);
  btStop();

  Wire.begin(I2C_SDA, I2C_SCL);
  scanI2Cdevice();

  axp192Init();

  // GPS sometimes gets wedged with no satellites in view and only a power-cycle
  // saves it. Here we turn off power and the delay in screen setup is enough
  // time to bonk the GPS
  axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // GPS power off

  // Buttons & LED
  pinMode(MIDDLE_BUTTON_PIN, INPUT);
  gpio_pullup_en((gpio_num_t)MIDDLE_BUTTON_PIN);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);  // Off

  // Hello
  DEBUG_MSG("\n" APP_NAME " " APP_VERSION "\n");

  mapper_restore_prefs();  // Fetch saved settings

  // Don't init display if we don't have one or we are waking headless due to a
  // timer event
  if (0 && wakeCause == ESP_SLEEP_WAKEUP_TIMER)
    oled_found = false;  // forget we even have the hardware

  // This creates the display object, so if we don't call it.. all screen ops are do-nothing.
  if (oled_found)
    screen_setup(oled_addr);
  is_screen_on = true;

  // GPS power on, so it has time to settle.
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);

  // Show logo on first boot (as opposed to wake)
  if (bootCount <= 1) {
    screen_print(APP_NAME " " APP_VERSION, 0, 0);  // Above the Logo
    screen_print(APP_NAME " " APP_VERSION "\n");   // Add it to the log too

    screen_show_logo();
    screen_update();
    delay(LOGO_DELAY);
  }

  // Helium setup
  if (!ttn_setup()) {
    screen_print("[ERR] Radio module not found!\n");
    sleep_forever();
  }

  ttn_register(lora_msg_callback);
  ttn_join();
  ttn_adr(LORAWAN_ADR);

  // Might have to add a longer delay here for GPS boot-up.  Takes longer to sync if we talk to it too early.
  delay(100);
  gps_setup(true);  // Init GPS baudrate and messages

  // This is bad.. we can't find the AXP192 PMIC, so no menu key detect:
  if (!axp192_found)
    screen_print("** Missing AXP192! **\n");

  Serial.printf("Deadzone: %f.0m @ %f, %f\n", deadzone_radius_m, deadzone_lat, deadzone_lon);
}

// Should be around 0.5mA ESP32 consumption, plus OLED controller and PMIC overhead.
void low_power_sleep(uint32_t seconds) {
  boolean was_screen_on = is_screen_on;

  Serial.printf("Sleep %d..\n", seconds);
  Serial.flush();

  screen_off();

  digitalWrite(RED_LED, HIGH);  // LED Off

  if (axp192_found) {
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // GPS power
    axp.setChgLEDMode(AXP20X_LED_OFF);            // Blue LED off

    // Turning off DCDC1 consumes MORE power, for reasons unknown
    // axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);  // OLED power off
    // pinMode(I2C_SCL, OUTPUT);
    // digitalWrite(I2C_SCL, HIGH);                    // Must enable pull-up on SCL to keep AXP accessible
  }

  // Wake on either button press
  gpio_wakeup_enable((gpio_num_t)MIDDLE_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)PMU_IRQ, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  esp_sleep_enable_timer_wakeup(seconds * 1000ULL * 1000ULL);  // call expects usecs
  // Some GPIOs need this to stay on?
  // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  esp_light_sleep_start();
  // If we woke by keypress (7) then turn on the screen
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
    // Try not to puke, but we pretend we moved if they hit a key, to exit SLEEP and restart timers
    last_moved_ms = screen_last_active_ms = millis();
    was_screen_on = true;  // Lies
    Serial.println("(GPIO)");
  }
  Serial.println("..woke");

  if (was_screen_on)
    screen_on();

  if (axp192_found) {
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // GPS power
    // axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);  // OLED power
    // if (oled_found)
    //  screen_setup();
  }

  delay(100);        // GPS doesn't respond right away.. not ready for baud-rate test.
  gps_setup(false);  // Resync with GPS
}

// Power OFF -- does not return
void clean_shutdown(void) {
  LMIC_shutdown();  // cleanly shutdown the radio
  mapper_save_prefs();
  ttn_write_prefs();
  if (axp192_found) {
    axp.setChgLEDMode(AXP20X_LED_OFF);  // Surprisingly sticky if you don't set it

    axp.shutdown();  // PMIC power off
  } else {
    while (1)
      ;  // ?? What to do here.  burn power?
  }
}

uint32_t woke_time_ms = 0;
uint32_t woke_fix_count = 0;

/* Determine the current activity state */
void update_activity() {
  static enum activity_state last_active_state = ACTIVITY_INVALID;

  if (active_state != last_active_state) {
    switch (active_state) {
      case ACTIVITY_MOVING:
        Serial.println("//MOVING//");
        break;
      case ACTIVITY_REST:
        Serial.println("//REST//");
        break;
      case ACTIVITY_SLEEP:
        Serial.println("//SLEEP//");
        break;
      case ACTIVITY_GPS_LOST:
        Serial.println("//GPS_LOST//");
        break;
      case ACTIVITY_WOKE:
        Serial.println("//WOKE//");
        break;
      default:
        Serial.println("//WTF?//");
        break;
    }
    last_active_state = active_state;
  }

  uint32_t now = millis();
  float bat_volts = axp.getBattVoltage() / 1000;
  float charge_ma = axp.getBattChargeCurrent();

  if (axp192_found && axp.isBatteryConnect() && bat_volts < battery_low_voltage && charge_ma < 99.0) {
    Serial.println("Low Battery OFF");
    screen_print("\nLow Battery OFF\n");
    delay(4999);  // Give some time to read the screen
    clean_shutdown();
  }

  // Here we just woke from a GPS-off long sleep.
  // When we have a fresh GPS fix, and the fix qualifies for mapper report, we can resume
  // either mapping or going back to sleep.  Until then, we loop in Wake looking for a good GPS signal.
  // Note that we have to be sensitive to "good fix, but not interesting" and go right back to sleep.
  // We're only staying awake until we got a good GPS fix or gave up, NOT until we send a mapper report.
  if (active_state == ACTIVITY_WOKE) {
    if (tGPS.sentencesWithFix() != woke_fix_count && mapper_uplink() != MAPPER_UPLINK_BADFIX)
      active_state = ACTIVITY_REST;
    else if (now - woke_time_ms > gps_lost_wait_s * 1000)
      active_state = ACTIVITY_GPS_LOST;
    return;  // else stay in WOKE until we make a good report
  }

  if (active_state == ACTIVITY_SLEEP && !in_menu) {
    low_power_sleep(tx_interval_s);
    active_state = ACTIVITY_WOKE;
    woke_time_ms = millis();
    woke_fix_count = tGPS.sentencesWithFix();
    return;
  }

  // In order of precedence:
  if (never_rest) {
    active_state = ACTIVITY_MOVING;
  } else if (now - last_moved_ms > sleep_wait_s * 1000) {
    active_state = ACTIVITY_SLEEP;
  } else if (now - last_fix_time > gps_lost_wait_s * 1000) {
    active_state = ACTIVITY_GPS_LOST;
  } else if (now - last_moved_ms > rest_wait_s * 1000) {
    active_state = ACTIVITY_REST;
  } else {
    active_state = ACTIVITY_MOVING;
  }

  {
    boolean had_usb_power = have_usb_power;
    have_usb_power = (axp192_found && axp.isVBUSPlug());

    if (have_usb_power && !had_usb_power) {
      usb_power_count++;
      status_uplink(STATUS_USB_ON, usb_power_count);
    }
    if (!have_usb_power && had_usb_power) {
      status_uplink(STATUS_USB_OFF, usb_power_count);
    }
  }

  // If we have USB power, keep GPS on all the time; don't sleep
  if (have_usb_power) {
    if (active_state == ACTIVITY_SLEEP)
      active_state = ACTIVITY_REST;
  }

  switch (active_state) {
    case ACTIVITY_MOVING:
      tx_interval_s = stationary_tx_interval_s;
      break;
    case ACTIVITY_REST:
      tx_interval_s = rest_tx_interval_s;
      break;
    case ACTIVITY_GPS_LOST:
      tx_interval_s = gps_lost_ping_s;
      break;
    case ACTIVITY_SLEEP:
      tx_interval_s = sleep_tx_interval_s;
      break;
    default:
      // ???
      tx_interval_s = stationary_tx_interval_s;
      break;
  }

  // Has the screen been on for longer than idle time?
  if (now - screen_last_active_ms > screen_idle_off_s * 1000) {
    if (is_screen_on && !screen_stay_on) {
      is_screen_on = false;
      screen_off();
    }
  } else {  // Else we had some recent activity.  Turn on?
    if (!is_screen_on && !screen_stay_off) {
      is_screen_on = true;
      screen_on();
    }
  }
}

/* I must know what that interrupt was for! */
const char *find_irq_name(void) {
  const char *irq_name = "MysteryIRQ";

  if (axp.isAcinOverVoltageIRQ())
    irq_name = "AcinOverVoltage";
  else if (axp.isAcinPlugInIRQ())
    irq_name = "AcinPlugIn";
  else if (axp.isAcinRemoveIRQ())
    irq_name = "AcinRemove";
  else if (axp.isVbusOverVoltageIRQ())
    irq_name = "VbusOverVoltage";
  else if (axp.isVbusPlugInIRQ())
    irq_name = "USB Connected";  // "VbusPlugIn";
  else if (axp.isVbusRemoveIRQ())
    irq_name = "USB Removed";  // "VbusRemove";
  else if (axp.isVbusLowVHOLDIRQ())
    irq_name = "VbusLowVHOLD";
  else if (axp.isBattPlugInIRQ())
    irq_name = "BattPlugIn";
  else if (axp.isBattRemoveIRQ())
    irq_name = "BattRemove";
  else if (axp.isBattEnterActivateIRQ())
    irq_name = "BattEnterActivate";
  else if (axp.isBattExitActivateIRQ())
    irq_name = "BattExitActivate";
  else if (axp.isChargingIRQ())
    irq_name = "Charging";
  else if (axp.isChargingDoneIRQ())
    irq_name = "ChargingDone";
  else if (axp.isBattTempLowIRQ())
    irq_name = "BattTempLow";
  else if (axp.isBattTempHighIRQ())
    irq_name = "BattTempHigh";
  else if (axp.isChipOvertemperatureIRQ())
    irq_name = "ChipOvertemperature";
  else if (axp.isChargingCurrentLessIRQ()) {
    irq_name = "ChargingCurrentLess";
  } else if (axp.isDC2VoltageLessIRQ())
    irq_name = "DC2VoltageLess";
  else if (axp.isDC3VoltageLessIRQ())
    irq_name = "DC3VoltageLess";
  else if (axp.isLDO3VoltageLessIRQ())
    irq_name = "LDO3VoltageLess";
  else if (axp.isPEKShortPressIRQ())
    irq_name = "PEKShortPress";
  else if (axp.isPEKLongtPressIRQ())
    irq_name = "PEKLongtPress";
  else if (axp.isNOEPowerOnIRQ())
    irq_name = "NOEPowerOn";
  else if (axp.isNOEPowerDownIRQ())
    irq_name = "NOEPowerDown";
  else if (axp.isVBUSEffectiveIRQ())
    irq_name = "VBUSEffective";
  else if (axp.isVBUSInvalidIRQ())
    irq_name = "VBUSInvalid";
  else if (axp.isVUBSSessionIRQ())
    irq_name = "VUBSSession";
  else if (axp.isVUBSSessionEndIRQ())
    irq_name = "VUBSSessionEnd";
  else if (axp.isLowVoltageLevel1IRQ())
    irq_name = "LowVoltageLevel1";
  else if (axp.isLowVoltageLevel2IRQ())
    irq_name = "LowVoltageLevel2";
  else if (axp.isTimerTimeoutIRQ())
    irq_name = "TimerTimeout";
  else if (axp.isPEKRisingEdgeIRQ())
    irq_name = "PEKRisingEdge";
  else if (axp.isPEKFallingEdgeIRQ())
    irq_name = "PEKFallingEdge";
  else if (axp.isGPIO3InputEdgeTriggerIRQ())
    irq_name = "GPIO3InputEdgeTrigger";
  else if (axp.isGPIO2InputEdgeTriggerIRQ())
    irq_name = "GPIO2InputEdgeTrigger";
  else if (axp.isGPIO1InputEdgeTriggerIRQ())
    irq_name = "GPIO1InputEdgeTrigger";
  else if (axp.isGPIO0InputEdgeTriggerIRQ())
    irq_name = "GPIO0InputEdgeTrigger";

  return irq_name;
}

struct menu_entry {
  const char *name;
  void (*func)(void);
};

void menu_send_now(void) {
  justSendNow = true;
}
void menu_power_off(void) {
  screen_print("\nPOWER OFF...\n");
  delay(4000);  // Give some time to read the screen
  clean_shutdown();
}
void menu_flush_prefs(void) {
  screen_print("\nFlushing Prefs!\n");
  ttn_erase_prefs();
  mapper_erase_prefs();
  delay(5000);  // Give some time to read the screen
  ESP.restart();
}
void menu_distance_plus(void) {
  min_dist_moved += 5;
}
void menu_distance_minus(void) {
  min_dist_moved -= 5;
  if (min_dist_moved < 10)
    min_dist_moved = 10;
}
void menu_time_plus(void) {
  stationary_tx_interval_s += 10;
}
void menu_time_minus(void) {
  stationary_tx_interval_s -= 10;
  if (stationary_tx_interval_s < 10)
    stationary_tx_interval_s = 10;
}
void menu_gps_passthrough(void) {
  axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
  axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);  // Kill LORA radio
  gps_passthrough();
  // Does not return.
}

void menu_experiment(void) {
  static int gps_mv = 3300;

  gps_mv += 100;
  if (gps_mv > 3600)
    gps_mv = 2700;

  Serial.printf("GPS Voltage: %d\n", gps_mv);
  snprintf(buffer, sizeof(buffer), "\nGPS %dmv", gps_mv);
  screen_print(buffer);

  axp.setLDO3Voltage(gps_mv);                          // Voltage for GPS Power.  (Neo-6 can take 2.7v to 3.6v) 
}

void menu_deadzone_here(void) {
  if (tGPS.location.isValid()) {
    deadzone_lat = tGPS.location.lat();
    deadzone_lon = tGPS.location.lng();
    deadzone_radius_m = DEADZONE_RADIUS_M;
  }
}
void menu_no_deadzone(void) {
  deadzone_radius_m = 0.0;
}

void menu_stay_on(void) {
  screen_stay_on = !screen_stay_on;
}

void menu_gps_reset(void) {
  gps_full_reset();
}

dr_t sf_list[] = {DR_SF7, DR_SF8, DR_SF9, DR_SF10};
#define SF_ENTRIES (sizeof(sf_list) / sizeof(sf_list[0]))
uint8_t sf_index = 0;

void menu_change_sf(void) {
  sf_index++;
  if (sf_index >= SF_ENTRIES)
    sf_index = 0;

  lorawan_sf = sf_list[sf_index];
  ttn_set_sf(lorawan_sf);
  ttn_get_sf_name(sf_name, sizeof(sf_name));
  Serial.printf("New SF: %s\n", sf_name);
}

struct menu_entry menu[] = {
    {"Send Now", menu_send_now},           {"Power Off", menu_power_off},     {"Distance +", menu_distance_plus},
    {"Distance -", menu_distance_minus},   {"Time +", menu_time_plus},        {"Time -", menu_time_minus},
    {"Change SF", menu_change_sf},         {"Full Reset", menu_flush_prefs},  {"USB GPS", menu_gps_passthrough},
    {"Deadzone Here", menu_deadzone_here}, {"No Deadzone", menu_no_deadzone}, {"Stay On", menu_stay_on},
    {"GPS Reset", menu_gps_reset},         {"Experiment", menu_experiment}};
#define MENU_ENTRIES (sizeof(menu) / sizeof(menu[0]))

const char *menu_prev;
const char *menu_cur;
const char *menu_next;
boolean is_highlighted = false;
int menu_entry = 0;
static uint32_t menu_idle_start = 0;  // what tick should we call this press long enough

void menu_press(void) {
  if (in_menu)
    menu_entry = (menu_entry + 1) % MENU_ENTRIES;
  else
    in_menu = true;

  menu_prev = menu[(menu_entry - 1) % MENU_ENTRIES].name;
  menu_cur = menu[menu_entry].name;
  menu_next = menu[(menu_entry + 1) % MENU_ENTRIES].name;

  menu_idle_start = millis();
}

void menu_selected(void) {
  menu_idle_start = millis();
  menu[menu_entry].func();
}

void update_screen(void) {
  screen_header(tx_interval_s, min_dist_moved, sf_name, in_deadzone, screen_stay_on, never_rest);
  screen_body(in_menu, menu_prev, menu_cur, menu_next, is_highlighted);
}

void loop() {
  static uint32_t last_fix_count = 0;
  static boolean booted = true;
  uint32_t now_fix_count;
  uint32_t now = millis();

  gps_loop(0 /* active_state == ACTIVITY_WOKE */);  // Update GPS
  now_fix_count = tGPS.sentencesWithFix();          // Did we get a new fix?
  if (now_fix_count != last_fix_count) {
    last_fix_count = now_fix_count;
    last_fix_time = now;  // Note the time of most recent fix
  }

  ttn_loop();

  // menu timeout
  if (in_menu && now - menu_idle_start > (MENU_TIMEOUT_S)*1000)
    in_menu = false;

  update_screen();

  // If any interrupts on PMIC, report the name
  // PEK button handler
  if (axp192_found && pmu_irq) {
    const char *irq_name;
    pmu_irq = false;
    axp.readIRQ();
    irq_name = find_irq_name();

    if (axp.isPEKShortPressIRQ())
      menu_press();
    else if (axp.isPEKLongtPressIRQ())  // want to turn OFF
      menu_power_off();
    else {
      snprintf(buffer, sizeof(buffer), "\n* %s  ", irq_name);
      screen_print(buffer);
    }
    axp.clearIRQ();
    screen_last_active_ms = now;
  }

  // Middle Button handler
  static uint32_t pressTime = 0;
  if (!digitalRead(MIDDLE_BUTTON_PIN)) {
    // Pressure is on
    if (!pressTime) {  // just started a new press
      pressTime = now;
      screen_last_active_ms = now;
      is_highlighted = true;
    }
  } else if (pressTime) {
    // we just did a release
    if (in_menu)
      menu_selected();
    else {
      screen_print("\nSend! ");
      justSendNow = true;
    }
    is_highlighted = false;

    if (now - pressTime > 1000) {
      // Was a long press
    } else {
      // Was a short press
    }
    pressTime = 0;  // Released
    screen_last_active_ms = now;
  }

  update_activity();

  if (booted) {
    // status_uplink(STATUS_BOOT, 0);
    booted = 0;
  }

  if (active_state == ACTIVITY_GPS_LOST) {
    now = millis();
    if ((last_gpslost_ms == 0) ||  // first time losing GPS?
        (now - last_gpslost_ms > GPS_LOST_PING * 1000)) {
      gpslost_uplink();
      last_gpslost_ms = now;
    }
  } else {
    if (active_state != ACTIVITY_SLEEP)  // not sure about this
      last_gpslost_ms = 0;               // Reset if we regained GPS
  }

  if (mapper_uplink() == MAPPER_UPLINK_SUCCESS) {
    // Good send, light Blue LED
    if (axp192_found)
      axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
  } else {
    // Nothing sent.
    // Do NOT delay() here.. the LoRa receiver and join housekeeping also needs to run!
  }
}
