/*
 Helium Mapper build for LilyGo TTGO T-Beam v0.7, v1.0, and v1.1 boards.
 Copyright (C) 2021 by Max-Plastix

 This is a development fork by Max-Plastix hosted here:
 https://github.com/Max-Plastix/tbeam-helium-mapper/

 This code comes from a number of developers and earlier efforts, visible in the lineage on Github and prior comments below.
 GPL makes this all possible -- continue to modify, extend, and share!
 */

/*
  This module and those attached with it have been modified for the Helium Network by Fizzy. The following has been changed from the original modifications for
  Helium, by longfi-arduino:
  - Added Helium Startup Logo
  - Changed App Name and Version of device to reflect more of a device name and number scheme.
  - Enabled long press middle button to Discard Prefs by default for future troubleshooting on device.
  - Changed Text output to reflect Helium, and not TTL (Code referances ttn, just to prevent brakes in this awesome code)
  - Changed credentials file to use OTAA by default.
  - Changed GPS metric output text "Error", to "Accuracy/HDOP".
*/
/*
  Main module

  # Modified by Kyle T. Gabriel to fix issue with incorrect GPS data for TTNMapper

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
#include <Wire.h>
#include <axp20x.h>
#include <lmic.h>
#include <Preferences.h>

#include "configuration.h"
#include "gps.h"
#include "screen.h"
#include "sleep.h"
#include "ttn.h"
#include "blacklistlocations.h"

// Defined in ttn.ino
void ttn_register(void (*callback)(uint8_t message));

bool justSendNow = true;  // Start by sending
unsigned long int last_send_millis = 0;
unsigned long int last_moved_millis = 0;
float last_send_lat = 0;
float last_send_lon = 0;
float dist_moved = 0;

/* Defaults that can be overwritten by downlink messages */
/* 32-bit int seconds is 50 days maximum */
unsigned int rest_wait_s; // prefs REST_WAIT
unsigned int rest_tx_interval_s; // prefs REST_TX_INTERVAL
unsigned int stationary_tx_interval_s; // prefs STATIONARY_TX_INTERVAL
unsigned int tx_interval_s;

float battery_low_voltage = BATTERY_LOW_VOLTAGE;
float min_dist_moved = MIN_DIST;

AXP20X_Class axp;
bool pmu_irq = false;

bool ssd1306_found = false;
bool axp192_found = false;

// bool packetSent;
bool packetQueued;
bool isJoined = false;

// Buffer for Payload frame
static uint8_t txBuffer[11];

// deep sleep support
RTC_DATA_ATTR int bootCount = 0;
esp_sleep_source_t wakeCause;  // the reason we booted this time

char buffer[40];  // Screen buffer

dr_t lorawan_sf; // prefs LORAWAN_SF
char sf_name[40];

unsigned long int ack_req = 0;
unsigned long int ack_rx = 0;

// Same format as CubeCell mappers
void buildPacket(uint8_t txBuffer[]) {
  uint32_t LatitudeBinary;
  uint32_t LongitudeBinary;
  uint16_t altitudeGps;
  // uint8_t hdopGps;
  uint8_t sats;
  uint16_t speed;

  LatitudeBinary = ((gps_latitude() + 90) / 180.0) * 16777215;
  LongitudeBinary = ((gps_longitude() + 180) / 360.0) * 16777215;
  altitudeGps = (uint16_t)gps_altitude();
  speed = (uint16_t)gps_speed();  // convert from float
  sats = gps_sats();

  sprintf(buffer, "Lat: %f, ", gps_latitude());
  Serial.print(buffer);
  sprintf(buffer, "Long: %f, ", gps_longitude());
  Serial.print(buffer);
  sprintf(buffer, "Alt: %f, ", gps_altitude());
  Serial.print(buffer);
  sprintf(buffer, "Sats: %d", sats);
  Serial.println(buffer);

  txBuffer[0] = (LatitudeBinary >> 16) & 0xFF;
  txBuffer[1] = (LatitudeBinary >> 8) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;
  txBuffer[3] = (LongitudeBinary >> 16) & 0xFF;
  txBuffer[4] = (LongitudeBinary >> 8) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;
  txBuffer[6] = (altitudeGps >> 8) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  txBuffer[8] = ((unsigned char *)(&speed))[0];

  uint16_t batteryVoltage = ((float_t)((float_t)(axp.getBattVoltage()) / 10.0) + .5);
  txBuffer[9] = (uint8_t)((batteryVoltage - 200) & 0xFF);

  txBuffer[10] = sats & 0xFF;
}

// Send a packet, if one is warranted
bool trySend() {
  float now_lat = gps_latitude();
  float now_long = gps_longitude();
  unsigned long int now_millis = millis();

  // Here we try to filter out bogus GPS readings.
  // It's not correct, and there should be a better indication from GPS that the fix is invalid
  if (gps_hdop() <= 0 || gps_hdop() > 50 || now_lat == 0.0               // Not fair to the whole equator
      || now_lat > 90.0 || now_lat < -90.0 || now_long == 0.0            // Not fair to King George
      || now_long < -180.0 || now_long > 180.0 || gps_altitude() == 0.0  // Not fair to the ocean
      || gps_sats() < 4
  )
    return false;  // Rejected as bogus GPS reading.

  // Don't attempt to send or update until we join Helium
  if (!isJoined)
    return false;

  // LoRa is not ready for a new packet, maybe still sending the last one.
  if (!LMIC_queryTxReady())
    return false;

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
    return false;

  // distance from last transmitted location
  float dist_moved = gps_distanceBetween(last_send_lat, last_send_lon, now_lat, now_long);

#if 0
  snprintf(buffer, sizeof(buffer), "Lat: %10.6f\n", gps_latitude());
  screen_print(buffer);
  snprintf(buffer, sizeof(buffer), "Long: %10.6f\n", gps_longitude());
  screen_print(buffer);
  snprintf(buffer, sizeof(buffer), "HDOP: %4.2fm\n", gps_hdop());
  screen_print(buffer);
#endif

  char because = '?';
#ifdef USE_LOCATION_BLACKLIST
 if (isLocationBlacklisted(now_lat,now_long)) {
    return false;
  } else //This becomes an else if when USE_LOCATION_BLACKLIST is
#endif  
if (justSendNow) {
    justSendNow = false;
    Serial.println("** JUST_SEND_NOW");
    because = '>';
  } else if (dist_moved > min_dist_moved) {
    Serial.println("** MOVING");
    last_moved_millis = now_millis;
    because = 'D';
  } else if (now_millis - last_send_millis > tx_interval_s * 1000) {
    Serial.println("** TIME");
    because = 'T';
  } else {
    return false;  // Nothing to do, go home early
  }

  // SEND a Packet!
  // digitalWrite(RED_LED, LOW);

  // The first distance-moved is crazy, since has no origin.. don't put it on screen.
  if (dist_moved > 1000000)
    dist_moved = 0;

  snprintf(buffer, sizeof(buffer), "\n%d %c %4lus %4.0fm ", ttn_get_count(), because, (now_millis - last_send_millis) / 1000, dist_moved);
  screen_print(buffer);

  // prepare the LoRa frame
  buildPacket(txBuffer);

  // Want an ACK on this one?
  bool confirmed = (LORAWAN_CONFIRMED_EVERY > 0) && (ttn_get_count() % LORAWAN_CONFIRMED_EVERY == 0);
  if (confirmed) {
    Serial.println("ACK requested");
    screen_print("? ");
    digitalWrite(RED_LED, LOW);  // Light LED
    ack_req++;
  }

  // send it!

  packetQueued = true;
  if (!ttn_send(txBuffer, sizeof(txBuffer), LORAWAN_PORT, confirmed)) {
    Serial.println("Surprise send failure!");
    return false;
  }

  last_send_millis = now_millis;
  last_send_lat = now_lat;
  last_send_lon = now_long;

  return true;  // We did it!
}

/*
Mapper namespace
mapper {

}
*/

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

void mapper_save_prefs (void)
{
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

void mapper_erase_prefs (void)
{
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

#if 0
void doDeepSleep(uint64_t msecToWake)
{
  Serial.printf("Entering deep sleep for %llu seconds\n", msecToWake / 1000);

  // not using wifi yet, but once we are this is needed to shutoff the radio hw
  // esp_wifi_stop();

  screen_off(); // datasheet says this will draw only 10ua
  LMIC_shutdown(); // cleanly shutdown the radio

  if (axp192_found) {
    // turn on after initial testing with real hardware
    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF); // LORA radio
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF); // GPS main power
  }

  // FIXME - use an external 10k pulldown so we can leave the RTC peripherals powered off
  // until then we need the following lines
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  // Only GPIOs which are have RTC functionality can be used in this bit map: 0,2,4,12-15,25-27,32-39.
  uint64_t gpioMask = (1ULL << MIDDLE_BUTTON_PIN);

  // FIXME change polarity so we can wake on ANY_HIGH instead - that would allow us to use all three buttons (instead of just the first)
  gpio_pullup_en((gpio_num_t) MIDDLE_BUTTON_PIN);

  esp_sleep_enable_ext1_wakeup(gpioMask, ESP_EXT1_WAKEUP_ALL_LOW);

  esp_sleep_enable_timer_wakeup(msecToWake * 1000ULL); // call expects usecs
  esp_deep_sleep_start();                              // TBD mA sleep current (battery)
}
#endif

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
    justSendNow = true;
  }

  if (EV_TXSTART == message) {
    screen_print("+\v");
    screen_update();
  }
  // We only want to say 'packetSent' for our packets (not packets needed for joining)
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
     * 2 Bytes: Minimum Time (1 to 65535) seconds (18.2 hours) between pings, or 0 no-change, or 0xFFFF to use default
     * 1 Byte:  Battery voltage (2.0 to 4.5) for auto-shutoff, or 0 no-change
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
  for (addr = 1; addr < 127; addr++) {
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

      if (addr == SSD1306_ADDRESS) {
        ssd1306_found = true;
        Serial.println("SSD1306 OLED display");
      }
      if (addr == AXP192_SLAVE_ADDRESS) {
        axp192_found = true;
        Serial.println("AXP192 PMU");
      }
    } else if (err == 4) {
      Serial.print("Unknow i2c device at 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found!\n");
  /* else  Serial.println("done\n"); */
}

/**
  Init the power manager chip

  axp192 power
  DCDC1 0.7-3.5V @ 1200mA max -> OLED // If you turn this off you'll lose comms to the axp192 because the OLED and the axp192 share the same i2c bus
  use ssd1306 sleep mode instead
  DCDC2 -> unused 
  DCDC3 0.7-3.5V @ 700mA max -> ESP32 (keep this on!) 
  LDO1 30mA -> "VCC_RTC" charges GPS backup battery // charges the tiny J13 battery by the GPS to power the GPS ram (for a couple of days), can not be turned off
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

    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);   // LORA radio
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);   // GPS main power
    axp.setLDO3Voltage(3300);                     // For GPS Power.  Can run on 2.7v to 3.6v
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);  // OLED power
    axp.setDCDC1Voltage(3300);                    // for the OLED power
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF); // Unconnected
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF); // "EXTEN" pin, normally unused

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

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, [] { pmu_irq = true; }, FALLING);

    // Configure REG 36H: PEK press key parameter set.  Index values for argument!
    axp.setStartupTime(2);      // "Power on time": 512mS
    axp.setlongPressTime(2);    // "Long time key press time": 2S
    axp.setShutdownTime(2);     // "Power off time" = 8S
    axp.setTimeOutShutdown(1);  // "When key press time is longer than power off time, auto power off"

    // Serial.printf("AC IN: %fv\n", axp.getAcinVoltage());
    // Serial.printf("Vbus: %fv\n", axp.getVbusVoltage());
    Serial.printf("PMIC Temp %0.2f°C\n", axp.getTemp());
    // Serial.printf("TSTemp %f°C\n", axp.getTSTemp());
    // Serial.printf("GPIO0 %fv\n", axp.getGPIO0Voltage());
    // Serial.printf("GPIO1 %fv\n", axp.getGPIO1Voltage());
    // Serial.printf("Batt In: %fmW\n", axp.getBattInpower());
    Serial.printf("Batt: %0.3fv\n", axp.getBattVoltage() / 1000.0);
    Serial.printf("SysIPSOut: %0.3fv\n", axp.getSysIPSOUTVoltage()/1000.0);
    Serial.printf("isVBUSPlug? %s\n", axp.isVBUSPlug() ? "Yes" : "No");
    Serial.printf("isChargingEnable? %s\n", axp.isChargeingEnable() ? "Yes" : "No");
    Serial.printf("ChargeCurrent: %.2fmA\n", axp.getSettingChargeCurrent());
    Serial.printf("ChargeControlCurrent: %d\n", axp.getChargeControlCur());
    Serial.printf("Charge: %d%%\n", axp.getBattPercentage());

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
    //    axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
    axp.enableIRQ(0xFFFFFFFFFF, 1);  // Give me ALL the interrupts you have.
    axp.clearIRQ();
  } else {
    Serial.println("AXP192 not found!");
  }
}

// Perform power on init that we do on each wake from deep sleep
void wakeup() {
  bootCount++;
  wakeCause = esp_sleep_get_wakeup_cause();
  /*
    Not using yet because we are using wake on all buttons being low

    wakeButtons = esp_sleep_get_ext1_wakeup_status();       // If one of these buttons is set it was the reason we woke
    if (wakeCause == ESP_SLEEP_WAKEUP_EXT1 && !wakeButtons) // we must have been using the 'all buttons rule for waking' to support busted boards, assume button
    one was pressed wakeButtons = ((uint64_t)1) << buttons.gpios[0];
  */

  Serial.printf("BOOT #%d!  cause:%d ext1:%08llx\n", bootCount, wakeCause, esp_sleep_get_ext1_wakeup_status());
}

void setup() {
  // Debug
#ifdef DEBUG_PORT
  DEBUG_PORT.begin(SERIAL_BAUD);
#endif
  wakeup();

  Wire.begin(I2C_SDA, I2C_SCL);
  scanI2Cdevice();

  axp192Init();

  // GPS sometimes gets wedged with no satellites in view and only a power-cycle saves it.
  // Here we turn off power and the delay in screen setup is enough time to bonk the GPS
  axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);   // GPS power off

  // Buttons & LED
  pinMode(MIDDLE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);  // Off

  // Hello
  DEBUG_MSG("\n" APP_NAME " " APP_VERSION "\n");

  mapper_restore_prefs(); // Fetch saved settings

  // Don't init display if we don't have one or we are waking headless due to a timer event
  if (0 && wakeCause == ESP_SLEEP_WAKEUP_TIMER)
    ssd1306_found = false;  // forget we even have the hardware

  if (ssd1306_found)
    screen_setup();

  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);   // GPS power on, so it has time to setttle.
  
  // Show logo on first boot after removing battery
#ifndef ALWAYS_SHOW_LOGO
  if (bootCount <= 1)
#endif
  {
    screen_print(APP_NAME " " APP_VERSION, 0, 0); // Above the Logo
    screen_print(APP_NAME " " APP_VERSION "\n"); // Add it to the log too
    
    screen_show_logo();
    screen_update();
    delay(LOGO_DELAY);
  }

  // Helium setup
  if (!ttn_setup()) {
    screen_print("[ERR] Radio module not found!\n");

    if (REQUIRE_RADIO) {
      screen_off();
      sleep_forever();
    }
  } else {
    ttn_register(lora_msg_callback);
    ttn_join();
    ttn_adr(LORAWAN_ADR);
  }

  // Might have to add a longer delay here  
  gps_setup();  // Init GPS baudrate and messages

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
      ;  // ?? What to do here
  }
}

void update_activity() {
  float bat_volts = axp.getBattVoltage() / 1000;
  float charge_ma = axp.getBattChargeCurrent();
  // float discharge_ma = axp.getBatChargeCurrent();

  if (axp192_found && axp.isBatteryConnect() && bat_volts < battery_low_voltage && charge_ma < 99.0) {
    Serial.println("Low Battery OFF");
    screen_print("\nLow Battery OFF\n");
    delay(4999);  // Give some time to read the screen
    clean_shutdown();
  }

  if (millis() - last_moved_millis > rest_wait_s * 1000)
    tx_interval_s = rest_tx_interval_s;
  else
    tx_interval_s = stationary_tx_interval_s;
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
    irq_name = "VbusPlugIn";
  else if (axp.isVbusRemoveIRQ())
    irq_name = "VbusRemove";
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
  else if (axp.isChargingCurrentLessIRQ())
    irq_name = "ChargingCurrentLess";
  else if (axp.isDC2VoltageLessIRQ())
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
  void(*func)(void);
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
  axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);   // Kill LORA radio
  gps_passthrough();
  // Does not return.
}
void menu_experiment(void){
  static boolean power_toggle = true;

  Serial.printf("%f mA  %f mW\n", axp.getBattChargeCurrent() - axp.getBattDischargeCurrent(), axp.getBattInpower());

  axp.setPowerOutPut(AXP192_LDO3, power_toggle ? AXP202_ON : AXP202_OFF);  // GPS main power
  power_toggle = !power_toggle;
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

struct menu_entry menu[] = {{"Send Now", menu_send_now},         {"Power Off", menu_power_off},     {"Distance +", menu_distance_plus},
                            {"Distance -", menu_distance_minus}, {"Time +", menu_time_plus},        {"Time -", menu_time_minus},
                            {"Change SF", menu_change_sf},       {"Flush Prefs", menu_flush_prefs}, {"USB GPS", menu_gps_passthrough},
                            {"Danger", menu_experiment}};
#define MENU_ENTRIES (sizeof(menu) / sizeof(menu[0]))

const char *menu_prev;
const char *menu_cur;
const char *menu_next;
boolean in_menu = false;
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

void loop() {
  gps_loop();
  ttn_loop();

  if (in_menu && millis() - menu_idle_start > (5 * 1000))
    in_menu = false;

  screen_loop(tx_interval_s, min_dist_moved, sf_name, gps_sats(), in_menu, menu_prev, menu_cur, menu_next, is_highlighted);

  update_activity();

  /*
    if (packetSent) {
      packetSent = false;
    } */

  // If any interrupts on PMIC, report the name
  if (axp192_found && pmu_irq) {
    const char *irq_name;
    pmu_irq = false;
    axp.readIRQ();
    irq_name = find_irq_name();

    if (axp.isPEKShortPressIRQ())
      menu_press();
    else if (axp.isPEKLongtPressIRQ())  // They want to turn OFF
      menu_power_off();
    else {
      snprintf(buffer, sizeof(buffer), "\n* %s ", irq_name);
      screen_print(buffer);
    }
    axp.clearIRQ();
  }

  static uint32_t pressTime = 0;
  if (!digitalRead(MIDDLE_BUTTON_PIN)) {
    // Pressure is on
    if (!pressTime) {  // just started a new press
      pressTime = millis();
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

    if (millis() - pressTime > 1000) {
      // Was a long press
    } else {
      // Was a short press
    }
    pressTime = 0;  // Released
  }

  if (trySend()) {
    // Good send
    if (axp192_found)
      axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
  } else {
    // Nothing sent.
    // Do NOT delay() here.. the LoRa receiver and join housekeeping also needs to run!
  }
}
