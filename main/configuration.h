/*
TTGO T-Beam Mapper for Helium
Copyright (C) 2021 by @Max_Plastix

Forked from:
TTGO T-BEAM Tracker for The Things Network
Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

This code requires LMIC library by Matthijs Kooijman
https://github.com/matthijskooijman/arduino-lmic

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
#pragma once

// -----------------------------------------------------------------------------
// Version
// -----------------------------------------------------------------------------
#define APP_NAME "MaxP Mapper"
#define APP_VERSION "1.7.8"  // 2022-Nov-18

// -----------------------------------------------------------------------------
// CONFIGURATION
// Stuff you might reasonably want to change is here:
// -----------------------------------------------------------------------------

// All time settings here are SECONDS

// Minimum Distance between Mapper reports.  This is your MAIN knob to turn for more/fewer uplink packets.
// SMALLER distance: More packets, more dots on the map, more DC spent, more power consumed
// (If you set this to a very small value, it will still be rate-limited by how often your Region allows back-to-back
// Uplink packets.) LARGER distance: Fewer packets, might miss some hexes, conserves DC, battery might last longer Note
// that a hex is about 340m across.  Ideally, you want at least two uplinks in each hex to map it.
#define MIN_DIST 70.0  // Minimum distance in meters from the last sent location before we  send again.

// If we are not moving at least MIN_DIST meters away from the last uplink, when should we send a redundant
// Mapper Uplink from the same location?  This Heartbeat or ping isn't all that important for mapping, but might be
// useful for time-at-location tracking or other monitoring.  You can safely set this value very high.
#define STATIONARY_TX_INTERVAL (5 * 60)  // Send one uplink at least once every N seconds
#define NEVER_REST 0                     // Change to 1 if you want to always send at THIS rate, with no slowing or sleeping.

// After being stationary for a long while, we move to a slower heartbeat interval:
#define REST_WAIT           (20 * 60)  // If we still haven't moved in this many seconds, start sending even slower..
#define REST_TX_INTERVAL    (30 * 60)  // Slow resting ping frequency in seconds

// This last stage is a low-power sleep to conserve battery when the Mapper has not moved for a long time.
// This one is a difficult compromise:  Waking up to boot & power on the GPS is not a fast operation,
// so we want to avoid it as much as possible. There is no other motion sensor, so if we make it too long,
// we miss the first minutes of each motion while sleeping.
// Note that USB Power will prevent this low-power sleep, and also wake us up from it.
// A button press will also wake from sleep, but takes some time to initialise and re-acquire
#define SLEEP_WAIT          (2 * 60 * 60)  // If we STILL haven't moved in this long, turn off the GPS to save power
// For a vehicle application where USB Power appears BEFORE motion, this can be set very high without missing anything:
#define SLEEP_TX_INTERVAL   (1 * 60 * 60)  // Wake up and check position every now and then to see if movement happened

// When searching for a GPS Fix, we may never find one due to obstruction, noise, or reduced availability.
// Note that GPS Lost also counts as no-movement, so the Sleep tier above still applies
#define GPS_LOST_WAIT (5 * 60)   // How long to wait for a GPS fix before declaring failure
#define GPS_LOST_PING (15 * 60)  // Without GPS reception, how often to send a non-mapper status packet

#define SCREEN_IDLE_OFF_S  (2 * 60) // If there are no Uplinks or button presses sent for this long, turn the screen off.
#define MENU_TIMEOUT_S      5       // Seconds to wait before exiting the menu.

// Below BATTERY_LOW_VOLTAGE, power off until USB power allows charging.
// The PMIC also has a (safety) turn-off at 2.6v, much lower than this setting. 
// We use a conservative cutoff here since the battery will last longer if it's not repeatedly run to minimum.
// A new-condition 18650 cell gives around 9Wh (Molicell) when discharged to the absolute lowest 2.5v
// 8110mWh to 3.3v
// 8875mWh to 3.1v
// 9210mWh to 2.5v
#define BATTERY_LOW_VOLTAGE 3.1

// Confirmed packets (ACK request) conflict with the function of a Mapper and should not normally be enabled.
// In areas of reduced coverage, the Mapper will try to send each packet six or more times with different SF/DR.
// This causes irregular results and the location updates are infrequent, unpredictable, and out of date.
// (0 means never, 1 means always, 2 every-other-one..)
#define LORAWAN_CONFIRMED_EVERY 0  // Request Confirmation message every N Uplinks 

// Spreading Factor (Data Rate) determines how long each 11-byte Mapper Uplink is on-air, and how observable it is.
// SF10 is about two seconds per packet, and the highest range, while SF7 is a good compromise
// for moving vehicles and reasonable mapping observations.
#define LORAWAN_SF DR_SF7  // Spreading factor (recommended DR_SF7 for network map purposes)

// Deadzone defines a circular area where no map packets will originate.
// This is useful to avoid sending many redundant packets in your own driveway or office, or just for local privacy.
// You can "re-center" the deadzone from the screen menu.
// Set Radius to zero to disable altogether.
// (Thanks to @Woutch for the name)
#ifndef DEADZONE_LAT
#define DEADZONE_LAT 34.5678
#endif
#ifndef DEADZONE_LON
#define DEADZONE_LON -123.4567
#endif
#ifndef DEADZONE_RADIUS_M
#define DEADZONE_RADIUS_M 500  // meters
#endif

// Uplink format, defaults into a shorter payload. 7 bytes instead of 11. The altitude is in blocks of 150m
// and the sattelite count is combined into a single byte. The decoder takes care of this.
#define UPLINK_SHORT 

// There are some extra non-Mapper Uplink messages we can send, but there's no good way to avoid sending these
// to all Integrations from the Decoder.  This causes (normal) Error messages on the Console because Mapper will throw
// them out for having no coordinates.  It doesn't hurt anything, as they are correctly filtered by the Decoder, but if
// you don't like seeing Integration Errors, then set these to 0.  Set these to 1 for extra non-mapper messages.
#ifndef SEND_GPSLOST_UPLINKS
#define SEND_GPSLOST_UPLINKS 0  // GPS Lost messages
#endif
#ifndef SEND_STATUS_UPLINKS
#define SEND_STATUS_UPLINKS  0  // USB Connect/disconnect messages
#endif

// -----------------------------------------------------------------------------
// Less common Configuration items
// -----------------------------------------------------------------------------

// Select which T-Beam board is being used. Only uncomment one.
//#define T_BEAM_V07  // AKA Rev0 (first board released) UNTESTED!  Expect bugs.
#define T_BEAM_V10  // AKA Rev1 (second board released), this is the common "v1.1"

#define LOGO_DELAY 2000  // Time to show logo on first boot (ms)

#define DEBUG_PORT Serial   // Serial debug port
#define SERIAL_BAUD 115200  // Serial debug baud rate (note that bootloader is fixed at 115200)

// Never enable ADR on Mappers because they are moving, so we don't want to adjust
// anything based on packet reception.
#define LORAWAN_ADR 0  // Do not enable ADR

// If you are having difficulty sending messages to TTN after the first successful send,
// uncomment the next option and experiment with values (~ 1 - 5)
//#define CLOCK_ERROR             5

// If using a single-channel gateway, uncomment this next option and set to your gateway's channel
//#define SINGLE_CHANNEL_GATEWAY  0

// -----------------------------------------------------------------------------
// DEBUG
// -----------------------------------------------------------------------------
#ifdef DEBUG_PORT
#define DEBUG_MSG(...) DEBUG_PORT.printf(__VA_ARGS__)
#else
#define DEBUG_MSG(...)
#endif

// Verbose LoRa message callback reporting
// #define DEBUG_LORA_MESSAGES

// -----------------------------------------------------------------------------
// Custom messages
// -----------------------------------------------------------------------------

#define EV_QUEUED 100
#define EV_PENDING 101
#define EV_ACK 102
#define EV_RESPONSE 103

// -----------------------------------------------------------------------------
// General
// -----------------------------------------------------------------------------
// Wiring for I2C OLED display:
//
// Signal     Header   OLED
// 3V3         7       VCC
// GND         8       GND
// IO22(SCL)   9       SCL
// IO21(SDA)   10      SDA
#define I2C_SDA 21
#define I2C_SCL 22

#if defined(T_BEAM_V07)
#define LED_PIN 14
#define MIDDLE_BUTTON_PIN 39
#elif defined(T_BEAM_V10)
#define MIDDLE_BUTTON_PIN 38  // Middle button SW5, BUTTON0, GPIO38.  Low active
#endif

#define RED_LED 4  // GPIO4 on T-Beam v1.1

// -----------------------------------------------------------------------------
// GPS
// -----------------------------------------------------------------------------

#define GPS_SERIAL_NUM 1     // SerialX
#define GPS_BAUDRATE 115200  // Make haste!  NMEA is big.. go fast
#define USE_GPS 1

#if defined(T_BEAM_V07)
#define GPS_RX_PIN 12
#define GPS_TX_PIN 15
#elif defined(T_BEAM_V10)  // Or T-Beam v1.1
#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
#define GPS_INT 37  // 30ns accurate timepulse from Neo-6M pin 3
#endif

// -----------------------------------------------------------------------------
// LoRa SPI
// -----------------------------------------------------------------------------

#define SCK_GPIO 5
#define MISO_GPIO 19
#define MOSI_GPIO 27
#define NSS_GPIO 18
#if defined(T_BEAM_V10)
#define RESET_GPIO 14
#else
#define RESET_GPIO 23
#endif
#define DIO0_GPIO 26
#define DIO1_GPIO 33  // Note: not really used on this board
#define DIO2_GPIO 32  // Note: not really used on this board

// -----------------------------------------------------------------------------
// AXP192 (Rev1-specific options)
// -----------------------------------------------------------------------------

#define GPS_POWER_CTRL_CH 3
#define LORA_POWER_CTRL_CH 2
#define PMU_IRQ 35
