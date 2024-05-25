#include "Adafruit_TLC5947.h"
#define NUM_TLC5947 4

#define data   5
#define clock   6
#define latch   7
#define oe  -1

Adafruit_TLC5947 tlc = Adafruit_TLC5947(NUM_TLC5947, clock, data, latch);

#include <math.h>
#include <Timezone.h>

TimeChangeRule CDT = { "CDT", Second, Sun, Mar, 2, -300 };    //Daylight time = UTC - 5 hours
TimeChangeRule CST = { "CST", First, Sun, Nov, 2, -360 };     //Standard time = UTC - 6 hours
Timezone Central(CDT, CST);

Timezone* timezones[] = {&Central};
Timezone* tz;
uint8_t tzIndex = 0;
time_t utc, local;
TimeChangeRule *tcr;

#include <EEPROM.h>
#include <NeoGPS_cfg.h>
#include <GPSTime.h>
#include <Garmin/GrmNMEA.h>
#include <Streamers.h>
#include <NeoHWSerial.h>
#include <NeoHWSerial_private.h>
#define gpsPort NeoSerial1
#define GPS_PORT_NAME "NeoSerial1"
//#define DEBUG_PORT NeoSerial

static GarminNMEA gps;
static gps_fix    fix;

#include <Wire.h>

#define WITH_LCD 1

#include <Encoder.h>
Encoder myEnc(3, 4);

#ifdef WITH_LCD
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

//Global varibles:

uint16_t brightness[81];  //array for the brightness values to be sent to the LEDs
uint16_t previousBrightness[81]; //array for the previous brightness values sent to the LEDs

const uint16_t maxBrightness = 255;
const uint16_t maxPWMBrightness = 4095;

int8_t last_arrow  = 1;

uint8_t main_menu_array[] = {2, 3, 4};
uint8_t full_menu_array[] = {2, 3, 4, 5, 6, 7};
uint8_t menu_offset = 0;

byte down_arrow[] = {
  B00000,
  B11111,
  B01110,
  B00100,
  B00000,
  B11111,
  B01110,
  B00100
};

byte up_arrow[] = {
  B00100,
  B01110,
  B11111,
  B00000,
  B00100,
  B01110,
  B11111,
  B00000
};

uint8_t t_second, t_minute, t_hour, t_day, t_month = 0;
uint8_t  primary_display_mode = 1;
int8_t menu_number = 0;
int16_t last, value, previous_encoder_pos, encoder_pos = 0;
uint16_t t_millis, main_nth_billion, alt_nth_billion, main_nth_million, alt_nth_million,
         fake_pps_offset, t_year = 0, last_unix_leap;

uint32_t alt_rate, current_millis, last_second_start_millis, encoder_t1, animationStartTime, deltaT = 0;
int64_t t_unix, next_new_year_unix, main_count, alt_count, alt_count_start_time = 0;

const int64_t unix_clock_epoch = 1517508000;

const char twenty_spaces[21] = "                    ";

char display_time[21];
char display_date[21];

boolean second_advanced, clock_strings_updated, recalculate_local_time_next_PPS, led_on, main_next_billion_Strings_updated,
        main_next_million_Strings_updated, alt_next_million_Strings_updated,
        alt_next_billion_Strings_updated, unix_end_Strings_updated, day_changed,
        lcd_day_changed, invertOutputs, fake_pps = false;

int time_zone = -6;
uint16_t century = 2000;

float alt_rate_divisor, rate, scrollWidth;

const byte numChars = 87;
char receivedChars[numChars];

boolean newData = false;

uint16_t pwmBri[(maxBrightness + 1)];

void GPSisr( uint8_t c ) {
  gps.handle( c );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  !!! What happens when the GPS PPS pin goes high? !!!
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PPS() {

  if (!fake_pps) {
    PORTB |= 1 << PORTB7;
    led_on = true;
    t_millis = 0;
    last_second_start_millis = millis();
  }

  else {
    last_second_start_millis = millis() - fake_pps_offset;
    t_millis = fake_pps_offset;
    fake_pps = false;
  }

  second_advanced = true;
  main_count++;
  t_unix++;
  t_second++;

  if (t_second >= 60) {

    t_second = 0;
    t_minute++;

    if (t_minute == 60)
    {
      t_minute = 0;
    }

    clock_strings_updated = false;
    recalculate_local_time_next_PPS = true;
  }

  if (recalculate_local_time_next_PPS || primary_display_mode == 3 ||  menu_number == 3) {
    setTime(t_unix);
    utc = now();
    local = Central.toLocal(utc, &tcr);
    t_hour = hourFormat12(local);
    t_day = day(local);
    t_month = month(local);
    t_year = year(local);

    recalculate_local_time_next_PPS = false;
  }
}


boolean recvInProgress = false;

static void handleRxChar(uint8_t c) {

  const char startMarker = '<';
  const char endMarker = '>';
  static byte ndx;

  if (recvInProgress) {
    if (c != endMarker) {
      receivedChars[ndx] = c;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      recvInProgress = false;
      newData = true;
    }
  }

  else if (c == startMarker) {
    recvInProgress = true;
    ndx = 0;
  }
}


void setup() {

  //DEBUG_PORT.begin(38400);
  //DEBUG_PORT.print("Program started!");


  gpsPort.begin(38400);
  gpsPort.attachInterrupt(GPSisr);

  lcd.begin(20, 4);
  lcd.createChar(0, up_arrow);
  lcd.createChar(1, down_arrow);

  lcd_print("     POWER ON       ", twenty_spaces, twenty_spaces, twenty_spaces);

  tlc.begin();

  for (int j = 0; j < 80; j++) {
    tlc.setPWM(j, maxPWMBrightness);
  }
  tlc.write();

  delay(500);

  lcd_print("  Waiting for GPS   ", twenty_spaces, twenty_spaces, twenty_spaces);
  delay(250);

  boolean gps_fix = false;
  uint8_t num_sats = 0;

  while (num_sats == 0) {
    if (gps.available( gpsPort )) {
      fix = gps.read();
    }

    if (fix.valid.status) {
      num_sats = fix.satellites;
    }
  }

  last_unix_leap = GPSTime::leap_seconds + 9;

  lcd_print("     GPS FOUND!     ", twenty_spaces, twenty_spaces,  twenty_spaces);

  delay(500);

  //century = EEPROMReadInt(3);
  century = 2000;

  tz = timezones[tzIndex];

  for (uint16_t appBri = 0; appBri <= maxBrightness; appBri++)
  {
    pwmBri[appBri] = round(pow (2.0, ((float)appBri / ((float)maxBrightness * ( (log10(2.0)) /
                                      (log10((float)maxPWMBrightness)) )))));
  }

  //force apparent brightness 0 to PWM brightness 0 (it will otherwise round to 1)
  //and force max apparent brightness to max PWMBrightness
  pwmBri[0] = 0;
  pwmBri[maxBrightness] = maxPWMBrightness;

  DDRB |= 1 << DDB7;
  pinMode(2, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (2), PPS, RISING);

  NeoSerial.attachInterrupt(handleRxChar);
  NeoSerial.begin(115200);

  lcd_print("  SETUP COMPLETE!   ", twenty_spaces, twenty_spaces, twenty_spaces);
  delay(500);
  //DEBUG_PORT.print("SETUP COMPLETE!");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  !!!START MAIN LOOP!!!
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  const int8_t num_defaults = 7; //How many default screens does the LCD cycle through?

  static uint8_t last_primary_display_mode, last_num_sats, fast_turn_count,
         animation, num_animations;

  static uint16_t last_hdop, last_vdop, last_pdop;

  static uint16_t leap_offset, unix_leap;

  static int16_t last_alt_count;

  static uint64_t main_next_billion, main_next_million, alt_next_million, alt_next_billion;

  static uint32_t next_screen_update, encoder_t2, encoder_t3, alt_count_start_ms,
         timeout, last_millis, end_light_show_buffer_time, last_astro_data_timeout,
         last_LS_millis;

  static int32_t internal_encoder_val, previous_encoder_val;

  static boolean menu_mode, menu_pushed_to_lcd, clicked, turning, fast_turn_plus,
         fast_turn_minus, very_fast_turn, new_astro_data, animation_in_progress, waiting,
         animation_initialized, have_some_astro_data;

  static char AM_PM[5];

  static char main_next_N_million[21];
  static char main_next_N_billion[21];
  static char alt_next_N_million[21];
  static char alt_next_N_billion[21];

  static char main_start_date[21];
  static char main_start_time[21];

  static char main_next_billion_date[21];
  static char main_next_billion_time[21];

  static char main_next_million_date[21];
  static char main_next_million_time[21];

  static char alt_next_billion_date[21];
  static char alt_next_billion_time[21];

  static char alt_next_million_date[21];
  static char alt_next_million_time[21];

  static char unix_end_date[21];
  static char unix_end_time[21];

  static char alt_counts_string[21];

  static char alt_mil_date[21];
  static char alt_mil_time[21];
  static char alt_counts_mil_string[21];

  static char current_date[21];
  static char current_time[21];

  static char minute_string[3];
  static char second_string[3];

  static char hour_string[3];
  static char hour_minute_string[7];
  static char end_time_stamp_string[13];

  static char all_dops[21], sat_string[21];
  static char h_dop[5], v_dop[5], p_dop[5];

  static char illum[21], phase_name[21], sunrise[21], sunset[21];

  static uint8_t  last_minute, last_hour, last_day;


  uint8_t tenths_bin, dop_bin;
  char counts_per_second[21];

  static boolean previous_button;


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //  !!! Read Astro-data over serial !!!
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (newData) {

    //clear strings
    strcpy(illum, "");
    strcpy(phase_name, "");
    strcpy(sunrise, "");
    strcpy(sunset, "");

    uint8_t astro_string_num = 0;
    uint8_t char_index = 0;
    uint8_t string_char_index = 0;
    const char delim = ',';

    while (astro_string_num < 4 && current_millis < timeout) {

      current_millis = millis();

      if (receivedChars[char_index] != delim) {

        switch (astro_string_num) {
          case 0:
            illum[string_char_index] = receivedChars[char_index];
            break;

          case 1:
            phase_name[string_char_index] = receivedChars[char_index];
            break;

          case 2:
            sunrise[string_char_index] = receivedChars[char_index];
            break;

          case 3:
            sunset[string_char_index] = receivedChars[char_index];
            break;
        }
        string_char_index++;
      }

      else {
        astro_string_num++;
        string_char_index = 0;
      }
      char_index++;
    }

    newData = false;
    have_some_astro_data = true;
    new_astro_data = true;
    last_astro_data_timeout = current_millis + 300000;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //  Internal time-keeping.
  //
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  current_millis = millis();

  if (current_millis != last_millis) {

    t_millis = abs(current_millis - last_second_start_millis);
    timeout = current_millis + 1000;

    if (led_on && (t_millis >= 250)) {
      PORTB &= ~(1 << PORTB7);
      led_on = false;
    }

    if (have_some_astro_data && (current_millis > last_astro_data_timeout)) {
      have_some_astro_data = false;
    }

    //  If no PPS 10 ms after one is expected, advance clock anyway
    if ((t_millis >= 1010) && (t_millis < 1500)) {
      fake_pps = true;
      fake_pps_offset = abs(t_millis - 1000);
      PPS();
    }

    last_millis = current_millis;
  }


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //   Parse GPS Data
  //
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (gps.available( gpsPort )) {

    fix = gps.read();

    if (fix.valid.time) {

      leap_offset = 18 - GPSTime::leap_seconds;
      unix_leap = GPSTime::leap_seconds + 9;

      if ((last_unix_leap != unix_leap) || (fix.dateTime.seconds == 60)) {
        t_second = 60;
        lcd_print("LEAP SECOND DETECTED", "LEAP SECOND DETECTED", "LEAP SECOND DETECTED", "LEAP SECOND DETECTED");
        next_screen_update = current_millis + 10000;
        recalculate_local_time_next_PPS = true;
        last_unix_leap = unix_leap;
      }

      if (
        (t_second != 0)
        && (t_second != 59)
        && (fix.dateTime.seconds  != 0)
        && (fix.dateTime.seconds  != 59)
        && (fix.dateTime.seconds  != 60))
      {
        if (
          ((t_second != fix.dateTime.seconds) && (abs(t_second - fix.dateTime.seconds) != 1))
          || ((t_minute != fix.dateTime.minutes) && (abs(t_minute - fix.dateTime.minutes) != 1))
        )
        {
          t_second = fix.dateTime.seconds;
          t_minute = fix.dateTime.minutes;

          setTime(fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds, fix.dateTime.date, fix.dateTime.month, fix.dateTime.year + century);
          t_unix = utc = now();
          main_count = t_unix - unix_clock_epoch + leap_offset;
          clock_strings_updated = true;
          recalculate_local_time_next_PPS = true;
        }
      }
    }
  }




  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //   Recalculate local time, only when needed
  //
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (!clock_strings_updated) {

    if (last_day != t_day) {

      day_changed = true;

      if (menu_number == 3) //push new date to LCD if showing current time
      {
        lcd_day_changed = true;
      }
      last_day = t_day;
    }

    strcpy(AM_PM, "");

    if (isAM(local)) {
      strcat(AM_PM, " AM ");
    }
    else {
      strcat(AM_PM, " PM ");
    }

    strcpy(end_time_stamp_string, "");
    strcat(end_time_stamp_string, AM_PM);
    strcat(end_time_stamp_string, tcr -> abbrev);
    strcat(end_time_stamp_string,  "     ");

    if (t_minute < 10) {
      char min_bin[2] = "";
      itoa(t_minute, min_bin, 10);
      strcpy(minute_string, "");
      strcat(minute_string, "0");
      strcat(minute_string, min_bin);
    }

    else {
      char min_bin[3] = "";
      itoa(t_minute, min_bin, 10);
      strcpy(minute_string, "");
      strcat(minute_string, min_bin);
    }

    char hour_bin[3] = "";
    itoa (t_hour, hour_bin, 10);
    strcpy(hour_string, "");
    strcat(hour_string, hour_bin);

    strcpy(hour_minute_string, "");
    strcat(hour_minute_string, hour_string);
    strcat(hour_minute_string, ":");
    strcat(hour_minute_string, minute_string);
    strcat(hour_minute_string, ":");

    if (t_second < 10) {
      char sec_bin[2] = "";
      itoa (t_second, sec_bin, 10);
      strcpy(second_string, "");
      strcat(second_string, "0");
      strcat(second_string, sec_bin);
    }

    else {
      char sec_bin[3] = "";
      itoa (t_second, sec_bin, 10);
      strcpy(second_string, "");
      strcat(second_string, sec_bin);
    }

    strcpy(current_time, "");
    strcat(current_time, hour_minute_string);
    strcat(current_time, second_string);
    strcat(current_time, end_time_stamp_string);

    if (day_changed) {

      char month_string[3] = "";
      itoa(t_month, month_string, 10);

      char day_string[3] = "";
      itoa(t_day, day_string, 10);

      char year_string[5] = "";
      itoa(t_year, year_string, 10);

      strcpy(current_date, "");
      strcat(current_date, dayShortStr(weekday(local)));
      strcat(current_date, ", ");
      strcat(current_date, month_string);
      strcat(current_date, "/");
      strcat(current_date, day_string);
      strcat(current_date, "/");
      strcat(current_date, year_string);
      strcat(current_date, "    ");
      day_changed = false;
    }
    clock_strings_updated = true;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //   Update counters and clock when the unix count advances from either PPS or internal clock
  //
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (second_advanced) {

    if (primary_display_mode != 2) {

      if (primary_display_mode == 1) { //display main count

        display_number(main_count, maxBrightness);

        if (t_unix >= main_next_million) {
          main_next_million_Strings_updated = false;
        }

        if (t_unix >= main_next_billion) {
          alt_next_billion_Strings_updated = false;
        }

        //If we reach an even Billion, flash the number on the display a few times
        if ((main_count % 1000000000) == 0) {

          for (int i = 0; i < 20; i++) {
            display_number(main_count, maxBrightness);
            delay(250);
            display_number(0, maxBrightness);
            delay(250);

            main_next_billion_Strings_updated = false;
          }
        }
      }

      else if (primary_display_mode == 3) { //clock mode

        uint8_t hour_tens = (t_hour / 10) % 10;
        uint8_t hour_ones = t_hour % 10;

        if (hour_tens == 0)
          hour_tens = 10;

        uint8_t minute_tens = (t_minute / 10) % 10;
        uint8_t minute_ones = t_minute % 10;

        uint8_t second_tens = (t_second / 10) % 10;
        uint8_t second_ones = t_second  % 10;

        set_digit(0, second_ones, maxBrightness);
        set_digit(1, second_tens, maxBrightness);
        set_digit(2, 35, maxBrightness);
        set_digit(3, minute_ones, maxBrightness);
        set_digit(4, minute_tens, maxBrightness);
        set_digit(5, 35, maxBrightness);
        set_digit(6, hour_ones, maxBrightness);
        set_digit(7, hour_tens, maxBrightness);
        set_digit(8, 10, maxBrightness);
        set_digit(9, 10, maxBrightness);

        tlc.write();
      }

      else if (primary_display_mode == 4) { //unix time
        display_number(t_unix, maxBrightness);
      }

      else if (primary_display_mode == 6) { //New Years

        int64_t seconds_to_new_year = abs(next_new_year_unix - t_unix);

        display_number(seconds_to_new_year, maxBrightness);

        //If we reach New Year's, flash "0" on the display a few times.
        if (seconds_to_new_year == 0) {
          lcd_print(twenty_spaces, "   HAPPY NEW YEAR!  ", twenty_spaces, twenty_spaces);
          for (int i = 0; i < 20; i++) {
            display_number_with_zeros(0, maxBrightness);
            delay(250);
            for (int j = 0; j < 80; j++) {
              tlc.setPWM(j, 0);
            }
            tlc.write();
            delay(250);
          }

          calculate_next_new_year();
        }

      }
    }

    if (menu_number == 3) { //if displaying current date and time

      if (t_minute != last_minute)
      {
        if (t_minute < 10) {
          char min_bin[2] = "";
          itoa(t_minute, min_bin, 10);
          strcpy(minute_string, "");
          strcat(minute_string, "0");
          strcat(minute_string, min_bin);
        }

        else {
          char min_bin[3] = "";
          itoa(t_minute, min_bin, 10);
          strcpy(minute_string, "");
          strcat(minute_string, min_bin);
        }

        if (t_hour != last_hour) {
          char hour_bin[3] = "";
          itoa (t_hour, hour_bin, 10);
          strcpy(hour_string, "");
          strcat(hour_string, hour_bin);
          last_hour = t_hour;
        }

        strcpy(hour_minute_string, "");
        strcat(hour_minute_string, hour_string);
        strcat(hour_minute_string, ":");
        strcat(hour_minute_string, minute_string);
        strcat(hour_minute_string, ":");

        last_minute = t_minute;
      }

      if (t_second < 10) {
        char sec_bin[2] = "";
        itoa (t_second, sec_bin, 10);
        strcpy(second_string, "");
        strcat(second_string, "0");
        strcat(second_string, sec_bin);
      }

      else {
        char sec_bin[3] = "";
        itoa (t_second, sec_bin, 10);
        strcpy(second_string, "");
        strcat(second_string, sec_bin);
      }

      strcpy(current_time, "");
      strcat(current_time, hour_minute_string);
      strcat(current_time, second_string);
      strcat(current_time, end_time_stamp_string);

      lcd.setCursor (0, 4);
      lcd.print(current_time);

      if (lcd_day_changed) { //if midnight happens to occur while the current time is being displayed
        lcd.setCursor (0, 3);
        lcd.print(current_date);
        lcd_day_changed = false;
      }
    }
    second_advanced = false;
  }

  if (menu_number == 4) { //If GPS info is being displayed

    boolean something_changed = false;

    if (fix.pdop != last_pdop) {

      char dop_bin_char[3];
      char tenths_bin_char[2];
      strcpy(tenths_bin_char, "");
      strcpy(dop_bin_char, "");
      dop_bin = fix.pdop / 1000;
      itoa(dop_bin, dop_bin_char, 10);
      tenths_bin = (fix.pdop - (dop_bin * 1000)) / 100;
      itoa(tenths_bin, tenths_bin_char, 10);

      strcpy(p_dop, "");
      strcat(p_dop, dop_bin_char);
      strcat(p_dop, ".");
      strcat(p_dop, tenths_bin_char);

      something_changed = true;
      last_pdop = fix.pdop;
    }

    if (fix.hdop != last_hdop) {
      char dop_bin_char[3];
      char tenths_bin_char[2];
      strcpy(tenths_bin_char, "");
      strcpy(dop_bin_char, "");
      dop_bin = fix.hdop / 1000;
      itoa(dop_bin, dop_bin_char, 10);
      tenths_bin = (fix.hdop - (dop_bin * 1000)) / 100;
      itoa(tenths_bin, tenths_bin_char, 10);

      strcpy(h_dop, "");
      strcat(h_dop, dop_bin_char);
      strcat(h_dop, ".");
      strcat(h_dop, tenths_bin_char);

      something_changed = true;
      last_hdop = fix.hdop;
    }

    if (fix.vdop != last_vdop) {

      char dop_bin_char[3];
      char tenths_bin_char[2];
      strcpy(tenths_bin_char, "");
      strcpy(dop_bin_char, "");
      dop_bin = fix.vdop / 1000;
      itoa(dop_bin, dop_bin_char, 10);
      tenths_bin = (fix.vdop - (dop_bin * 1000)) / 100;
      itoa(tenths_bin, tenths_bin_char, 10);

      strcpy(v_dop, "");
      strcat(v_dop, dop_bin_char);
      strcat(v_dop, ".");
      strcat(v_dop, tenths_bin_char);

      something_changed = true;
      last_vdop = fix.vdop;
    }

    if (something_changed) {

      strcpy(all_dops, "");
      strcat(all_dops, p_dop);
      strcat(all_dops, "/");
      strcat(all_dops, h_dop);
      strcat(all_dops, "/");
      strcat(all_dops, v_dop);
      strcat(all_dops, "      ");
    }

    if (fix.satellites != last_num_sats) {
      char sat_bin_char[3] = "";

      itoa(fix.satellites, sat_bin_char, 10);

      strcpy(sat_string, "");
      strcat(sat_string, sat_bin_char);
      strcat(sat_string, "/12 satellites.   ");

      last_num_sats = fix.satellites;

      something_changed = true;
    }

    if (something_changed) {
      lcd_print("Fix from            ", sat_string, "PDOP/HDOP/VDOP:     ", all_dops);
    }
  }

  else if (menu_number == 6) {
    if (new_astro_data) {
      lcd.setCursor ( 0, 1 );
      lcd.print(illum);
      lcd.setCursor ( 0, 3);
      lcd.print(phase_name);
      new_astro_data = false;
    }
  }

  else if (menu_number == 7) {
    if (new_astro_data) {
      lcd.setCursor ( 0, 1 );
      lcd.print(sunrise);
      lcd.setCursor ( 0, 3);
      lcd.print(sunset);
      new_astro_data = false;
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // Deal with alternate count rate, if needed.
  //
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (primary_display_mode == 2) {

    uint64_t elapsed_seconds = abs(t_unix - alt_count_start_time);

    if (elapsed_seconds > 1) {
      alt_count = (1000 - alt_count_start_ms) + t_millis + (elapsed_seconds - 1) * 1000;
    }

    else if (elapsed_seconds == 0) {
      alt_count = t_millis - alt_count_start_ms;
    }

    else if (elapsed_seconds == 1) {
      alt_count = (1000 - alt_count_start_ms) + t_millis;
    }

    else {
      alt_count = 0;
    }

    alt_count = round(alt_count / alt_rate_divisor);

    display_number(alt_count, maxBrightness);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //  LIGHT SHOW
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  else if ((primary_display_mode == 5) && (current_millis != last_LS_millis)) {

    const int16_t xCoords[81] =
    { 3250, 3175, 3265, 3075, 3160, 3225, 3125, 3040, 2950, 2875, 2965, 2775, 2860, 2925, 2825, 2740,
      2650, 2575, 2665, 2475, 2560, 2625, 2525, 2440, 2350, 2275, 2365, 2175, 2260, 2325, 2225, 2140,
      2050, 1975, 2065, 1875, 1960, 2025, 1925, 1840, 1750, 1675, 1765, 1575, 1660, 1725, 1625, 1540,
      1450, 1375, 1465, 1275, 1360, 1425, 1325, 1240, 1150, 1075, 1165, 975, 1060, 1125, 1025, 940, 850,
      775, 865, 675, 760, 825, 725, 640, 550, 475, 565, 375, 460, 525, 425, 340
    };

    const int16_t yCoords[81] = {
      610, 685, 350, 610, 520, 425, 350, 425, 610, 685, 350, 610, 520, 425, 350, 425, 610, 685, 350, 610, 520,
      425, 350, 425, 610, 685, 350, 610, 520, 425, 350, 425, 610, 685, 350, 610, 520, 425, 350, 425, 610, 685,
      350, 610, 520, 425, 350, 425, 610, 685, 350, 610, 520, 425, 350, 425, 610, 685, 350, 610, 520, 425,
      350, 425, 610, 685, 350, 610, 520, 425, 350, 425, 610, 685, 350, 610, 520, 425, 350, 425
    };

    const int16_t maxX = 3360;
    const int16_t maxY = 740;

    const float minRate = 1;//rate units are coordinates per ms
    const float maxRate = 2;

    const uint16_t minScrollWidth = 300;
    const uint16_t maxScrollWidth = 900;

    const uint8_t minSlopeFraction = 5;
    const uint8_t maxSlopeFraction = 20;

    static float y;
    static float initalY;

    static float x;
    static float initalX;

    static uint8_t fadeSlopeFraction;
    static float fadeSlope;

    if (!animation_in_progress) { //if no animation is in progress, decide what animation to do

      if (waiting) {
        if (current_millis >= end_light_show_buffer_time) {
          waiting = false;
        }
      }

      if (!waiting) {
        randomSeed(micros());

        animation = random(11) + 1;

        if (num_animations <= 0) {
          rate = random(minRate, (maxRate) ) + ( random(10) * 0.1) + ( random(11) * 0.01);
          num_animations = random(6, 11);

          scrollWidth = random(minScrollWidth, maxScrollWidth) + ( random(10) * 0.1) + ( random(11) * 0.01) ;

          fadeSlopeFraction = random(minSlopeFraction, maxSlopeFraction) + 1; //how quickly does the brigntess drop off from the edge of the sweep line?
          fadeSlope = (-1.0) * ((float)maxBrightness / ((float)maxX / (float)fadeSlopeFraction) );
        }

        else {
          num_animations--;
        }


        if (!invertOutputs) {

          for (uint8_t i = 0; i < 81; i++)
          {
            previousBrightness[i] = 1;
          }


          for (uint8_t i = 0; i < 81; i++)
          {
            assignBV(i, 0);
          }
          applyBV();
        }

        else {

          for (uint8_t i = 0; i < 81; i++)
          {
            previousBrightness[i] = (maxBrightness - 1);
          }

          for (uint8_t i = 0; i < 81; i++)
          {
            assignBV(i, maxBrightness);
          }
          applyBV();
        }

        animationStartTime = current_millis;
        animation_in_progress = true;
        animation_initialized = false;
      }
    }

    if (animation_in_progress) {

      deltaT = abs(current_millis - animationStartTime);

      switch (animation) {

        case 1: //sweep up
          {
            if (!animation_initialized) {
              y = 0;
              animation_initialized = true;
            }

            y = rate * (float)deltaT; //calculate new position of the imaginary line

            if (y > maxY)
            {
              animation_in_progress = false;
              invertOutputs = !invertOutputs;
              end_light_show_buffer_time = current_millis + 500;
              waiting = true;
            }

            else {
              //for each light, check to see if its y-coordiate is under the imaginary line
              for (uint8_t j = 0; j < 81 ; j++)
              {
                if (yCoords[j] < y)
                {
                  assignBV(j, maxBrightness);
                }

                else
                {
                  int16_t z = round(fadeSlope * abs(y - yCoords[j]) ) + maxBrightness;
                  assignBV(j, z);
                }
              }//end zone-check for-loop

              applyBV();
            }
          }
          break;

        case 2: //sweep down
          {
            static float y;

            if (!animation_initialized) {
              y = maxY;
              animation_initialized = true;
            }

            y = maxY - (rate * (float)deltaT); //calculate new position of the imaginary line

            if (y < 0)
            {
              animation_in_progress = false;
              invertOutputs = !invertOutputs;
              end_light_show_buffer_time = current_millis + 500;
              waiting = true;
            }

            else {
              for (uint8_t j = 0; j < 81 ; j++)
              {
                if (yCoords[j] > y)
                {
                  assignBV(j, maxBrightness);
                }

                else
                {
                  int16_t z = round(fadeSlope * abs(y - yCoords[j]) ) + maxBrightness;
                  assignBV(j, z);
                }
              }//end zone-check for-loop

              applyBV();
            }
          }
          break;

        case 3: //sweep right
          {

            if (!animation_initialized) {
              x = 0;
              animation_initialized = true;
            }

            x = rate * (float)deltaT; //calculate new position of the imaginary line

            if (x > maxX)
            {
              animation_in_progress = false;
              invertOutputs = !invertOutputs;
              end_light_show_buffer_time = current_millis + 500;
              waiting = true;
            }

            else {
              for (uint8_t i = 0; i < 81 ; i++)
              {
                if (xCoords[i] < x)
                {
                  assignBV(i, maxBrightness);
                }

                else
                {
                  int16_t z = round(fadeSlope * abs(x - xCoords[i]) ) + maxBrightness;
                  assignBV(i, z);
                }
              }//end zone-check for-loop

              applyBV();
            }
          }
          break;

        case 4: //sweep left
          {
            if (!animation_initialized) {
              x = maxX;
              animation_initialized = true;
            }

            x = maxX - (rate * (float)deltaT);//calculate new position of the imaginary line

            if (x < 0)
            {
              animation_in_progress = false;
              invertOutputs = !invertOutputs;
              end_light_show_buffer_time = current_millis + 500;
              waiting = true;
            }

            else {
              for (uint8_t i = 0; i < 81 ; i++)
              {
                if (xCoords[i] > x)
                {
                  assignBV(i, maxBrightness);
                }

                else
                {
                  int16_t z = round(fadeSlope * abs(x - xCoords[i]) ) + maxBrightness;
                  assignBV(i, z);
                }
              }//end zone-check for-loop

              applyBV();

            }
          }
          break;

        ////////////////////////////////

        case 5: //scroll up
          {
            if (!animation_initialized) {
              initalY = y;
              animation_initialized = true;
            }

            y = initalY + (rate * (float)deltaT); //calculate new position of the imaginary line

            if (y > (maxY + (scrollWidth + ((float)maxX / fadeSlopeFraction))))
            {
              animation_in_progress = false;
              end_light_show_buffer_time = current_millis + 500;
              waiting = true;
            }

            else {

              for (uint8_t j = 0; j < 81 ; j++)
              {
                if (yCoords[j] < (y + scrollWidth) && yCoords[j] > (y - scrollWidth))
                {
                  assignBV(j, maxBrightness);
                }
                else
                {
                  int16_t z;

                  if (yCoords[j] < y)
                  {
                    z = round(fadeSlope * abs((y - scrollWidth) - yCoords[j])) + maxBrightness;
                  }

                  else
                  {
                    z = round(fadeSlope * abs((y + scrollWidth) - yCoords[j])) + maxBrightness;
                  }

                  assignBV(j, z);
                }
              }
              applyBV();
            }
          }
          break;

        case 6: //scroll down
          {
            if (!animation_initialized) {
              y = maxY + (scrollWidth + (maxX / fadeSlopeFraction));
              initalY = y;
              animation_initialized = true;
            }

            y = initalY - (rate * (float) deltaT);

            if (y > (0 - (scrollWidth + (maxX / fadeSlopeFraction))))
            {
              animation_in_progress = false;
              end_light_show_buffer_time = current_millis + 500;
              waiting = true;
            }

            else {

              for (uint8_t j = 0; j < 81 ; j++) {
                if (yCoords[j] < (y + scrollWidth) && yCoords[j] > (y - scrollWidth)) {
                  assignBV(j, maxBrightness);
                }
                else {
                  int32_t z;

                  if (yCoords[j] < y) {
                    z = round(fadeSlope * abs((y - scrollWidth) - yCoords[j])) + maxBrightness;
                  }

                  else {
                    z = round(fadeSlope * abs((y + scrollWidth) - yCoords[j])) + maxBrightness;
                  }
                  assignBV(j, z);
                }
              }//end zone-check for-loop

              applyBV();
            }
          }
          break;

        case 7: //scroll right
          {
            if (!animation_initialized) {
              x = 0 - (scrollWidth + (maxX / fadeSlopeFraction));
              initalX = x;
              animation_initialized = true;
            }

            x = initalX + (rate * (float)deltaT);

            if (x > (maxX + (scrollWidth + (maxX / fadeSlopeFraction)))) {
              animation_in_progress = false;
              end_light_show_buffer_time = current_millis + 500;
              waiting = true;
            }

            else {
              for (uint8_t i = 0; i < 81 ; i++) {
                if ((xCoords[i] < (x + scrollWidth)) && (xCoords[i] > (x - scrollWidth))) {
                  assignBV(i, maxBrightness);
                }

                else {
                  int16_t z;

                  if (xCoords[i] < x) {
                    z = round(fadeSlope * abs((x - scrollWidth) - xCoords[i])) + maxBrightness;
                  }

                  else {
                    z = round(fadeSlope * abs((x + scrollWidth) - xCoords[i])) + maxBrightness;
                  }

                  assignBV(i, z);
                }
              }//end zone-check for-loop

              applyBV();
            }
          }
          break;

        case 8: //scroll left
          {
            if (!animation_initialized) {
              x = maxX + (scrollWidth + (maxX / fadeSlopeFraction));
              initalX = x;
              animation_initialized = true;
            }

            x = initalX - (rate * (float)deltaT);

            if (x > (0 - (scrollWidth + (maxX / fadeSlopeFraction))))
            {
              animation_in_progress = false;
              end_light_show_buffer_time = current_millis + 500;
              waiting = true;
            }

            else {
              for (uint8_t i = 0; i < 81 ; i++)
              {
                if (xCoords[i] > x)
                {
                  assignBV(i, maxBrightness);
                }

                else
                {
                  for (uint8_t i = 0; i < 81 ; i++)
                  {
                    if (xCoords[i] < (x + scrollWidth) && xCoords[i] > (x - scrollWidth))
                    {
                      assignBV(i, maxBrightness);
                    }

                    else
                    {
                      int16_t z;

                      if (xCoords[i] < x)
                      {
                        z = round(fadeSlope * abs((x - scrollWidth) - xCoords[i])) + maxBrightness;
                      }

                      else
                      {
                        z = round(fadeSlope * abs((x + scrollWidth) - xCoords[i])) + maxBrightness;
                      }
                      assignBV(i, z);
                    }
                  }//end zone-check for-loop
                }
              }//end zone-check for-loop
              applyBV();
            }
          }
          break;

        case 9: //flying blob animation. Based on code by Lars Christensen
          //https://gist.github.com/larsch/90173bdc9cfa11518ff175151675f275
          {
            const double ext = 128.0;
            const double midx = maxX / 2;
            const double midy = maxY / 2;

            double pfx = midx + (midx + ext) * (0.3 * sin1(current_millis, 1000) + 0.7 * sin1(deltaT, 2623)); // sin(t * 2 * M_PI);
            double pfy = midy + (midy + ext) * (0.3 * sin1(current_millis, 759) + 0.7 * sin1(deltaT, 1728)); // cos(t2 * 2 * M_PI);

            double px = pfx;
            double py = pfy;
            int radius = 18 + 4.0 * sin1(round(current_millis / 3), 2873);

            boolean all_off = true;

            for (uint8_t i = 0; i < 81 ; i++) {
              double dx = xCoords[i] - px;
              double dy = yCoords[i] - py;
              double dist = sqrt(dx * dx + dy * dy);
              int j = radius - dist / 32;
              if (j < 4) {
                assignBV(i, 0);
              }
              else {
                assignBV(i, 255);
                all_off = false;
              }
            }

            if (deltaT > 4000 && all_off)
            {
              animation_in_progress = false;
              end_light_show_buffer_time = current_millis + 500;
              waiting = true;
            }

            applyBV();
          }
          break;

        case 10: //animateRotatingBar by by Lars Christensen
          {
            double midx = maxX / 2;
            const double midy = maxY / 2;
            double ext = 128.0;

            double x1 = midx + midx * sin1((current_millis * 0.5), 1672);

            double y1 = midy + 100.0 * sin1((current_millis * 0.5), 934);
            double ta = (current_millis % 2873) / 2873.0;
            double tb = (current_millis % 3273) / 3273.0;
            double angle = 0.673 * M_PI * sin(ta * 2 * M_PI) + 0.842 * M_PI * sin(tb * 2 * M_PI);

            double x2 = x1 + ext * cos(angle);
            double y2 = y1 + ext * sin(angle);
            double dy = (y2 - y1);
            double dx = (x2 - x1);
            double d3 = pow(dx, 2) + pow(dy, 2);

            boolean all_off = true;

            for (uint8_t i = 0; i < 81 ; i++) {
              double d1 = dx * (y1 - yCoords[i]) - (x1 - xCoords[i]) * dy;
              double dsq = (d1 * d1) / d3;
              long dist = sqrt(dsq);
              long h = 8 - dist / 32;

              if (h < 4) {
                assignBV(i, 0);
              }
              else {
                assignBV(i, 255);
                all_off = false;
              }
            }

            if (deltaT > 4000 && all_off)
            {
              animation_in_progress = false;
              end_light_show_buffer_time = current_millis + 500;
              waiting = true;
            }

            applyBV();
          }
          break;

        case 11:
          {
            const uint8_t loop_sequence[24] = {
              73, 65, 57, 49, 41, 33, 25, 17, 9, 1, 0, 5, 6, 14, 22, 30, 38, 46, 54, 62,
              70, 78, 79, 75
            };

            const uint16_t loop_positions[24] = {
              0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300,
              1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300
            };

            static uint8_t times_around;
            static uint16_t loop_rate;
            static boolean CW;

            if (!animation_initialized) {

              times_around = 0;
              loop_rate = rate * 46;
              CW = random(2);

              if (CW)
                x = 0;

              else
                x = 2400;

              invertOutputs = false;

              for (uint8_t i = 0; i < 81; i++)
              {
                previousBrightness[i] = 1;
              }

              for (uint8_t i = 0; i < 81; i++)
              {
                assignBV(i, 0);
              }
              applyBV();

              animation_initialized = true;
            }

            //x = 1192.9119288 * (tanh(((((float)deltaT / 125.0) - 2.0))) + tanh(2.0));

            if (CW)
              x = (float)deltaT;

            else
              x = 2400.0 - (float)deltaT;

            if ((deltaT > 100) && ((CW && x >= 2400) || (!CW && x <= 0))) {
              animationStartTime = current_millis;
              times_around++;
              if (times_around > 3) {
                animation_in_progress = false;
                end_light_show_buffer_time = current_millis + 500;
                waiting = true;
              }
            }

            else {
              for (uint8_t j = 0; j < 24 ; j++) {
                int16_t z = round((fadeSlope * 1.5) * abs(x - (float)loop_positions[j])
                                  + (float)maxBrightness);
                assignBV(loop_sequence[j], z);
              }

              if (x > 1200) {
                for (uint8_t j = 0; j < 10 ; j++) {

                  if (brightness[j] == 0) {
                    int16_t z = round((fadeSlope * 1.5) * abs((x - 2400.0) -
                                      (float)loop_positions[j])  + (float)maxBrightness);
                    assignBV(loop_sequence[j], z);
                  }
                }
              }
              else {
                for (uint8_t j = 14; j < 24 ; j++) {
                  if (brightness[j] == 0) {
                    int16_t z = round((fadeSlope * 1.5) * abs((x + 2400.0) -
                                      (float)loop_positions[j])  + (float)maxBrightness);
                    assignBV(loop_sequence[j], z);
                  }
                }
              }
              applyBV();
            }

            break;
          }
      }
      last_LS_millis = current_millis;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // Check to see if the rotary encoder has been touched
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  static uint32_t encoder_timeout;
  static uint32_t turning_expires_time;

  boolean button = !digitalRead(8);

  if (button && !previous_button)
  {
    clicked = true;
    if (!menu_mode) {
      menu_number = num_defaults + 1;
    }

    else {
      encoder_t1 = current_millis;
    }
    encoder_timeout = current_millis + 5000;
  }

  else {
    clicked = false;
  }

  previous_button = button;

  internal_encoder_val = myEnc.read();

  //DEBUG_PORT.println(internal_encoder_val);

  if (internal_encoder_val != previous_encoder_val) {

    encoder_timeout = current_millis + 5000;

    if (!menu_mode) {
      menu_number = num_defaults + 1;
    }

    menu_mode = true;

    turning = true;
    turning_expires_time = current_millis + 100;


    if (internal_encoder_val % 4 == 0) {

      encoder_t3 = encoder_t2;
      encoder_t2 = encoder_t1;
      encoder_t1 = current_millis;

      if (internal_encoder_val < previous_encoder_val) {
        encoder_pos--;

        if ( ( encoder_t1 - encoder_t3) < 500) {
          fast_turn_count++;
          fast_turn_minus = true;
          fast_turn_plus = false;

          if ( ( encoder_t1 - encoder_t3) < 200  && fast_turn_count >= 4) {
            very_fast_turn = true;
          }
          else {
            very_fast_turn = false;
          }
        }

        else {
          fast_turn_count = 0;
          fast_turn_minus = false;
          fast_turn_plus = false;
          very_fast_turn = false;
        }

      }

      else {
        encoder_pos++;

        if ( ( encoder_t1 - encoder_t3) < 500) {
          fast_turn_count++;
          fast_turn_plus = true;
          fast_turn_minus = false;
          if ( ( encoder_t1 - encoder_t3) < 200 && fast_turn_count >= 4) {
            very_fast_turn = true;
          }
          else {
            very_fast_turn = false;
          }
        }

        else {
          fast_turn_count = 0;
          fast_turn_plus = false;
          fast_turn_minus = false;
          very_fast_turn = false;
        }
      }

      previous_encoder_val = internal_encoder_val;
    }

    if (previous_encoder_pos != encoder_pos) {

      if (abs(previous_encoder_pos - encoder_pos) != 1)
      {
        if (encoder_pos < previous_encoder_pos) {
          encoder_pos = previous_encoder_pos - 1;
        }
        else {
          encoder_pos = previous_encoder_pos + 1;
        }
        myEnc.write(0);
        internal_encoder_val = previous_encoder_val = 0;
      }

      previous_encoder_pos = encoder_pos ;
    }
  }

  if (turning) {
    if (current_millis >= turning_expires_time) {
      turning = false;
      myEnc.write(0);
      internal_encoder_val = previous_encoder_val = 0;
    }
  }

  if (current_millis >= encoder_timeout) {
    if (primary_display_mode == 8) {

      if (last_primary_display_mode == 8) {
        last_primary_display_mode = 1;
      }

      primary_display_mode = last_primary_display_mode;
      fast_turn_count = 0;
      fast_turn_minus = false;
      fast_turn_plus = false;
      very_fast_turn = false;
    }
    menu_mode = false;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //  !!! BEGIN LCD SCREEN LOGIC !!!
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //  !!! Encoder has not been touched recenly. Defualt mode !!!
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if  (!menu_mode) { //encoder has not been touched recenly. Defualt mode

    if (current_millis >= next_screen_update) {

      clear_lcd();

      menu_number++;

      if  (menu_number > num_defaults)
        menu_number = 1;

      switch (menu_number) {

        case 1: //next million
          {
            if (primary_display_mode == 1) { //main count

              if (main_count >= 1000000000)
                next_screen_update = current_millis - 1;

              else {

                if (t_unix >= main_next_million)
                  main_next_million_Strings_updated = false;

                if (main_next_million_Strings_updated == false) {

                  uint16_t i = 0;

                  do {
                    i++;
                    main_next_million = unix_clock_epoch + (1000000 * (int64_t)i);
                    current_millis = millis();
                  } while (main_next_million < t_unix && current_millis < timeout);
                  main_nth_million = i;

                  char nth_million_char[2];

                  itoa(main_nth_million, nth_million_char, 10);

                  strcpy(main_next_N_million, "");
                  strcat(main_next_N_million, nth_million_char);
                  strcat(main_next_N_million, " million on:      ");

                  unix_time_to_strings(main_next_million);

                  strcpy(main_next_million_date, "");
                  strcpy(main_next_million_time, "");
                  strcpy(main_next_million_date, display_date);
                  strcpy(main_next_million_time, display_time);

                  main_next_million_Strings_updated = true;
                }

                lcd_print("     Main Count     ", main_next_N_million, main_next_million_date,
                          main_next_million_time);

                next_screen_update = current_millis + 10000;
              }
            }

            else if (primary_display_mode == 2) { //alt count

              if (t_unix >= alt_next_million) {
                alt_next_million_Strings_updated = false;
              }

              if (alt_next_million_Strings_updated == false) {

                uint16_t i = 0;

                do {
                  i++;
                  alt_next_million = alt_count_start_time + ((1000000 * (int64_t)i) / (int64_t)alt_rate);
                  current_millis = millis();
                } while (alt_next_million < t_unix && current_millis < timeout);

                alt_nth_million = i;

                char nth_million_char[2];

                itoa(alt_nth_million, nth_million_char, 10);

                strcpy(alt_next_N_million, "");
                strcat(alt_next_N_million, nth_million_char);
                strcat(alt_next_N_million, " million on:      ");

                unix_time_to_strings(alt_next_million);

                strcpy(alt_next_million_date, "");
                strcpy(alt_next_million_time, "");
                strcpy(alt_next_million_date, display_date);
                strcpy(alt_next_million_time, display_time);


                alt_next_million_Strings_updated = true;
              }

              lcd_print(alt_counts_string, alt_next_N_million, alt_next_million_date, alt_next_million_time);

              next_screen_update = current_millis + 10000;
            }

            else {
              next_screen_update = current_millis - 1;
            }
          }
          break;

        case 2:
          {
            if (primary_display_mode == 1) { //main count

              if (t_unix >= main_next_billion) {
                main_next_billion_Strings_updated = false;
              }

              if (main_next_billion_Strings_updated == false) {

                uint16_t i = 0;
                do {
                  i++;
                  main_next_billion = unix_clock_epoch + (1000000000 * (int64_t)i);
                  current_millis = millis();
                } while (main_next_billion < t_unix && current_millis < timeout);
                main_nth_billion = i;

                uint32_t seconds_remaining = main_next_billion - main_count;
                uint32_t expected_leap_seconds = round(((float)seconds_remaining * 27) / 1451606400);
                main_next_billion -= expected_leap_seconds;

                char nth_billion_char[2];

                itoa(main_nth_billion, nth_billion_char, 10);

                strcpy(main_next_N_billion, "");
                strcat(main_next_N_billion, "reach ");
                strcat(main_next_N_billion, nth_billion_char);
                strcat(main_next_N_billion, " billion on: ");

                unix_time_to_strings(main_next_billion);

                strcpy(main_next_billion_date, "");
                strcpy(main_next_billion_time, "");
                strcpy(main_next_billion_date, display_date);
                strcpy(main_next_billion_time, display_time);

                primary_display_mode = 1;
                main_next_billion_Strings_updated = true;
              }
              lcd_print("Main count will     ", main_next_N_billion, main_next_billion_date, main_next_billion_time);
              next_screen_update = current_millis + 10000;
            }

            else if (primary_display_mode == 2) { //alt count

              if (t_unix >= alt_next_billion) {
                alt_next_billion_Strings_updated = false;
              }

              if (alt_next_billion_Strings_updated == false) {

                uint16_t i = 0;

                do {
                  i++;
                  alt_next_billion = alt_count_start_time + ((1000000000 * (int64_t)i) / (int64_t)alt_rate);
                  current_millis = millis();
                } while (alt_next_billion < t_unix && current_millis < timeout);
                alt_nth_billion = i;

                char nth_billion_char[2];

                itoa(alt_nth_billion, nth_billion_char, 10);

                strcpy(alt_next_N_billion, "");
                strcat(alt_next_N_billion, nth_billion_char);
                strcat(alt_next_N_billion, " billion on:      ");

                unix_time_to_strings(alt_next_billion);

                strcpy(alt_next_billion_date, "");
                strcpy(alt_next_billion_time, "");
                strcpy(alt_next_billion_date, display_date);
                strcpy(alt_next_billion_time, display_time);

                alt_next_billion_Strings_updated = true;
              }
              lcd_print(alt_counts_string, alt_next_N_billion, alt_next_billion_date, alt_next_billion_time);
              next_screen_update = current_millis + 10000;
            }

            else if (primary_display_mode == 4) { //unix time

              if (!unix_end_Strings_updated) {

                unix_time_to_strings(9999999999);

                strcpy(unix_end_date, "");
                strcpy(unix_end_time, "");
                strcat(unix_end_date, display_date);
                strcat(unix_end_time, display_time);

                unix_end_Strings_updated = true;
              }

              lcd_print("     UNIX TIME      ", "9999999999 on:      ", unix_end_date, unix_end_time);
              next_screen_update = current_millis + 10000;
            }

            else {
              next_screen_update = current_millis - 1;
            }
          }
          break;

        case 3:
          {
            lcd_print("Current Date/Time:  ", twenty_spaces, current_date, current_time);
            next_screen_update = current_millis + 10000;
          }
          break;

        case 4:
          {
            char dop_bin_char[3];
            char tenths_bin_char[2];
            strcpy(tenths_bin_char, "");
            strcpy(dop_bin_char, "");

            strcpy(tenths_bin_char, "");
            strcpy(dop_bin_char, "");
            dop_bin = fix.pdop / 1000;
            itoa(dop_bin, dop_bin_char, 10);
            tenths_bin = (fix.pdop - (dop_bin * 1000)) / 100;
            itoa(tenths_bin, tenths_bin_char, 10);

            strcpy(p_dop, "");
            strcat(p_dop, dop_bin_char);
            strcat(p_dop, ".");
            strcat(p_dop, tenths_bin_char);
            last_pdop = fix.pdop;

            dop_bin = fix.hdop / 1000;
            itoa(dop_bin, dop_bin_char, 10);
            tenths_bin = (fix.hdop - (dop_bin * 1000)) / 100;
            itoa(tenths_bin, tenths_bin_char, 10);

            strcpy(h_dop, "");
            strcat(h_dop, dop_bin_char);
            strcat(h_dop, ".");
            strcat(h_dop, tenths_bin_char);
            last_hdop = fix.hdop;

            strcpy(tenths_bin_char, "");
            strcpy(dop_bin_char, "");
            dop_bin = fix.vdop / 1000;
            itoa(dop_bin, dop_bin_char, 10);
            tenths_bin = (fix.vdop - (dop_bin * 1000)) / 100;
            itoa(tenths_bin, tenths_bin_char, 10);

            strcpy(v_dop, "");
            strcat(v_dop, dop_bin_char);
            strcat(v_dop, ".");
            strcat(v_dop, tenths_bin_char);
            last_vdop = fix.vdop;

            strcpy(all_dops, "");
            strcat(all_dops, p_dop);
            strcat(all_dops, "/");
            strcat(all_dops, h_dop);
            strcat(all_dops, "/");
            strcat(all_dops, v_dop);
            strcat(all_dops, "      ");

            char sat_bin_char[3];
            strcpy(sat_bin_char, "");
            itoa(fix.satellites, sat_bin_char, 10);

            strcpy(sat_string, "");
            strcat(sat_string, sat_bin_char);
            strcat(sat_string, "/12 satellites.   ");
            last_num_sats = fix.satellites;

            lcd_print("Fix from            ", sat_string, "PDOP/HDOP/VDOP:     ", all_dops);

            next_screen_update = current_millis + 10000;
          }
          break;

        case 5:
          {
            char leap_second_string[21];
            char num_leaps[4];
            itoa(unix_leap, num_leaps, 10);

            strcpy(leap_second_string, "");
            strcat(leap_second_string, num_leaps);
            strcat(leap_second_string, "                ");


            lcd_print("Leap seconds        ", "since Unix epoch:   ", twenty_spaces, leap_second_string);
            next_screen_update = current_millis + 5000;
          }
          break;

        case 6:
          {
            if (have_some_astro_data) {
              lcd_print("        Moon        ", illum, "Phase:              ", phase_name);
              next_screen_update = current_millis + 10000;
              new_astro_data = false;
            }
            else {
              next_screen_update = current_millis - 1;
            }
          }
          break;

        case 7:
          {
            if (have_some_astro_data) {
              lcd_print("Next sunrise:       ", sunrise, "Next sunset:    ", sunset);
              next_screen_update = current_millis + 10000;
              new_astro_data = false;
            }
            else {
              next_screen_update = current_millis - 1 ;
            }
          }
          break;

        default:
          lcd.clear();
          break;
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //  !!! Menu Mode Starts Here (encoder has been touched recently) !!!
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  else { //menu_mode == true

    if (!menu_pushed_to_lcd) {

      clear_lcd();

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //
      //  Menus to be pushed to LCD screen
      //
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      if (menu_number <= num_defaults) {

        menu_number = num_defaults + 1;
      }

      switch (menu_number - num_defaults) {

        case 1: //main menu
          {
            uint8_t j = 1;

            for (uint8_t i = 1; i < 3; i++) {

              if (primary_display_mode == j) {
                j++;
              }

              main_menu_array[i] = j;

              j++;
            }

            j = 1;

            for (uint8_t i = 0; i < 6; i++) {

              if (primary_display_mode == j) {
                j++;
              }

              full_menu_array[i] = j;

              j++;
            }

            lcd.home ();
            lcd.print("     MAIN MENU      ");

            menu_offset = 0;

            print_menu_options();

            last_arrow = 2;
            last_alt_count = alt_rate = encoder_pos = previous_encoder_pos = 1;

            menu_select_arrow();
          }
          break;

        case 2: //adjsut rate screen 1
          {
            lcd_print("Turn knob to        ", "adjust rate.        ", twenty_spaces, "1                   ");
            encoder_pos = previous_encoder_pos = 1;
          }
          break;

        case 3: //adjsut rate screen 2
          {
            lcd_print("Push knob to select ", "counts / second:      ", twenty_spaces, twenty_spaces);

            if (encoder_pos != 1) {

              lcd.setCursor ( 0, 3);

              if (encoder_pos > 1) {
                lcd.print("2");
                display_number(2, maxBrightness);
                last_alt_count = alt_rate = encoder_pos = previous_encoder_pos = 2;
              }
              else {
                lcd.print("1000");
                display_number(1000, maxBrightness);
                last_alt_count = alt_rate = encoder_pos = previous_encoder_pos = 1000;
              }
            }
          }
          break;
      }
      menu_pushed_to_lcd = true;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Menu logic after screens have been pushed
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (menu_pushed_to_lcd) {

      switch (menu_number - num_defaults) {

        case 1: //main menu
          {
            menu_select_arrow();

            if (clicked) {

              switch  (main_menu_array[encoder_pos - 1]) {
                case 1: //main_count
                  {
                    menu_mode = false;
                    primary_display_mode = 1;
                    menu_number = 0;
                    next_screen_update = current_millis - 1;
                  }
                  break;

                case 2: //alt count
                  {
                    menu_mode = true;
                    menu_pushed_to_lcd = false;
                    menu_number = num_defaults + 2;
                    last_alt_count = alt_rate = encoder_pos = previous_encoder_pos = 1;
                    last_primary_display_mode = primary_display_mode;

                    if (last_primary_display_mode == 8) {
                      last_primary_display_mode = 1;
                    }

                    primary_display_mode = 8;
                    display_number(1, maxBrightness);
                  }
                  break;

                case 3: //clock mode
                  {
                    primary_display_mode = 3;
                    menu_mode = false;
                    menu_number = 0;
                    next_screen_update = current_millis - 1;
                  }
                  break;

                case 4: //unix time
                  {
                    primary_display_mode = 4;
                    menu_mode = false;
                    menu_number = 0;
                    next_screen_update = current_millis - 1;
                  }
                  break;

                case 5: //light show
                  {
                    primary_display_mode = 5;
                    waiting = false;
                    animation_in_progress = false;
                    menu_mode = false;
                    menu_number = 0;
                    next_screen_update = current_millis - 1;
                  }
                  break;

                case 6: //New Year Countdown
                  {
                    calculate_next_new_year();
                    primary_display_mode = 6;
                    menu_mode = false;
                    menu_number = 0;
                    next_screen_update = current_millis - 1;
                  }
                  break;

              }
              encoder_t1 = current_millis;
              menu_pushed_to_lcd = false;
            }
          }
          break;

        case 2: //Adjust Count Rate screen 1
          {
            if (encoder_pos != 1) {
              menu_number = num_defaults + 3;
              menu_mode = true;
              menu_pushed_to_lcd = false;
            }
          }
          break;

        case 3: //Adjust Count Rate screen 2
          {
            if (encoder_pos != last_alt_count) {

              uint8_t turn_factor;

              if (very_fast_turn) {
                turn_factor = 100;
              }

              else {
                turn_factor = 10;
              }

              if (fast_turn_plus) {
                while ((encoder_pos  % turn_factor ) != 0 && current_millis < timeout) {
                  current_millis = millis();
                  encoder_pos++;
                }
              }

              else if (fast_turn_minus) {
                while ((encoder_pos  % turn_factor) != 0 && current_millis < timeout) {
                  current_millis = millis();
                  encoder_pos--;
                }
              }

              if (encoder_pos > 1000) {
                encoder_pos = previous_encoder_pos = 1;
              }

              else if (encoder_pos < 1) {
                encoder_pos = previous_encoder_pos = 1000;
              }

              alt_rate = encoder_pos;

              strcpy(counts_per_second, "");

              itoa(alt_rate, counts_per_second, 10);

              char spaces_bin[21] = "";

              if (alt_rate < 10) {
                strcpy(spaces_bin, "                   ");
              }

              else if (alt_rate < 100) {
                strcpy(spaces_bin, "                  ");
              }


              else if (alt_rate < 1000) {
                strcpy(spaces_bin, "                 ");
              }

              else {
                strcpy(spaces_bin, "                ");
              }

              strcat(counts_per_second, spaces_bin);
              lcd.setCursor ( 0, 3);
              lcd.print(counts_per_second);

              display_number(alt_rate, maxBrightness);
              last_alt_count = previous_encoder_pos = encoder_pos;
            }

            if (clicked) {

              strcpy(alt_counts_string, "");
              char count_bin[5] = "";
              itoa(alt_rate, count_bin, 10);
              strcat(alt_counts_string, count_bin);
              strcat(alt_counts_string, " counts / second");

              char spaces_bin[21] = "";

              if (alt_rate < 10) {
                strcpy(spaces_bin, "     ");
              }

              else if (alt_rate < 100) {
                strcpy(spaces_bin, "    ");
              }

              else if (alt_rate < 1000)  {
                strcpy(spaces_bin, "   ");
              }
              else {
                strcpy(spaces_bin, "  ");
              }

              strcat(alt_counts_string, spaces_bin);

              alt_next_million_Strings_updated = false;
              alt_next_billion_Strings_updated = false;

              alt_count = 0;
              alt_count_start_time = t_unix;
              current_millis = millis();
              t_millis = current_millis - last_second_start_millis;

              if (t_millis >= 1000) {

                if (t_millis <= 2000) {
                  t_millis = t_millis - 1000;
                }

                else {
                  t_millis = 0;
                }
                alt_count_start_time++;
              }
              alt_count_start_ms = t_millis;
              alt_rate_divisor = 1000 / (float)alt_rate;
              menu_pushed_to_lcd = false;

              menu_mode = false;
              menu_number = 0;
              primary_display_mode = 2;

            } //if clicked
          }
          break;

      } //switch
    }
  } //else (menu mode == true)
} // void loop()
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  !!!END VOID LOOP!!!
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void unix_time_to_strings(int64_t u_time)
{
  setTime(u_time);
  utc = now();
  local = Central.toLocal(utc, &tcr);

  char month_string[3] = "";
  itoa(month(local), month_string, 10);
  char day_string[3] = "";
  itoa(day(local), day_string, 10);
  char year_string[5] = "";
  itoa(year(local), year_string, 10);

  strcpy(display_date, "");
  strcat(display_date, dayShortStr(weekday(local)));
  strcat(display_date, ", ");
  strcat(display_date, month_string);
  strcat(display_date, "/");
  strcat(display_date, day_string);
  strcat(display_date, "/");
  strcat(display_date, year_string);
  strcat(display_date, "    ");

  uint8_t h = hourFormat12(local);

  char hour_bin[3] = "";
  itoa (h, hour_bin, 10);

  char AM_PM[5];

  strcpy(AM_PM, "");

  if (isAM(local)) {
    strcat(AM_PM, " AM ");
  }
  else {
    strcat(AM_PM, " PM ");
  }

  uint8_t t_minute = minute(local);
  uint8_t t_second = second(local);
  char second_string[3];
  char minute_string[3];

  if (t_minute < 10) {
    char min_bin[2] = "";
    itoa (t_minute, min_bin, 10);
    strcpy(minute_string, "");
    strcat(minute_string, "0");
    strcat(minute_string, min_bin);
  }

  else {
    char min_bin[3] = "";
    itoa (t_minute, min_bin, 10);
    strcpy(minute_string, "");
    strcpy(minute_string, min_bin);
  }

  if (t_second < 10) {
    char sec_bin[2] = "";
    itoa (t_second, sec_bin, 10);
    strcpy(second_string, "");
    strcat(second_string, "0");
    strcat(second_string, sec_bin);
  }

  else {
    char sec_bin[3] = "";
    itoa (t_second, sec_bin, 10);
    strcpy(second_string, "");
    strcpy(second_string, sec_bin);
  }

  strcpy(display_time, "");
  strcat(display_time, hour_bin);
  strcat(display_time, ":");
  strcat(display_time, minute_string);
  strcat(display_time, ":");
  strcat(display_time, second_string);
  strcat(display_time, AM_PM);
  strcat(display_time, tcr -> abbrev);
  strcat(display_time, "    ");

  setTime(t_unix);
  utc = now();
  local = Central.toLocal(utc, &tcr);
}

void calculate_next_new_year() {

  setTime(6, 0, 0, 1, 1, (t_year + 1));

  next_new_year_unix = now();

  setTime(t_unix);
  utc = now();
  local = Central.toLocal(utc, &tcr);
}


void lcd_print(char row0[21], char row1[21], char row2[21], char row3[21]) {
  lcd.home ();
  lcd.print(row0);
  lcd.setCursor ( 0, 1 );
  lcd.print(row1);
  lcd.setCursor (0, 2);
  lcd.print(row2);
  lcd.setCursor ( 0, 3);
  lcd.print(row3);
}

void clear_lcd()
{
  lcd.home ();
  lcd.print(twenty_spaces);
  lcd.setCursor ( 0, 1 );
  lcd.print(twenty_spaces);
  lcd.setCursor (0, 2);
  lcd.print(twenty_spaces);
  lcd.setCursor ( 0, 3);
  lcd.print(twenty_spaces);
}

void menu_select_arrow()
{
  if (encoder_pos != last_arrow)
  {
    if (encoder_pos < 1)
    {
      encoder_pos = previous_encoder_pos = 1;
      if (menu_offset > 0) {
        --menu_offset;
        last_arrow = 0;
        print_menu_options();
      }
    }

    else if (encoder_pos > 2 && menu_offset != 2)
    {
      encoder_pos = previous_encoder_pos = 2;

      if (menu_offset < 2) {
        ++menu_offset;
        last_arrow = 0;
        print_menu_options();
      }
    }

    else if (encoder_pos > 3 && menu_offset == 2)
    {
      encoder_pos = previous_encoder_pos = 3;
    }

    if (last_arrow != 0 || menu_offset == 0) {

      lcd.setCursor ( 0, last_arrow);
      lcd.print(" ");
    }

    if (menu_offset != 0) {
      lcd.setCursor ( 0, 0);
      lcd.print((char)0);
    }

    if (encoder_pos != 3 && menu_offset != 2) {
      lcd.setCursor ( 0, 3);
      lcd.print((char)1);
    }

    lcd.setCursor ( 0, encoder_pos);
    lcd.print((char)126);
    last_arrow = encoder_pos;
  }
}

void print_menu_options() {

  for (uint8_t i = 0; i < 3; i++) {
    main_menu_array[i] = full_menu_array[i + menu_offset];
  }

  for (uint8_t i = 0; i < 3; i++) {

    char to_print[21];
    strcpy(to_print, "");

    switch (main_menu_array[i]) {

      default:
        strcat(to_print, "ERmain_menu_array[x]");
        break;

      case 1:
        strcat(to_print, " Resume Main Count  ");
        break;

      case 2:
        strcat(to_print, " Set Alt. Count Rate");
        break;

      case 3:
        strcat(to_print, " Clock Mode         ");
        break;

      case 4:
        strcat(to_print, " Show Unix Time     ");
        break;

      case 5:
        strcat(to_print, " Light Show         ");
        break;

      case 6:
        strcat(to_print, " New Year Countdown ");
        break;
    }

    lcd.setCursor (0, i + 1);
    lcd.print(to_print);
  }
}

void set_digit(uint8_t digit, uint8_t numeral, uint16_t brightness)
{
  const uint8_t digits[10][8] =
  {
    //{a,   b,  c,  d,  e,  f,  g, dot},
    {1,   0,  5,  6,  7,  3,  4, 2},
    {9,   8, 13, 14, 15, 11, 12, 10},
    {17, 16, 21, 22, 23, 19, 20, 18},
    {25, 24, 29, 30, 31, 27, 28, 26},
    {33, 32, 37, 38, 39, 35, 36, 34},
    {41, 40, 45, 46, 47, 43, 44, 42},
    {49, 48, 53, 54, 55, 51, 52, 50},
    {57, 56, 61, 62, 63, 59, 60, 58},
    {65, 64, 69, 70, 71, 67, 68, 66},
    {73, 72, 77, 78, 79, 75, 76, 74}
    //{a,  b,  c,  d,  e,  f,  g, dot},
  };

  const boolean numerals[37][8] =
  {
    //{a, b, c, d, e, f, g, dot},
    {1, 1, 1, 1, 1, 1, 0, 0}, // 0
    {0, 1, 1, 0, 0, 0, 0, 0}, // 1
    {1, 1, 0, 1, 1, 0, 1, 0}, // 2
    {1, 1, 1, 1, 0, 0, 1, 0}, // 3
    {0, 1, 1, 0, 0, 1, 1, 0}, // 4
    {1, 0, 1, 1, 0, 1, 1, 0}, // 5
    {1, 0, 1, 1, 1, 1, 1, 0}, // 6
    {1, 1, 1, 0, 0, 0, 0, 0}, // 7
    {1, 1, 1, 1, 1, 1, 1, 0}, // 8
    {1, 1, 1, 1, 0, 1, 1, 0}, // 9
    {0, 0, 0, 0, 0, 0, 0, 0}, // 10 All 0ff
    {1, 1, 1, 0, 1, 1, 1, 0}, // 11 A
    {0, 0, 1, 1, 1, 1, 1, 0}, // 12 b
    {1, 0, 0, 1, 1, 1, 0, 0}, // 13 C
    {0, 0, 0, 1, 1, 0, 1, 0}, // 14 c
    {0, 1, 1, 1, 1, 0, 1, 0}, // 15 d
    {1, 0, 0, 1, 1, 1, 1, 0}, // 16 E
    {1, 0, 0, 0, 1, 1, 1, 0}, // 17 F
    {1, 0, 1, 1, 1, 1, 0, 0}, // 18 G
    {0, 1, 1, 0, 1, 1, 1, 0}, // 19 H
    {0, 0, 1, 0, 1, 1, 1, 0}, // 20 h
    {0, 0, 0, 0, 1, 1, 0, 0}, // 21 I
    {0, 1, 1, 1, 1, 0, 0, 0}, // 22 J
    {0, 0, 0, 1, 1, 1, 0, 0}, // 23 L
    {0, 0, 1, 0, 1, 0, 1, 0}, // 24 n
    {1, 1, 1, 1, 1, 1, 0, 0}, // 25 O
    {0, 0, 1, 1, 1, 0, 1, 0}, // 26 o
    {1, 1, 0, 0, 1, 1, 1, 0}, // 27 P
    {1, 1, 1, 0, 0, 1, 1, 0}, // 28 q
    {0, 0, 0, 0, 1, 0, 1, 0}, // 29 r
    {1, 0, 1, 1, 0, 1, 1, 0}, // 30 S
    {0, 0, 0, 1, 1, 1, 1, 0}, // 31 t
    {0, 1, 1, 1, 1, 1, 0, 0}, // 32 U
    {0, 0, 1, 1, 1, 0, 0, 0}, // 33 u
    {0, 1, 1, 1, 0, 1, 1, 0}, // 34 y
    {0, 0, 0, 0, 0, 0, 1, 0}, // 35 -
    {0, 0, 0, 0, 0, 0, 0, 1}  // 36 .
  };

  brightness = pwmBri[brightness];

  for (uint8_t segment = 0; segment < 8; segment++)
  {

    if (numerals[numeral][segment] == 1)
    {
      tlc.setPWM(digits[digit][segment], brightness);
    }

    else
    {
      tlc.setPWM(digits[digit][segment], 0);
    }
  }
}

void display_number(uint64_t number, uint16_t brightness)
{
  uint8_t ten = (number / 1000000000) % 10;
  uint8_t nine = (number / 100000000) % 10;
  uint8_t eight = (number / 10000000) % 10;
  uint8_t seven = (number / 1000000) % 10;
  uint8_t six = (number / 100000) % 10;
  uint8_t five = (number / 10000) % 10;
  uint8_t four = (number / 1000) % 10;
  uint8_t three = (number / 100) % 10;
  uint8_t two = (number / 10) % 10;
  uint8_t one = number % 10;

  //turn off leading zeros
  if (ten == 0) {
    ten = 10;

    if (nine == 0) {
      nine = 10;

      if (eight == 0) {
        eight = 10;

        if (seven == 0) {
          seven = 10;

          if (six == 0) {
            six = 10;

            if (five == 0) {
              five = 10;

              if (four == 0) {
                four = 10;

                if (three == 0) {
                  three = 10;

                  if (two == 0) {
                    two = 10;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  set_digit(0, one, brightness);
  set_digit(1, two, brightness);
  set_digit(2, three, brightness);
  set_digit(3, four, brightness);
  set_digit(4, five, brightness);
  set_digit(5, six, brightness);
  set_digit(6, seven, brightness);
  set_digit(7, eight, brightness);
  set_digit(8, nine, brightness);
  set_digit(9, ten, brightness);

  tlc.write();
}

void display_number_with_zeros(uint64_t number, uint16_t brightness)
{
  uint8_t ten = (number / 1000000000) % 10;
  uint8_t nine = (number / 100000000) % 10;
  uint8_t eight = (number / 10000000) % 10;
  uint8_t seven = (number / 1000000) % 10;
  uint8_t six = (number / 100000) % 10;
  uint8_t five = (number / 10000) % 10;
  uint8_t four = (number / 1000) % 10;
  uint8_t three = (number / 100) % 10;
  uint8_t two = (number / 10) % 10;
  uint8_t one = number % 10;

  set_digit(0, one, brightness);
  set_digit(1, two, brightness);
  set_digit(2, three, brightness);
  set_digit(3, four, brightness);
  set_digit(4, five, brightness);
  set_digit(5, six, brightness);
  set_digit(6, seven, brightness);
  set_digit(7, eight, brightness);
  set_digit(8, nine, brightness);
  set_digit(9, ten, brightness);

  tlc.write();
}



void EEPROM_writeInt(int address, int value)
{
  byte two = (value & 0xFF);
  byte one = ((value >> 8) & 0xFF);

  EEPROM.update(address, two);
  EEPROM.update(address + 1, one);
}

int EEPROMReadInt(int address)
{
  long two = EEPROM.read(address);
  long one = EEPROM.read(address + 1);

  return ((two << 0) & 0xFFFFFF) + ((one << 8) & 0xFFFFFFFF);
}


void assignBV(uint16_t n, int32_t z)
{
  //fix any out-of-bounds z-values
  if (z > maxBrightness)
  {
    z = maxBrightness;
  }
  else if (z < 0)
  {
    z = 0;
  }

  //convert apparent brightness to actual brightness and store it in brightness array
  brightness[n] =  pwmBri[z];
}

//sends the brightness values in the brightness[] array to the LEDs via the shift registers
void applyBV()
{
  for (uint8_t i = 0; i < 81; i++)
  {
    // only push new brightness values to LEDs whose brightness values have changed since the last time applyBV()
    // was called.

    if (brightness[i] != previousBrightness[i])
    {
      if (invertOutputs == true)
      {
        tlc.setPWM(i, (maxPWMBrightness - brightness[i]));
      }

      else
      {
        tlc.setPWM(i, brightness[i]);
      }
      previousBrightness[i] = brightness[i];
    }
  }

  tlc.write();
}

double sin1(long t, long w) {
  return sin((t % w) / (double)w * 2 * M_PI);
}

double tanh(double x)
{
  double x0 = exp(x);
  double x1 = 1.0 / x0;

  return ((x0 - x1) / (x0 + x1));
}
