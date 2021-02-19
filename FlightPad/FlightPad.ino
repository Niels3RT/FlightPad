/*
    FlightPad by Niels Lueddecke
    based on:
    
    A2600 Paddles/Spinners USB Adapter
    (C) Alexey Melnikov

    Based on project by Mikael Norrgård <mick@daemonbite.com>

    GNU GENERAL PUBLIC LICENSE
    Version 3, 29 June 2007

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

///////////////// Customizable settings /////////////////////////

///////////////// a shortcut ////////////////////////////////////

#define cbi(sfr, bit)     (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit)     (_SFR_BYTE(sfr) |= _BV(bit))

/////////////////////////////////////////////////////////////////

const char *gp_serial = "FlightPad";

#include <EEPROM.h>
#include "Joystick.h"
#include "AuxChannels.h"

volatile uint8_t cnt_adc;
volatile uint8_t cnt_mux;
volatile uint8_t cnt_adc_pos;
volatile uint8_t cnt_mux_pos;
volatile uint16_t adc_val[5][8];
volatile uint16_t mux_val[8][8];
int16_t adc_n[13] = { 0, 0 ,0 ,0, 0, 0, 0 ,0 ,0, 0, 0, 0, 0 };
int16_t adc_aux[8];
bool invert_axis[13] = { false, true, false, false, false, false, false, false, false, false, false, false, false };
uint16_t dead;
bool use_dead[13] = { true, true, true, true, true, false, false, false, false, true, true, true, true };
CalData cal[13];
uint16_t l_count;
bool do_cal;
bool cal_btn_released;
uint8_t btn_shift[5];

uint16_t drvpos;
volatile uint8_t t;
volatile uint8_t t_old;
volatile uint8_t t_old_old;

Joystick_ Joystick;
Aux_ Aux;

// ****** incremental encoder ******
void drv_proc(void) {
  if (t_old != t) {
    if (t == 3) {
      if ((t_old_old == 0) && (t_old == 1)) {
        drvpos++;
      }
      if ((t_old_old == 1) && (t_old == 0)) {
        drvpos--;
      }
    }
    t_old_old = t_old;
    t_old = t;
  }
}

void drv0_isr() {
  t = (digitalRead(0) << 1) + digitalRead(1);
  drv_proc();
}

void drv1_isr() {
  t = (digitalRead(0) << 1) + digitalRead(1);
  drv_proc();
}

// ****** adc conversion complete isr ******
ISR(ADC_vect) {
  // --- fetch sample
  uint8_t samplel = ADCL;
  uint8_t sampleh = ADCH;

  // --- determine channel and sample position
  if (cnt_adc < 5) {
    cnt_adc++;
  } else {
    cnt_adc = 0;
    if (cnt_adc_pos < 7) {
      cnt_adc_pos++;
    } else {
      cnt_adc_pos = 0;
    }
  }

  // --- decide on which adc to use and setup mux
  switch(cnt_adc) {
    case 0:   // read ADC4, joy1 x, start ADC5
      ADMUX = 0x45;     // AVCC with external capacitor on AREF pin
      ADCSRB = 0x00;
      break;
    case 1:   // read ADC5, joy1 y, start ADC6
      ADMUX = 0x46;
      ADCSRB = 0x00;
      break;
    case 2:   // read ADC6, joy2 x, start ADC7
      ADMUX = 0x47;
      ADCSRB = 0x00;
      break;
    case 3:   // read ADC7, joy2 y, start ADC12
      ADMUX = 0x44;
      ADCSRB = 0x20;
      break;
    case 4:   // read ADC12, joy1 z, start ADC13, mux
      ADMUX = 0x45;
      ADCSRB = 0x20;
      // -- handle multiplexer
      if (cnt_mux < 7) {
        cnt_mux++;
      } else {
        cnt_mux = 0;
        cnt_mux_pos++;
        cnt_mux_pos &= 0x07;
      }
      set_mux(cnt_mux);
      break;
    case 5:   // read ADC13, mux, start ADC4
      ADMUX = 0x44;
      ADCSRB = 0x00;
      break;
  }

  // --- invert axis and store sample
  if (cnt_adc < 5) {
    // -- direct axis
    if (invert_axis[cnt_adc]) {
      adc_val[cnt_adc][cnt_adc_pos] = 0x3ff - ((sampleh << 8) + samplel);
    } else {
      adc_val[cnt_adc][cnt_adc_pos] = (sampleh << 8) + samplel;
    }
  } else {
    //mux_val[cnt_mux][0] = (sampleh << 8) + samplel;
    mux_val[cnt_mux][cnt_mux_pos] = 0x3ff - ((sampleh << 8) + samplel);
  }

  //ADCSRA = 0xcf;    // start conversion, /128, 8kHz/sample
  ADCSRA = 0xce;    // start conversion, /64, 16kHz/sample
}

// ****** set analog multiplexer for aux channels ******
void set_mux(uint8_t nr) {
  if (nr & 0x01) sbi (PORTB, PB2); else cbi (PORTB, PB2); // --- A
  if (nr & 0x02) sbi (PORTB, PB3); else cbi (PORTB, PB3); // --- B
  if (nr & 0x04) sbi (PORTB, PB1); else cbi (PORTB, PB1); // --- C
}

// ****** setup µC ******
void setup() {
  // --- init serial for debug output
  Serial.begin(9600);
  
  // --- set all usable pins to input without pullups
  DDRB  &= ~B01111110;
  PORTB |=  B01111110;
  DDRC  &= ~B01000000;
  PORTC |=  B01000000;
  DDRD  &= ~B10011111;
  PORTD |=  B10011111;
  DDRE  &= ~B01000000;
  PORTE |=  B01000000;
  DDRF  &= ~B11110000;
  PORTF |=  B11110000;

  // --- 74HC165 pl/cp
  sbi (DDRD, PD7);  // PL
  cbi (PORTD, PD7);
  sbi (DDRC, PC6);  // CP
  cbi (PORTC, PC6);

  // --- 74HC4051 ABC, multiplexer addressing
  sbi (DDRB, PB2);  // A
  cbi (PORTB, PB2);
  sbi (DDRB, PB3);  // B
  cbi (PORTB, PB3);
  sbi (DDRB, PB1);  // C
  cbi (PORTB, PB1);
  
  // --- analog inputs
  cnt_adc = 0;
  cnt_mux = 0;
  cnt_adc_pos = 0;
  cnt_mux_pos = 0;
  DIDR0 = 0xf0;     // disable ditigal input buffers for usable adc pins
  DIDR2 = 0x30;
  ADMUX = 0x44;     // AVCC with external capacitor on AREF pin
  ADCSRA = 0x87;    // enable adc
  ADCSRA = 0xcf;    // start first conversion

  // --- read calibration data from eeprom
  for (uint8_t c=0;c<13;c++) {
    ee_cal_read(&cal[c], c);
  }
  l_count = 0;
  //dead = 0x100;
  dead = 0x80;

  // --- init HID reports
  Joystick.reset();
  Aux.reset();

  // --- init incremental encoder
  drv_proc();
  drvpos = 0;
  attachInterrupt(digitalPinToInterrupt(1), drv0_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(0), drv0_isr, CHANGE);
}

// ****** main loop ******
JoystickReport joy_rep;
AuxReport aux_rep;
void loop() {
  // --- LEDs off
  TXLED1;
  RXLED1;

  // --- put things to default
  joy_rep.buttons = 0x00000000;

  // --- shift in buttons and switches
  cbi (PORTD, PD7);   // parallel load
  sbi (PORTD, PD7);
  for (uint8_t i=0;i<8;i++) {
    btn_shift[0] <<= 1;
    btn_shift[1] <<= 1;
    btn_shift[2] <<= 1;
    btn_shift[3] <<= 1;
    btn_shift[4] <<= 1;
    if (!(PINE & 0x40)) btn_shift[0]++;   // buttons left
    if (!(PINB & 0x10)) btn_shift[1]++;   // rest of buttons left and bottom
    if (!(PIND & 0x02)) btn_shift[2]++;   // switches 1
    if (!(PIND & 0x01)) btn_shift[3]++;   // switches 2
    if (!(PIND & 0x10)) btn_shift[4]++;   // switches 3
    sbi (PORTC, PC6);   // clock in next bit
    cbi (PORTC, PC6);
  }

  // --- sum up adc sample arrays
  // -- joystick axis
  for (uint8_t c=0;c<5;c++) {
    adc_n[c] = 0;
    for (uint8_t i=0;i<8;i++) {
      adc_n[c] += adc_val[c][i];
    }
  }
  // -- muxed channels
  for (uint8_t c=0;c<8;c++) {
    adc_n[c+5] = 0;
    for (uint8_t i=0;i<8;i++) {
      adc_n[c+5] += mux_val[c][i];
    }
  }

  // --- handle calibration
  if (l_count < 200) {
    l_count++;
    // -- start calibration? Hold button 1 on boot to start calibration.
    if (btn_shift[0] & 0x01) {
      do_cal = true;
      // - set calibration data to something usefull
      for (uint8_t c=0;c<13;c++) {
        cal_to_default(&cal[c]);
      }
    }
  } else {
    if (do_cal) {
      // -- get midpoints if button 2 is pushed
      if (btn_shift[0] & 0x02) {
        for (uint8_t c=0;c<13;c++) {
          cal[c].mid = adc_n[c];
        }
      }
      // -- find min, max, quot, direct axis
      for (uint8_t c=0;c<13;c++) {
        if (adc_n[c] < cal[c].base) cal[c].base = adc_n[c];
        if (adc_n[c] > cal[c].top)  cal[c].top = adc_n[c];
        if (use_dead[c]) {
          cal[c].quot_up = (float)(cal[c].top - cal[c].mid - dead) / 128;
          cal[c].quot_down = (float)(cal[c].mid - cal[c].base - dead) / 128;
        } else {
          cal[c].quot_up = (float)(cal[c].top - cal[c].mid) / 128;
          cal[c].quot_down = (float)(cal[c].mid - cal[c].base) / 128;
        }
      }
    }

    // -- sanity check and normalize
    for (uint8_t c=0;c<13;c++) {
      if (adc_n[c] < cal[c].base) {
        adc_n[c] = cal[c].base;
      } else if (adc_n[c] >= cal[c].top) {
        adc_n[c] = cal[c].top - 1;
      }
      if (use_dead[c]) {
        // - decide if upper or lower half of range, use deadzone
        if (adc_n[c] > cal[c].mid) {
          if (adc_n[c] > (cal[c].mid + dead)) {
            adc_n[c] = ((adc_n[c] - cal[c].mid - dead) / cal[c].quot_up) + 128;
          } else {
            adc_n[c] = 128;
          }
        } else {
          if (adc_n[c] < (cal[c].mid - dead)) {
            adc_n[c] = (adc_n[c] - cal[c].base) / cal[c].quot_down;
          } else {
            adc_n[c] = 128;
          }
        }
      } else {
        // - decide if upper or lower half of range, no deadzone
      if (adc_n[c] > cal[c].mid) {
        adc_n[c] = ((adc_n[c] - cal[c].mid) / cal[c].quot_up) + 128;
      } else {
        adc_n[c] = (adc_n[c] - cal[c].base) / cal[c].quot_down;
      }
      }
    }

    // --- print info to serial when calibrating
    if (do_cal) {
      for (uint8_t c=0;c<5;c++) {
        Serial.print("c:");
        Serial.print(c);
        Serial.print("|");
        Serial.print(cal[c].base, HEX);
        Serial.print("|");
        Serial.print(cal[c].mid, HEX);
        Serial.print("|");
        Serial.print(cal[c].top, HEX);
        Serial.print("|");
        Serial.print(cal[c].quot_up);
        Serial.print("|");
        Serial.print(cal[c].quot_down);
        Serial.print("|");
        Serial.print(adc_n[c]);
        Serial.print("  ");
       // Serial.println();    // End the line
      }
      Serial.println();    // End the line
      
      // -- end calibration?
      if (btn_shift[0] & 0x01) {
        if (cal_btn_released) {
          // - end calibration and save cal data to eeprom
          do_cal = false;
          for (uint8_t c=0;c<13;c++) {
            ee_cal_update(&cal[c], c);
          }
        }
      } else {
        // - set button released flag
        cal_btn_released = true;
      }
    }
  }

  // --- prepare joystick axis data
  int16_t adc_x = -128 + adc_n[0];
  int16_t adc_y = -128 + adc_n[1];
  int16_t adc_z = -128 + adc_n[4];
  int16_t adc_j2x = -128 + adc_n[2];
  int16_t adc_j2y = -128 + adc_n[3];
  for (uint8_t c=0;c<8;c++) {
    adc_aux[c] = -128 + adc_n[c+5];
  }

  // --- named axis, joystick report
  joy_rep.x = adc_x;               // up/down
  joy_rep.y = adc_y;               // left/right
  joy_rep.z = adc_z;               // rudder
  joy_rep.jaux_x = adc_j2x;        // thumbstick up/down
  joy_rep.jaux_y = adc_j2y;        // thumbstick left/right
  joy_rep.throttle = adc_aux[0];   // throttle
  joy_rep.slider0 = adc_aux[1];    // slider

  // --- dial, rotary encoder
  static uint16_t prev = 0;
  int16_t val = ((int16_t)(drvpos - prev));
  if (val > 127) val = 127; else if (val < -127) val = -127;
  joy_rep.dial = val;
  prev += val;

  // --- aux report axis
  aux_rep.slider0 = adc_aux[0];    // slider
  aux_rep.slider1 = adc_aux[1];    // slider
  aux_rep.slider2 = adc_aux[2];    // slider
  aux_rep.slider3 = adc_aux[3];    // slider
  aux_rep.slider4 = adc_aux[4];    // slider
  aux_rep.slider5 = adc_aux[5];    // slider
  aux_rep.slider6 = adc_aux[6];    // slider
  aux_rep.slider7 = adc_aux[7];    // slider

  // --- prepare buttons to send
  joy_rep.buttons = btn_shift[0] << 2;                      // button block
  if (btn_shift[1] & 0x01) joy_rep.buttons += 0x00000400;   // 2 buttons left of block
  if (btn_shift[1] & 0x02) joy_rep.buttons += 0x00000800;
  if (btn_shift[1] & 0x04) joy_rep.buttons += 0x00000001;
  if (btn_shift[1] & 0x08) joy_rep.buttons += 0x00000002;
  if (btn_shift[2] & 0x01) joy_rep.buttons += 0x00001000;   // switches block 1, 2 state
  if (btn_shift[2] & 0x04) joy_rep.buttons += 0x00002000;
  if (btn_shift[2] & 0x10) joy_rep.buttons += 0x00004000;
  if (btn_shift[2] & 0x40) joy_rep.buttons += 0x00008000;
  joy_rep.buttons += (uint32_t)btn_shift[3] << 16;          // switches block 2, 3 state
  if (btn_shift[4] & 0x01) joy_rep.buttons += 0x01000000;   // switches block 3, 2 state
  if (btn_shift[4] & 0x04) joy_rep.buttons += 0x02000000;
  if (btn_shift[4] & 0x10) joy_rep.buttons += 0x04000000;
  if (btn_shift[4] & 0x40) joy_rep.buttons += 0x08000000;

  // --- Only report controller state if it has changed
  if (memcmp(&Joystick._JoystickReport, &joy_rep, sizeof(JoystickReport))) {
    Joystick._JoystickReport = joy_rep;
    Joystick.send();
  }

  // --- Only report controller state if it has changed
  if (memcmp(&Aux._AuxReport, &aux_rep, sizeof(AuxReport))) {
    Aux._AuxReport = aux_rep;
    Aux.send();
  }
}

// ****** calibration data to default start values for calibration ******
void cal_to_default(CalData * cal_ptr) {
  cal_ptr->base = 6000;
  cal_ptr->mid = 4000;
  cal_ptr->top = 2000;
  cal_ptr->quot_up = 25;
  cal_ptr->quot_down = 25;
}

// ****** read calibration data ******
void ee_cal_read(void * cal_ptr, uint8_t chn_nr) {
  uint8_t crc_c = 0x5a;
  uint8_t crc_r = 0x00;
  uint8_t size_cal = sizeof(CalData);
  int adr = (size_cal + 1) * chn_nr;
  for (uint8_t i=0;i<size_cal;i++) {
    *(uint8_t *)cal_ptr = EEPROM.read(adr++);
    crc_c ^= *(uint8_t *)cal_ptr++;
  }
  crc_r = EEPROM.read(adr);
  Serial.print("ee read cal: ");
  Serial.print(chn_nr);
  Serial.print(" crc: ");
  Serial.print(crc_r);
  Serial.print("   ");
  Serial.print(crc_c);
  Serial.println();    // End the line
}

// ****** write calibration data to eeprom ******
void ee_cal_update(void * cal_ptr, uint8_t chn_nr) {
  uint8_t crc_c = 0x5a;
  uint8_t size_cal = sizeof(CalData);
  int adr = (size_cal + 1) * chn_nr;
  for (uint8_t i=0;i<size_cal;i++) {
    EEPROM.update(adr++, *(uint8_t *)cal_ptr);
    crc_c ^= *(uint8_t *)cal_ptr++;
  }
  EEPROM.update(adr, crc_c);
  Serial.print("ee write cal: ");
  Serial.print(chn_nr);
  Serial.print(" crc: ");
  Serial.print(crc_c);
  Serial.println();    // End the line
}
