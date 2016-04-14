/*
 * LAM.TA
 * Copyright (C) 2016 Nicola Corna <nicola@corna.info>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>

#include <Adafruit_MPR121.h>

//Due to a timing bug (see https://github.com/jonblack/arduino-fsm/pull/5)
//old versions (<= 2.1.0) of arduino-fsm don't work correctly
#include <Fsm.h>

#include "RGBFader/RGBFader.h"

const uint8_t red_pin = 10;
const uint8_t green_pin = 11;
const uint8_t blue_pin = 9;

const RGB rgb_pins = { .red = red_pin, .green = green_pin, .blue = blue_pin };

const uint8_t finalBrightnessCycles = 20;

const uint16_t buttonPressedTimeout = 350;
const uint16_t buttonDoubleclickedTimeout = 250;

RGBFader *fader;

//----------------------------------------------

#define FSM_0_TOUCHES 1
#define FSM_1_TOUCH 2
#define FSM_2_TOUCHES 3

//See fsm.svg for the FSM's scheme
void timedout();
void pressed();
void released();
void clicked();
void doubleclicked();
void multipressed();
void multireleased();
void releasedAndMultipressed();

State ss(NULL, NULL);
State s1(NULL, NULL);
State s2(NULL, NULL);
State s3(NULL, NULL);
State s4(NULL, NULL);
State s5(NULL, NULL);
State s6(NULL, NULL);

Fsm fsm(&ss);

Adafruit_MPR121 touchbuttons = Adafruit_MPR121();

bool poweron = true;

uint8_t num_touched(uint16_t touchWord) {
  static const uint8_t onesLookup[] PROGMEM = {
    0,  //0000
    1,  //0001
    1,  //0010
    2,  //0011
    1,  //0100
    2,  //0101
    2,  //0110
    3,  //0111
    1,  //1000
    2,  //1001
    2,  //1010
    3,  //1011
    2,  //1100
    3,  //1101
    3,  //1110
    4,  //1111
  };
  
  return pgm_read_byte(onesLookup + (touchWord & 0x000F));
}

void touch_trigger(uint8_t touches) {

  switch (touches) {
    case 0:
      fsm.trigger(FSM_0_TOUCHES);
      break;

    case 1:
      fsm.trigger(FSM_1_TOUCH);
      break;

    case 2:
    default:  //More than 2 touches are counted as 2
      fsm.trigger(FSM_2_TOUCHES);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  pinMode(blue_pin, OUTPUT);

  Serial.println("Initializing RGBFader object...");
  fader = new RGBFader(rgb_pins, RGBFader::rainbowAndWhite, RGBFader::rainbowAndWhiteSize, 64, 7, 3, 255, true, RGBFader::QUADRATIC);
  fader->setBrightnessCycle(255, 0, finalBrightnessCycles);
  fader->freezeColor = true;
  fader->freezeBrightness = true;

  Serial.println("Creating multitouch's FSM...");
  //See fsm.svg for the FSM's scheme
  fsm.add_transition(&ss, &s1, FSM_1_TOUCH, NULL);
  fsm.add_timed_transition(&s1, &s2, buttonPressedTimeout, &pressed);
  fsm.add_transition(&s2, &ss, FSM_0_TOUCHES, &released);
  fsm.add_transition(&s2, &s6, FSM_2_TOUCHES, &releasedAndMultipressed);
  fsm.add_transition(&s1, &s3, FSM_0_TOUCHES, NULL);
  fsm.add_timed_transition(&s3, &ss, buttonDoubleclickedTimeout, &clicked);
  fsm.add_transition(&s3, &s4, FSM_1_TOUCH, &doubleclicked);
  fsm.add_transition(&s3, &s4, FSM_2_TOUCHES, &doubleclicked);
  fsm.add_transition(&s4, &ss, FSM_0_TOUCHES, NULL);
  fsm.add_transition(&s1, &s6, FSM_2_TOUCHES, &multipressed);
  fsm.add_transition(&ss, &s6, FSM_2_TOUCHES, &multipressed);
  fsm.add_transition(&s5, &s6, FSM_2_TOUCHES, &multipressed);
  fsm.add_transition(&s6, &ss, FSM_0_TOUCHES, &multireleased);
  fsm.add_transition(&s6, &s5, FSM_1_TOUCH, &multireleased);
  fsm.add_transition(&s5, &ss, FSM_0_TOUCHES, NULL);
  
  Serial.println("Initializing touch controller...");
  if (!touchbuttons.begin(0x5A)) {
    Serial.println("Can't find touch controller (MPR121)!!!");
    delay(1000);
  }
  touchbuttons.writeRegister(MPR121_ECR, 0x00);  //Stop the device
  touchbuttons.setThresholds(8, 4);
  touchbuttons.writeRegister(MPR121_DEBOUNCE, 0b01110111);  //Touch debounce (7 touches)
  touchbuttons.writeRegister(MPR121_MHDR, 48);  //Maximum half delta (rising)
  touchbuttons.writeRegister(MPR121_NHDR, 24);  //Noise half delta (rising)
  touchbuttons.writeRegister(MPR121_NCLR, 48);  //Noise count limit (rising)
  touchbuttons.writeRegister(MPR121_FDLR, 192); //Filter delay count limit (rising)
  touchbuttons.writeRegister(MPR121_MHDF, 48);  //Maximum half delta (falling)
  touchbuttons.writeRegister(MPR121_NHDF, 72);  //Noise half delta (falling)
  touchbuttons.writeRegister(MPR121_NCLF, 12);  //Noise count limit (falling)
  touchbuttons.writeRegister(MPR121_FDLF, 192); //Filter delay count limit (falling)
  touchbuttons.writeRegister(MPR121_ECR, 0x9C); //Enable only channels 0~3, 18 samples on the 2nd level filter

  Serial.println("LAM.TA is ready!!");
}

void loop() {
  static uint16_t lastTouches = 0x0000;
  uint16_t currentTouches;
  
  for (uint8_t i = 0; i < 16; i++) {
    fsm.check_timer();

    currentTouches = touchbuttons.touched();
    if (currentTouches != lastTouches) {
      touch_trigger(num_touched(currentTouches));
      lastTouches = currentTouches;
    }

    delay(1);
  }
  
  fader->nextStep();
}

void pressed() {  
  Serial.println("Pressed");

  if (poweron) {
    fader->setBrightnessSpeed(3);
    if (fader->getBrightnessTarget() == 0)
      fader->setBrightnessCycle(255, 0, finalBrightnessCycles);
    else
      fader->setBrightnessCycle(0, 255, finalBrightnessCycles);
  
    fader->freezeBrightness = false;
  }
}

void released() {
  Serial.println("Released");
  if (poweron)
    fader->freezeBrightness = true;
}

void clicked() {
  static uint8_t brightness;

  if (fader->getBrightness() != 0) {
    brightness = fader->getBrightness();
    fader->setBrightnessSpeed(5);
    fader->setBrightness(0);
    poweron = false;
  }
  else {
    fader->setBrightnessSpeed(9);
    fader->setBrightness(brightness);
    poweron = true;
  }
  
  fader->freezeBrightness = false;
  
  Serial.println("Clicked");
}

void doubleclicked() {
  Serial.println("Double clicked");
  //if (poweron) {  //Double-clicking should work also when powered off
    poweron = true;
    fader->goOnColorIndex(7);
    fader->setBrightness(255);
    fader->freezeBrightness = false;
  //}
}

void multipressed() {
  Serial.println("Multi pressed");
  if (poweron)
    fader->freezeColor = false;
}

void multireleased() {
  Serial.println("Multi released");
  fader->freezeColor = true;
}

void releasedAndMultipressed() {
  released();
  multipressed();
}

