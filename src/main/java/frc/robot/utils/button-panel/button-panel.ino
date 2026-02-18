#include <Joystick.h>
Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_GAMEPAD, 
  1, // button count
  1, // hat switch count
  true,  // X axis
  false,  // Y axis
  false, // Z axis
  false, // Rx
  false, // Ry
  false, // Rz
  false, // rudder
  false, // throttle
  false, // accelerator
  false, // brake
  false  // steering
);

#define CLK 10
#define DT 11
volatile float pos = 0.0;

float wrap(float pos, float wrap) {
  while (pos > wrap || pos < -wrap) {
    if (pos > wrap) {
      pos -= 2.0 * wrap;
    } else if (pos < -wrap) {
      pos += 2.0 * wrap;
    }
  }
  return pos;
}

void readEncoder() {
  if (digitalRead(DT) != digitalRead(CLK)) {
    pos += 0.01;
  } else {
    pos -= 0.01;
  }
}

void setup() {
  Joystick.begin();

  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLK), readEncoder, FALLING);

  for (int x = 2; x <= 9; x++) {
    pinMode(x, INPUT_PULLUP);
  }
}

void loop() {
  // pos = wrap(pos, 1.0);
  // if (abs(pos) < 0.01) {
  //   pos = 0.0;
  // }
  // Joystick.setXAxis(int((pos + 1.0) * 511.5));

  // int a = -1; // Default to -1 (no direction)
  // for (int x = 2; x <= 9; x++) {
  //   if (digitalRead(x) == LOW) { // Check if the pin is LOW (connected to GND)
  //     a = x - 2; // Map pin 2 to North (0), pin 3 to NE (1), etc.
  //     break; // Stop checking once a pin is found
  //   }
  // }
  // Joystick.setHatSwitch(0, a * 45); // Set the hat switch direction

  if (digitalRead(2)) {
    Joystick.setButton(0, 0);
  } else {
    Joystick.setButton(0, 1);
  }
}