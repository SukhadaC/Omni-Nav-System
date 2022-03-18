#include <PS4BT.h>
#include <usbhub.h>
#include"mpu.h"

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#define r1 6
#define l1 7
#define l2 45
#define r2 44
#define l3 12
#define r3 11
#define l4 10
#define r4 9
#define kp 80
#define kd 10
#define ki 0.0008

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

//                                 ************    Variable   **************
float lx, ly, rx, ry;
float v, v1, v2, v3, v4;
int omega;
float theta = 0, prev_theta = 0;
int x = 0, fg;
int m = 0, n = 0;
float  curr_angle, prev_error, set_angle = 0, error = 0, correction = 0,cumm=0;
//                               ************    Setup    **************
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);

#if !defined(MIPSEL)
  while (!Serial);
#endif
  if (Usb.Init() == -1) {
        Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
    Serial.println(F("\r\nPS4 Bluetooth Library Started"));
}

//                         ****************   Loop  **************
void loop() {

  Usb.Task();
  Serial.println("a");
  //                      ****************  PID    ***********
  curr_angle = mpu(); //getting current angle

  error = curr_angle - set_angle; // error
  cumm+=error;
  correction = kp * error + kd * (error - prev_error) + ki * (cumm); // PID equation
  
 
 
  //     Serial.print("\n\rError: ");
  //     Serial.println(error);


  //                  ***********   Hat mapping    **************
  if (PS4.connected()) {
    lx = map(PS4.getAnalogHat(LeftHatX), 0, 255, -127, 127); //Getting hat values
    ly = map(PS4.getAnalogHat(LeftHatY), 0, 255, 127, -127);

    if ((lx < -10 || lx > 10) || (ly < -10 || ly > 10) || PS4.getAnalogButton(R2) > 0 || PS4.getAnalogButton(L2) > 0) {
      fg = PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2);
      //            Serial.print(F("\r\nLX: "));
      //            Serial.print(lx);
      //            Serial.print(F("\tLY: "));
      //            Serial.print(ly);

      theta = atan2(ly, lx);

      //      if (theta >= 0) {
      //        Serial.print("\r\nTheta:");
      //        Serial.print(theta * 57.2958);
      //      }
      //      else {
      //        Serial.print("\r\nTheta:");
      //        Serial.print(theta * 57.2958 + 360);
      //      }

      //                  ************    Equations with correction added *********
      v = sqrt((pow(lx, 2) + pow(ly, 2)));
      fg = map(fg, -255, 255, -1200, 1200);
      if (fg != 0)
      {
        set_angle = mpu();
      }
      v = map(v, -127, 127, -680, 680);
      if (v > 680) {
        v = 680;
      }
      else if (v < -680) {
        v = -680;
      }
      //      Serial.print("\r\nv:");
      //      Serial.print(v);
      correction = correction / 4;
      v1 = 0.70711 * (-v * cos(theta) + v * sin(theta)) + fg / 4 + correction; //kinematics of equations
      v2 = 0.70711 * (-v * cos(theta) - v * sin(theta)) + fg / 4 + correction;
      v3 = (0.70711 * (v * cos(theta) - v * sin(theta))) + fg / 4 + correction;
      v4 = (0.70711 * (v * cos(theta) + v * sin(theta))) + fg / 4 + correction;

      if (v1 > 0) {
        v1 = map(v1, 0, 680, 0, 100);
        if (v1 > 127)
          v1 = 127;
        analogWrite(r1, abs(v1));
      }
      else {
        v1 = map(-v1, 0, 680, 0, 100);
        if (v1 > 127)
          v1 = 127;
        analogWrite(l1, abs(v1));
        v1 = -v1;

      }

      if (v2 > 0) {
        v2 = map(v2, 0, 680, 0, 100);
        if (v2 > 127)
          v2 = 127;
        analogWrite(r2, abs(v2));
      }
      else {
        v2 = map(-v2, 0, 680, 0, 100);
        if (v2 > 127)
          v2 = 127;
        analogWrite(l2, abs(v2));
        v2 = -v2;

      }


      if (v3 > 0) {
        v3 = map(v3, 0, 680, 0, 100);
        if (v3 > 127)
          v3 = 127;
        analogWrite(r3, abs(v3));
      }
      else {
        v3 = map(-v3, 0, 680, 0, 100);
        if (v3 > 127)
          v3 = 127;
        analogWrite(l3, abs(v3));
        v3 = -v3;


      }
      if (v4 > 0) {
        v4 = map(v4, 0, 680, 0, 100);
        if (v4 > 127)
          v4 = 127;

        analogWrite(r4, abs(v4));
      }
      else {
        v4 = map(-v4, 0, 680, 0, 100);
        if (v4 > 127)
          v4 = 127;
        analogWrite(l4, abs(v4));
        v4 = -v4;


      }
                  Serial.print(F("\r\nv1: "));
                  Serial.print(v1);
                  Serial.print(F("\r\tv2: "));
                  Serial.print(v2);
                  Serial.print(F("\r\tv3: "));
                  Serial.print(v3);
                  Serial.print(F("\r\t v4: "));
                  Serial.print(v4);

    }

    //                *********    Safety Condition for analog Hat    *************
    else {
      lx = 0;
      ly = 0;
      //            Serial.print(F("\r\nLX: "));
      //            Serial.print(lx);
      //            Serial.print(F("\tLY: "));
      //            Serial.print(ly);
      analogWrite(l1, 0);
      analogWrite(l2, 0);
      analogWrite(l3, 0);
      analogWrite(l4, 0);
      analogWrite(r1, 0);
      analogWrite(r2, 0);
      analogWrite(r3, 0);
      analogWrite(r4, 0);
    }

    //            ***********       Clockwise       ************
    //    if (PS4.getAnalogButton(L2)) {
    //      m = map(PS4.getAnalogButton(L2), 0, 255, 0, 80);
    //      analogWrite(r1, m);
    //      analogWrite(r2, m);
    //      analogWrite(r3, m);
    //      analogWrite(r4, m);
    //      //      Serial.print("\r\nAntilockwise :");
    //      //      Serial.print(m);
    //
    //      set_angle = mpu();
    //    }
    //
    //    //            **********       AntiClockwise    ************
    //    else if (PS4.getAnalogButton(R2)) {
    //      n = map(PS4.getAnalogButton(R2), 0, 255, 0, 80);
    //      analogWrite(l1, n);
    //      analogWrite(l2, n);
    //      analogWrite(l3, n);
    //      analogWrite(l4, n);
    //      //      Serial.print("\r\nClockwise :");
    //      //      Serial.print(n);
    //      set_angle = mpu();
    //    }

    //             **********     PS Disconnect       ************
    if (PS4.getButtonClick(PS)) {
      analogWrite(l1, 0);
      analogWrite(l2, 0);
      analogWrite(l3, 0);
      analogWrite(l4, 0);
      analogWrite(r1, 0);
      analogWrite(r2, 0);
      analogWrite(r3, 0);
      analogWrite(r4, 0);
      PS4.disconnect();
    }
  }
}
