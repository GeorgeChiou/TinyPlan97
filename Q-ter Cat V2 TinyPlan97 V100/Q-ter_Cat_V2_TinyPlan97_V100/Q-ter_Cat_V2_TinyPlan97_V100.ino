//////////////////////////////////////////////////////////////////////
// Q-ter Cat V2 (Using MiniPlan TinyPlan97) by Mason E & D (2020.2.7)
/////////////////////////////////////////////////////////////////////

// Author : Mason 2019/7/30, masonchen1003@gmail.com
// FB : https://www.facebook.com/mason.chen.1420
// Licensed under the Creative Commons - Attribution - Non-Commercial license.
// 請尊重智慧財產權，如要使用於他處或應用，請註明出處及通知原作者。

//   FL  --- HEAD --- FR
//   ---              ---
//   BL  --- TAIL --- BR
// Servo : Suggest MG90S

#include "Arduino.h"
#include "Wire.h"

#include <Servo.h>
#include <LRemote.h>
#include <EEPROM.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////
// servo offset, please update the offset values
//                           P2, P5,P13,P10, P4, P7,P12,P11,P17,P16,P3,P15
int offset_12x[] PROGMEM = {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // initial all 0

// your servo offset values
//                           P2, P5, P13, P10, P4, P7, P12, P11, P17, P16, P3, P15
//int offset_12x[] PROGMEM = {  0,  0,   0,   0,  0,  0,   0,   0,   0,   0,  0,   0};
//////////////////////////////////////////////////////////////////////////////////////////////////////


// offset value for EEPROM, debug on going
int offset[] PROGMEM = {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // default

// define linkit remote variable
LRemoteCircleButton home_btn;
LRemoteButton fwd_btn;
LRemoteButton bwd_btn;
LRemoteButton left_btn;
LRemoteButton right_btn;
LRemoteButton demo1_btn;
LRemoteButton demo2_btn;
LRemoteButton demo3_btn;
LRemoteButton demo4_btn;
LRemoteButton act1_btn;
LRemoteButton act2_btn;
LRemoteButton act3_btn;
LRemoteButton act4_btn;
LRemoteButton act5_btn;
LRemoteButton act6_btn;
LRemoteButton act7_btn;
LRemoteButton act8_btn;
LRemoteButton act9_btn;
LRemoteButton act10_btn;
LRemoteButton act11_btn;
LRemoteButton act12_btn;
LRemoteSwitch HL_btn;

// Version
String FW_Version = "Quadruped Robot V101 (2020/01/09)";

// Battery
#define BatteryPin  14
float BatteryVol = 0;
int BatteryCount = 0;
const float BatteryLowVol = 4.6;
const float R1 = 100;     // 10Kohm, The Voltage devider resistor R1
const float R2 = 47;      // 4.7Kohm, The Voltage devider resistor R2
const float Vmax = 6.2;   // Input voltage that would give 2.5V in voltage devider.
const int   VStep = 1024; // Each voltage step in the analog pin will indicate this voltage is given.
float VoltageDivider = 0; // VoltageDivider = (ADC/(R2/(R1+R2)))/1024

// Servos Matrix
const int ALLMATRIX = 13;
const int ALLSERVOS = 12;

// SG90 Servo PWM Pulse Traveling
const int PWMRES_Min = 1;       // PWM Resolution 1
const int PWMRES_Max = 180;     // PWM Resolution 180
const int SERVOMIN = 500;       // 500
const int SERVOMAX = 2400;      // 2400

// Servo update time
const int Update_time = 10;   // 10ms

// Backup Servo Value
float Running_Servo_POS [ALLSERVOS];

Servo All_Servo[ALLSERVOS];

int tune[] PROGMEM = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int motion[] PROGMEM = { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 500};

int Servo_home [][ALLMATRIX] = { 50,  90,  90,  130,  130,  90,  90,  50, 90 , 90, 90, 90, 500 };
int Servo_init [][ALLMATRIX] = { 90,  90,  90,  90,  90,  90,  90,  90, 90 , 90, 90, 90, 500 };

void Servo_PROGRAM_All(int iMatrix[][13],  int iSteps) {
  float inter_increment[ALLSERVOS];
  unsigned long target_time;
  unsigned long step_time;

  for ( int MainLoopIndex = 0; MainLoopIndex < iSteps; MainLoopIndex++)
  {
    int run_time = iMatrix [ MainLoopIndex ] [ ALLMATRIX - 1 ];
    if (run_time > Update_time) {
      for (int i = 0; i < ALLSERVOS; i++)
      {
        inter_increment[i] = (iMatrix[MainLoopIndex][i] - Running_Servo_POS[i]) / (run_time / Update_time);
      }

      target_time =  millis() + run_time;

      while (millis() < target_time) {
        step_time = millis() + Update_time;
        // Servo run to step's position
        for (int i = 0; i < ALLSERVOS; i++)
        {
          All_Servo[i].writeMicroseconds((Running_Servo_POS[i] + inter_increment[i] + tune[i] + offset[i]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
          Running_Servo_POS[i] = Running_Servo_POS[i] + inter_increment[i];
        }

        while (millis() < step_time); // wait to 10ms
        yield();
      }
    }  else {
      //  Move to target angle, run_time < Update_time
      for (int i = 0; i < ALLSERVOS; i++)
      {
        All_Servo[i].writeMicroseconds((iMatrix[MainLoopIndex][i] + tune[i] + offset[i]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
        Running_Servo_POS[i] = iMatrix[MainLoopIndex][i];
      }
    }
  }   // end of main_loop
}

void Servo_PROGRAM_Run(int iMatrix[][11],  int iSteps) {
  float inter_increment[10];
  unsigned long target_time;
  unsigned long step_time;

  for ( int MainLoopIndex = 0; MainLoopIndex < iSteps; MainLoopIndex++)
  {
    int run_time = iMatrix [ MainLoopIndex ] [ ALLMATRIX - 1 ];
    for (int i = 0; i < 10; i++)
    {
      inter_increment[i] = (iMatrix[MainLoopIndex][i] - Running_Servo_POS[i]) / (run_time / Update_time);
    }
    target_time =  millis() + run_time;

    while (millis() < target_time) {
      step_time = millis() + Update_time;
      // Servo run to step's position

      // servo 和 offset 要一致
      // 0 : p2 -> p5
      All_Servo[1].writeMicroseconds(floor((Running_Servo_POS[0] + inter_increment[0] + tune[1] + offset_12x[1]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN));
      Running_Servo_POS[0] = Running_Servo_POS[0] + inter_increment[0];
      // 1: p5 -> p4
      All_Servo[4].writeMicroseconds(floor((Running_Servo_POS[1] + inter_increment[1] + tune[4] + offset_12x[4]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN));
      Running_Servo_POS[1] = Running_Servo_POS[1] + inter_increment[1];
      // 2: p13 -> p11
      All_Servo[7].writeMicroseconds(floor((Running_Servo_POS[2] + inter_increment[2] + tune[7] + offset_12x[7]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN));
      Running_Servo_POS[2] = Running_Servo_POS[2] + inter_increment[2];
      // 3: p10 -> p13
      All_Servo[2].writeMicroseconds(floor((Running_Servo_POS[3] + inter_increment[3] + tune[7] + offset_12x[2]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN));
      Running_Servo_POS[3] = Running_Servo_POS[3] + inter_increment[3];
      // 4: p4 -> p3
      All_Servo[10].writeMicroseconds(floor((Running_Servo_POS[4] + inter_increment[4] + tune[10] + offset_12x[10]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN));
      Running_Servo_POS[4] = Running_Servo_POS[4] + inter_increment[4];
      // 5: p7 -> p7
      All_Servo[5].writeMicroseconds(floor((Running_Servo_POS[5] + inter_increment[5] + tune[5] + offset_12x[5]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN));
      Running_Servo_POS[5] = Running_Servo_POS[5] + inter_increment[5];
      // 6: p12 -> p12
      All_Servo[6].writeMicroseconds(floor((Running_Servo_POS[6] + inter_increment[6] + tune[6] + offset_12x[6]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN));
      Running_Servo_POS[6] = Running_Servo_POS[6] + inter_increment[6];
      // 7: p11 -> p17
      All_Servo[8].writeMicroseconds(floor((Running_Servo_POS[7] + inter_increment[7] + tune[7] + offset_12x[8]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN));
      Running_Servo_POS[7] = Running_Servo_POS[7] + inter_increment[7];

      // do mapping from V1 to V2

      delay(10);
    }
  }   // end of main_loop
}

String splitString(String data, char separator, int index)
{
  int stringData = 0;        //variable to count data part nr
  String dataPart = "";      //variable to hole the return text

  for (int i = 0; i < data.length(); i++) {
    if (data[i] == separator) {
      //Count the number of times separator character appears in the text
      stringData++;
    } else if (stringData == index) {
      //get the text when separator is the rignt one
      dataPart.concat(data[i]);
    } else if (stringData > index) {
      //return text and stop if the next separator appears - to save CPU-time
      return dataPart;
      break;
    }
  }
  return dataPart;
}

float init_pos[] PROGMEM = {15.0, 0, 65.0};

struct Single_Leg
{
  float x;
  float y;
  float z;
  float dx;
  float dy;
  float dz;
  float angle_1;  // High
  float angle_2;  // Middle
  float angle_3;  // Low
};

struct Single_Leg FR_leg;
struct Single_Leg FL_leg;
struct Single_Leg BL_leg;
struct Single_Leg BR_leg;

void set_default_pos() {
  FR_leg.dx = 0;    BL_leg.dx = 0;
  FL_leg.dx = 0;    BR_leg.dx = 0;
  FR_leg.dy = 0;    BL_leg.dy = 0;
  FL_leg.dy = 0;    BR_leg.dy = 0;
  FR_leg.dz = 0;    BL_leg.dz = 0;
  FL_leg.dz = 0;    BR_leg.dz = 0;
}

int Move_4T_pattern [][12] PROGMEM = {
  { -10 , 10 , 0 , -3 , -3 , 0 , -4 , -4 , 0 , -10 , -10 , 0},
  { -8 , 8 , -1 , -4 , -4 , 0 , -3 , -3 , 0 , -10 , -10 , 0},
  { -6 , 6 , -2 , -4 , -4 , 0 , -2 , -2 , 0 , -9 , -9 , 0},
  { -4 , 4 , -3 , -5 , -5 , 0 , -2 , -2 , 0 , -8 , -8 , 0},
  { -2 , 2 , -4 , -6 , -6 , 0 , -1 , -1 , 0 , -8 , -8 , 0},
  {0 , 0 , -5 , -6 , -6 , 0 , 0 , 0 , 0 , -7 , -7 , 0},
  {2 , -2 , -4 , -7 , -7 , 0 , 0 , 0 , 0 , -6 , -6 , 0},
  {4 , -4 , -3 , -8 , -8 , 0 , 1 , 1 , 0 , -6 , -6 , 0},
  {6 , -6 , -2 , -8 , -8 , 0 , 2 , 2 , 0 , -5 , -5 , 0},
  {8 , -8 , -1 , -9 , -9 , 0 , 2 , 2 , 0 , -4 , -4 , 0},
  {10 , -10 , 0 , -10 , -10 , 0 , 3 , 3 , 0 , -4 , -4 , 0},
  {10 , -10 , 0 , -8 , -8 , -1 , 4 , 4 , 0 , -3 , -3 , 0},
  {9 , -9 , 0 , -6 , -6 , -2 , 4 , 4 , 0 , -2 , -2 , 0},
  {8 , -8 , 0 , -4 , -4 , -3 , 5 , 5 , 0 , -2 , -2 , 0},
  {8 , -8 , 0 , -2 , -2 , -4 , 6 , 6 , 0 , -1 , -1 , 0},
  {7 , -7 , 0 , 0 , 0 , -5 , 6 , 6 , 0 , 0 , 0 , 0},
  {6 , -6 , 0 , 2 , 2 , -4 , 7 , 7 , 0 , 0 , 0 , 0},
  {6 , -6 , 0 , 4 , 4 , -3 , 8 , 8 , 0 , 1 , 1 , 0},
  {5 , -5 , 0 , 6 , 6 , -2 , 8 , 8 , 0 , 2 , 2 , 0},
  {4 , -4 , 0 , 8 , 8 , -1 , 9 , 9 , 0 , 2 , 2 , 0},
  {4 , -4 , 0 , 10 , 10 , 0 , 10 , 10 , 0 , 3 , 3 , 0},
  {3 , -3 , 0 , 10 , 10 , 0 , 8 , 8 , -1 , 4 , 4 , 0},
  {2 , -2 , 0 , 9 , 9 , 0 , 6 , 6 , -2 , 4 , 4 , 0},
  {2 , -2 , 0 , 8 , 8 , 0 , 4 , 4 , -3 , 5 , 5 , 0},
  {1 , -1 , 0 , 8 , 8 , 0 , 2 , 2 , -4 , 6 , 6 , 0},
  {0 , 0 , 0 , 7 , 7 , 0 , 0 , 0 , -5 , 6 , 6 , 0},
  {0 , 0 , 0 , 6 , 6 , 0 , -2 , -2 , -4 , 7 , 7 , 0},
  { -1 , 1 , 0 , 6 , 6 , 0 , -4 , -4 , -3 , 8 , 8 , 0},
  { -2 , 2 , 0 , 5 , 5 , 0 , -6 , -6 , -2 , 8 , 8 , 0},
  { -2 , 2 , 0 , 4 , 4 , 0 , -8 , -8 , -1 , 9 , 9 , 0},
  { -3 , 3 , 0 , 4 , 4 , 0 , -10 , -10 , 0 , 10 , 10 , 0},
  { -4 , 4 , 0 , 3 , 3 , 0 , -10 , -10 , 0 , 8 , 8 , -1},
  { -4 , 4 , 0 , 2 , 2 , 0 , -9 , -9 , 0 , 6 , 6 , -2},
  { -5 , 5 , 0 , 2 , 2 , 0 , -8 , -8 , 0 , 4 , 4 , -3},
  { -6 , 6 , 0 , 1 , 1 , 0 , -8 , -8 , 0 , 2 , 2 , -4},
  { -6 , 6 , 0 , 0 , 0 , 0 , -7 , -7 , 0 , 0 , 0 , -5},
  { -7 , 7 , 0 , 0 , 0 , 0 , -6 , -6 , 0 , -2 , -2 , -4},
  { -8 , 8 , 0 , -1 , -1 , 0 , -6 , -6 , 0 , -4 , -4 , -3},
  { -8 , 8 , 0 , -2 , -2 , 0 , -5 , -5 , 0 , -6 , -6 , -2},
  { -9 , 9 , 0 , -2 , -2 , 0 , -4 , -4 , 0 , -8 , -8 , -1}
};

int Move_2T_pattern [][12] PROGMEM = {
  { -10 , -10 , 0 , 10 , 10 , 0 , 10 , 10 , 0 , -10 , -10 , 0},
  { -10 , -10 , 0 , 10 , 10 , 0 , 10 , 10 , 0 , -10 , -10 , 0},
  { -9 , -9 , 0 , 9 , 9 , 0 , 9 , 9 , 0 , -9 , -9 , 0},
  { -8 , -8 , -1 , 8 , 8 , -1 , 8 , 8 , 0 , -8 , -8 , 0},
  { -8 , -8 , -1 , 8 , 8 , -1 , 8 , 8 , 0 , -8 , -8 , 0},
  { -7 , -7 , -1 , 7 , 7 , -1 , 7 , 7 , 0 , -7 , -7 , 0},
  { -6 , -6 , -2 , 6 , 6 , -2 , 6 , 6 , 0 , -6 , -6 , 0},
  { -6 , -6 , -2 , 6 , 6 , -2 , 6 , 6 , 0 , -6 , -6 , 0},
  { -5 , -5 , -2 , 5 , 5 , -2 , 5 , 5 , 0 , -5 , -5 , 0},
  { -4 , -4 , -3 , 4 , 4 , -3 , 4 , 4 , 0 , -4 , -4 , 0},
  { -4 , -4 , -3 , 4 , 4 , -3 , 4 , 4 , 0 , -4 , -4 , 0},
  { -3 , -3 , -3 , 3 , 3 , -3 , 3 , 3 , 0 , -3 , -3 , 0},
  { -2 , -2 , -4 , 2 , 2 , -4 , 2 , 2 , 0 , -2 , -2 , 0},
  { -2 , -2 , -4 , 2 , 2 , -4 , 2 , 2 , 0 , -2 , -2 , 0},
  { -1 , -1 , -4 , 1 , 1 , -4 , 1 , 1 , 0 , -1 , -1 , 0},
  {0 , 0 , -5 , 0 , 0 , -5 , 0 , 0 , 0 , 0 , 0 , 0},
  {0 , 0 , -4 , 0 , 0 , -4 , 0 , 0 , 0 , 0 , 0 , 0},
  {1 , 1 , -4 , -1 , -1 , -4 , -1 , -1 , 0 , 1 , 1 , 0},
  {2 , 2 , -4 , -2 , -2 , -4 , -2 , -2 , 0 , 2 , 2 , 0},
  {2 , 2 , -3 , -2 , -2 , -3 , -2 , -2 , 0 , 2 , 2 , 0},
  {3 , 3 , -3 , -3 , -3 , -3 , -3 , -3 , 0 , 3 , 3 , 0},
  {4 , 4 , -3 , -4 , -4 , -3 , -4 , -4 , 0 , 4 , 4 , 0},
  {4 , 4 , -2 , -4 , -4 , -2 , -4 , -4 , 0 , 4 , 4 , 0},
  {5 , 5 , -2 , -5 , -5 , -2 , -5 , -5 , 0 , 5 , 5 , 0},
  {6 , 6 , -2 , -6 , -6 , -2 , -6 , -6 , 0 , 6 , 6 , 0},
  {6 , 6 , -1 , -6 , -6 , -1 , -6 , -6 , 0 , 6 , 6 , 0},
  {7 , 7 , -1 , -7 , -7 , -1 , -7 , -7 , 0 , 7 , 7 , 0},
  {8 , 8 , -1 , -8 , -8 , -1 , -8 , -8 , 0 , 8 , 8 , 0},
  {8 , 8 , 0 , -8 , -8 , 0 , -8 , -8 , 0 , 8 , 8 , 0},
  {9 , 9 , 0 , -9 , -9 , 0 , -9 , -9 , 0 , 9 , 9 , 0},
  {10 , 10 , 0 , -10 , -10 , 0 , -10 , -10 , 0 , 10 , 10 , 0},
  {10 , 10 , 0 , -10 , -10 , 0 , -10 , -10 , 0 , 10 , 10 , 0},
  {9 , 9 , 0 , -9 , -9 , 0 , -9 , -9 , 0 , 9 , 9 , 0},
  {8 , 8 , 0 , -8 , -8 , 0 , -8 , -8 , -1 , 8 , 8 , -1},
  {8 , 8 , 0 , -8 , -8 , 0 , -8 , -8 , -1 , 8 , 8 , -1},
  {7 , 7 , 0 , -7 , -7 , 0 , -7 , -7 , -1 , 7 , 7 , -1},
  {6 , 6 , 0 , -6 , -6 , 0 , -6 , -6 , -2 , 6 , 6 , -2},
  {6 , 6 , 0 , -6 , -6 , 0 , -6 , -6 , -2 , 6 , 6 , -2},
  {5 , 5 , 0 , -5 , -5 , 0 , -5 , -5 , -2 , 5 , 5 , -2},
  {4 , 4 , 0 , -4 , -4 , 0 , -4 , -4 , -3 , 4 , 4 , -3},
  {4 , 4 , 0 , -4 , -4 , 0 , -4 , -4 , -3 , 4 , 4 , -3},
  {3 , 3 , 0 , -3 , -3 , 0 , -3 , -3 , -3 , 3 , 3 , -3},
  {2 , 2 , 0 , -2 , -2 , 0 , -2 , -2 , -4 , 2 , 2 , -4},
  {2 , 2 , 0 , -2 , -2 , 0 , -2 , -2 , -4 , 2 , 2 , -4},
  {1 , 1 , 0 , -1 , -1 , 0 , -1 , -1 , -4 , 1 , 1 , -4},
  {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , -5 , 0 , 0 , -5},
  {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , -4 , 0 , 0 , -4},
  { -1 , -1 , 0 , 1 , 1 , 0 , 1 , 1 , -4 , -1 , -1 , -4},
  { -2 , -2 , 0 , 2 , 2 , 0 , 2 , 2 , -4 , -2 , -2 , -4},
  { -2 , -2 , 0 , 2 , 2 , 0 , 2 , 2 , -3 , -2 , -2 , -3},
  { -3 , -3 , 0 , 3 , 3 , 0 , 3 , 3 , -3 , -3 , -3 , -3},
  { -4 , -4 , 0 , 4 , 4 , 0 , 4 , 4 , -3 , -4 , -4 , -3},
  { -4 , -4 , 0 , 4 , 4 , 0 , 4 , 4 , -2 , -4 , -4 , -2},
  { -5 , -5 , 0 , 5 , 5 , 0 , 5 , 5 , -2 , -5 , -5 , -2},
  { -6 , -6 , 0 , 6 , 6 , 0 , 6 , 6 , -2 , -6 , -6 , -2},
  { -6 , -6 , 0 , 6 , 6 , 0 , 6 , 6 , -1 , -6 , -6 , -1},
  { -7 , -7 , 0 , 7 , 7 , 0 , 7 , 7 , -1 , -7 , -7 , -1},
  { -8 , -8 , 0 , 8 , 8 , 0 , 8 , 8 , -1 , -8 , -8 , -1},
  { -8 , -8 , 0 , 8 , 8 , 0 , 8 , 8 , 0 , -8 , -8 , 0},
  { -9 , -9 , 0 , 9 , 9 , 0 , 9 , 9 , 0 , -9 , -9 , 0},
};

void Cal_FL(float x1, float y1, float z1)
{
  FL_leg.x =  init_pos[0] + x1;
  FL_leg.y =  init_pos[1] + y1;
  FL_leg.z =  init_pos[2] + z1;

  float L;
  L = sqrt(FL_leg.x * FL_leg.x + FL_leg.y * FL_leg.y + FL_leg.z * FL_leg.z);

  FL_leg.angle_1 = atan(FL_leg.y / FL_leg.z) * 57.596f;
  FL_leg.angle_2 = (acos(L / 76 - 1157 / 76 / L) - asin(FL_leg.x / L)) * 57.596f;
  FL_leg.angle_3 = 180 - acos((4045 - L * L) / 3876) * 57.596f;

  FL_leg.angle_1 = 90 - FL_leg.angle_1 ;
  FL_leg.angle_2 = 90 + FL_leg.angle_2 ;
  FL_leg.angle_3 = FL_leg.angle_3 ;

  All_Servo[3].writeMicroseconds(floor(FL_leg.angle_1 + offset_12x[3]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
  All_Servo[2].writeMicroseconds(floor(FL_leg.angle_2 + offset_12x[2]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
  All_Servo[7].writeMicroseconds(floor(FL_leg.angle_3 + offset_12x[7]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
}

void Cal_BL(float x1, float y1, float z1)
{
  BL_leg.x =  init_pos[0] + x1;
  BL_leg.y =  init_pos[1] + y1;
  BL_leg.z =  init_pos[2] + z1;

  float L;
  L = sqrt(BL_leg.x * BL_leg.x + BL_leg.y * BL_leg.y + BL_leg.z * BL_leg.z);

  BL_leg.angle_1 = atan(BL_leg.y / BL_leg.z) * 57.596f;
  BL_leg.angle_2 = (acos(L / 76 - 1157 / 76 / L) - asin(BL_leg.x / L)) * 57.596f;
  BL_leg.angle_3 = 180 - acos((4045 - L * L) / 3876) * 57.596f;

  BL_leg.angle_1 = 90 + BL_leg.angle_1 ;
  BL_leg.angle_2 = 90 - BL_leg.angle_2 ;
  BL_leg.angle_3 = 180 - BL_leg.angle_3 ;

  All_Servo[9].writeMicroseconds(floor(BL_leg.angle_1 + offset_12x[9]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
  All_Servo[8].writeMicroseconds(floor(BL_leg.angle_2 + offset_12x[8]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
  All_Servo[6].writeMicroseconds(floor(BL_leg.angle_3 + offset_12x[6]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
}

void Cal_FR(float x1, float y1, float z1)
{
  FR_leg.x =  init_pos[0] + x1;
  FR_leg.y =  init_pos[1] + y1;
  FR_leg.z =  init_pos[2] + z1;

  float L;
  L = sqrt(FR_leg.x * FR_leg.x + FR_leg.y * FR_leg.y + FR_leg.z * FR_leg.z);

  FR_leg.angle_1 = atan(FR_leg.y / FR_leg.z) * 57.596f;
  FR_leg.angle_2 = (acos(L / 76 - 1157 / 76 / L) - asin(FR_leg.x / L)) * 57.596f;
  FR_leg.angle_3 = 180 - acos((4045 - L * L) / 3876) * 57.596f;

  FR_leg.angle_1 = 90 - FR_leg.angle_1 ;
  FR_leg.angle_2 = 90 - FR_leg.angle_2 ;
  FR_leg.angle_3 = 180 - FR_leg.angle_3 ;

  All_Servo[0].writeMicroseconds(floor(FR_leg.angle_1 + offset_12x[0]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
  All_Servo[1].writeMicroseconds(floor(FR_leg.angle_2 + offset_12x[1]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
  All_Servo[4].writeMicroseconds(floor(FR_leg.angle_3 + offset_12x[4]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
}

void Cal_BR(float x1, float y1, float z1)
{
  BR_leg.x =  init_pos[0] + x1;
  BR_leg.y =  init_pos[1] + y1;
  BR_leg.z =  init_pos[2] + z1;

  float L;
  L = sqrt(BR_leg.x * BR_leg.x + BR_leg.y * BR_leg.y + BR_leg.z * BR_leg.z);

  BR_leg.angle_1 = atan(BR_leg.y / BR_leg.z) * 57.596f;
  BR_leg.angle_2 = (acos(L / 76 - 1157 / 76 / L) - asin(BR_leg.x / L)) * 57.596f;
  BR_leg.angle_3 = 180 - acos((4045 - L * L) / 3876) * 57.596f;

  BR_leg.angle_1 = 90 + BR_leg.angle_1 ;
  BR_leg.angle_2 = 90 + BR_leg.angle_2 ;
  BR_leg.angle_3 = BR_leg.angle_3 ;

  All_Servo[11].writeMicroseconds(floor(BR_leg.angle_1 + offset_12x[11]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
  All_Servo[10].writeMicroseconds(floor(BR_leg.angle_2 + offset_12x[10]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
  All_Servo[5].writeMicroseconds(floor(BR_leg.angle_3 + offset_12x[5]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
}

void set_servo_PWM(int d_time) {
  Cal_FR(FR_leg.dx, FR_leg.dy, FR_leg.dz);
  Cal_FL(FL_leg.dx, FL_leg.dy, FL_leg.dz);
  Cal_BR(BR_leg.dx, BR_leg.dy, BR_leg.dz);
  Cal_BL(BL_leg.dx, BL_leg.dy, BL_leg.dz);

  delay(d_time);
}

void Move_Forward(void)
{
  static int i = 0;
  set_default_pos();

  int Move_X = 20;
  int Move_Y = 0;
  int Move_Z = 35;

  for (i = 0; i < 40; i++) {
    FL_leg.dx = Move_X / 10 * Move_4T_pattern[i][0];
    BR_leg.dx = -Move_X / 10 * Move_4T_pattern[i][3];

    FR_leg.dx = -Move_X / 10 * Move_4T_pattern[i][6];
    BL_leg.dx = Move_X / 10 * Move_4T_pattern[i][9];

    FL_leg.dz = Move_Z / 10 * Move_4T_pattern[i][2];
    BR_leg.dz = Move_Z / 10 * Move_4T_pattern[i][5];

    FR_leg.dz = Move_Z / 10 * Move_4T_pattern[i][8];
    BL_leg.dz = Move_Z / 10 * Move_4T_pattern[i][11];

    set_servo_PWM(10);
  }
}


void Move_Backward(void)
{
  static int i = 0;
  set_default_pos();

  int Move_X = -20;
  int Move_Y = 0;
  int Move_Z = 35;

  for (i = 0; i < 40; i++) {
    FL_leg.dx = Move_X / 10 * Move_4T_pattern[i][0];
    BR_leg.dx = -Move_X / 10 * Move_4T_pattern[i][3];

    FR_leg.dx = -Move_X / 10 * Move_4T_pattern[i][6];
    BL_leg.dx = Move_X / 10 * Move_4T_pattern[i][9];

    FL_leg.dz = Move_Z / 10 * Move_4T_pattern[i][2];
    BR_leg.dz = Move_Z / 10 * Move_4T_pattern[i][5];

    FR_leg.dz = Move_Z / 10 * Move_4T_pattern[i][8];
    BL_leg.dz = Move_Z / 10 * Move_4T_pattern[i][11];

    set_servo_PWM(10);
  }
}

void Move_Circle_L_2T(void)
{
  static int i = 0;
  set_default_pos();

  int Move_X = 0;
  int Move_Y = -10;
  int Move_Z = 30;

  for (i = 0; i < 60; i++) {
    FL_leg.dx = Move_X / 10 * Move_2T_pattern[i][0];
    BR_leg.dx = Move_X / 10 * Move_2T_pattern[i][3];

    FR_leg.dx = Move_X / 10 * Move_2T_pattern[i][6];
    BL_leg.dx = Move_X / 10 * Move_2T_pattern[i][9];

    FL_leg.dy = Move_Y / 10 * Move_2T_pattern[i][1];
    BR_leg.dy = Move_Y / 10 * Move_2T_pattern[i][4];

    FR_leg.dy = Move_Y / 10 * Move_2T_pattern[i][7];
    BL_leg.dy = Move_Y / 10 * Move_2T_pattern[i][10];

    FL_leg.dz = Move_Z / 10 * Move_2T_pattern[i][2];
    BR_leg.dz = Move_Z / 10 * Move_2T_pattern[i][5];

    FR_leg.dz = Move_Z / 10 * Move_2T_pattern[i][8];
    BL_leg.dz = Move_Z / 10 * Move_2T_pattern[i][11];

    set_servo_PWM(2);
  }
}

void Move_Circle_R_2T(void)   // from forward_R
{
  static int i = 0;
  set_default_pos();

  int Move_X = 0;
  int Move_Y = 10;
  int Move_Z = 30;

  for (i = 0; i < 60; i++) {
    FL_leg.dx = Move_X / 10 * Move_2T_pattern[i][0];
    BR_leg.dx = Move_X / 10 * Move_2T_pattern[i][3];

    FR_leg.dx = Move_X / 10 * Move_2T_pattern[i][6];
    BL_leg.dx = Move_X / 10 * Move_2T_pattern[i][9];

    FL_leg.dy = Move_Y / 10 * Move_2T_pattern[i][1];
    BR_leg.dy = Move_Y / 10 * Move_2T_pattern[i][4];

    FR_leg.dy = Move_Y / 10 * Move_2T_pattern[i][7];
    BL_leg.dy = Move_Y / 10 * Move_2T_pattern[i][10];

    FL_leg.dz = Move_Z / 10 * Move_2T_pattern[i][2];
    BR_leg.dz = Move_Z / 10 * Move_2T_pattern[i][5];

    FR_leg.dz = Move_Z / 10 * Move_2T_pattern[i][8];
    BL_leg.dz = Move_Z / 10 * Move_2T_pattern[i][11];

    set_servo_PWM(2);
  }
}


void print_all_data() {
  Serial.print("{");
  Serial.print(FL_leg.dx);
  Serial.print(" , ");
  Serial.print(FL_leg.dx);
  Serial.print(" , ");
  Serial.print(FL_leg.dz);
  Serial.print(" , ");
  Serial.print(BR_leg.dx);
  Serial.print(" , ");
  Serial.print(BR_leg.dx);
  Serial.print(" , ");
  Serial.print(BR_leg.dz);
  Serial.print(" , ");
  Serial.print(FR_leg.dx);
  Serial.print(" , ");
  Serial.print(FR_leg.dx);
  Serial.print(" , ");
  Serial.print(FR_leg.dz);
  Serial.print(" , ");
  Serial.print(BL_leg.dx);
  Serial.print(" , ");
  Serial.print(BL_leg.dx);
  Serial.print(" , ");
  Serial.print(BL_leg.dz);
  Serial.println("}");
}

void Move_Turn_L(void)
{
  static int i = 0;
  set_default_pos();

  int Move_XR = 30;
  int Move_XL = 10;
  int Move_Y = 0;
  int Move_Z = 24;

  for (i = 0; i < 40; i++) {
    FL_leg.dx = Move_XL / 10 * Move_4T_pattern[i][0];
    BR_leg.dx = -Move_XR / 10 * Move_4T_pattern[i][3];

    FR_leg.dx = -Move_XR / 10 * Move_4T_pattern[i][6];
    BL_leg.dx = Move_XL / 10 * Move_4T_pattern[i][9];

    FL_leg.dy = Move_Y / 10 * Move_4T_pattern[i][1];
    BR_leg.dy = Move_Y / 10 * Move_4T_pattern[i][4];

    FR_leg.dy = Move_Y / 10 * Move_4T_pattern[i][7];
    BL_leg.dy = Move_Y / 10 * Move_4T_pattern[i][10];

    FL_leg.dz = Move_Z / 10 * Move_4T_pattern[i][2];
    BR_leg.dz = Move_Z / 10 * Move_4T_pattern[i][5];

    FR_leg.dz = Move_Z / 10 * Move_4T_pattern[i][8];
    BL_leg.dz = Move_Z / 10 * Move_4T_pattern[i][11];

    set_servo_PWM(10);
  }
}

void Move_Turn_R(void)
{
  static int i = 0;
  set_default_pos();

  int Move_XR = 10;
  int Move_XL = 30;
  int Move_Y = 0;
  int Move_Z = 24;

  for (i = 0; i < 40; i++) {
    FL_leg.dx = Move_XL / 10 * Move_4T_pattern[i][0];
    BR_leg.dx = -Move_XR / 10 * Move_4T_pattern[i][3];

    FR_leg.dx = -Move_XR / 10 * Move_4T_pattern[i][6];
    BL_leg.dx = Move_XL / 10 * Move_4T_pattern[i][9];

    FL_leg.dy = Move_Y / 10 * Move_4T_pattern[i][1];
    BR_leg.dy = Move_Y / 10 * Move_4T_pattern[i][4];

    FR_leg.dy = Move_Y / 10 * Move_4T_pattern[i][7];
    BL_leg.dy = Move_Y / 10 * Move_4T_pattern[i][10];

    FL_leg.dz = Move_Z / 10 * Move_4T_pattern[i][2];
    BR_leg.dz = Move_Z / 10 * Move_4T_pattern[i][5];

    FR_leg.dz = Move_Z / 10 * Move_4T_pattern[i][8];
    BL_leg.dz = Move_Z / 10 * Move_4T_pattern[i][11];

    set_servo_PWM(10);
  }
}

void Move_Stop() {
  set_default_pos();
  Cal_FR(FR_leg.dx, FR_leg.dy, FR_leg.dz);
  Cal_FL(FL_leg.dx, FL_leg.dy, FL_leg.dz);
  Cal_BR(BR_leg.dx, BR_leg.dy, BR_leg.dz);
  Cal_BL(BL_leg.dx, BL_leg.dy, BL_leg.dz);
}

int user_data_sit_Step = 2;
int user_data_sit  [][ALLMATRIX]   PROGMEM  =  {
  // P2, P5, P13, P10, P4, P7, P12, P11, P17, P16, ms
  {50,  90,  90,  130,  130,  90,  90,  50, 90, 90, 100},
  {20,  30, 150, 160, 160, 150,  30,  20, 50, 50, 200},
};

void setup() {
  Serial.begin(115200);      // initialize serial communication
  pinMode(BatteryPin, INPUT);

  All_Servo[0].attach(2, SERVOMIN, SERVOMAX);
  All_Servo[1].attach(5, SERVOMIN, SERVOMAX);
  All_Servo[2].attach(13, SERVOMIN, SERVOMAX);
  All_Servo[3].attach(10, SERVOMIN, SERVOMAX);

  All_Servo[4].attach(4, SERVOMIN, SERVOMAX);
  All_Servo[5].attach(7, SERVOMIN, SERVOMAX);
  All_Servo[6].attach(12, SERVOMIN, SERVOMAX);
  All_Servo[7].attach(11, SERVOMIN, SERVOMAX);

  All_Servo[8].attach(17, SERVOMIN, SERVOMAX);
  All_Servo[9].attach(16, SERVOMIN, SERVOMAX);
  All_Servo[10].attach(3, SERVOMIN, SERVOMAX);
  All_Servo[11].attach(15, SERVOMIN, SERVOMAX);

  // Battery Voltage Divider
  VoltageDivider = ((Vmax / (R2 / (R1 + R2))) / VStep);
  Move_Stop();

  // Initialize BLE subsystem & get BLE address
  LBLE.begin();
  while (!LBLE.ready()) {
    delay(100);
  }

  Serial.print("Device Address = [");
  LBLEAddress ble_address;
  String qter_cat_address;

  ble_address = LBLE.getDeviceAddress();
  qter_cat_address = ble_address.toString();
  Serial.print(qter_cat_address);
  Serial.println("]");

  String qter_cat_name;
  qter_cat_name = "CAT-" + qter_cat_address.substring(0, 2) + qter_cat_address.substring(3, 5);
  //  qter_cat_name = "CAT-xxxx";  // Please change to CAT-NXX.

  // Setup the Remote Control's Name
  LRemote.setName(qter_cat_name);

  LRemote.setGrid(3, 10);
  LRemote.setOrientation(RC_PORTRAIT);
  // home_btn.setText("HOME");
  // home_btn.setPos(1, 2);
  //  home_btn.setSize(1, 2);
  //  home_btn.setColor(RC_ORANGE);
  //  LRemote.addControl(home_btn);
  HL_btn.setText("H->L");
  HL_btn.setPos(1, 2);
  HL_btn.setSize(1, 2);
  HL_btn.setColor(RC_ORANGE);
  LRemote.addControl(HL_btn);
  fwd_btn.setText("FORWARD");
  fwd_btn.setPos(1, 0);
  fwd_btn.setSize(1, 2);
  fwd_btn.setColor(RC_BLUE);
  LRemote.addControl(fwd_btn);
  bwd_btn.setText("BACKWARD");
  bwd_btn.setPos(1, 4);
  bwd_btn.setSize(1, 2);
  bwd_btn.setColor(RC_BLUE);
  LRemote.addControl(bwd_btn);
  left_btn.setText("LEFT");
  left_btn.setPos(0, 2);
  left_btn.setSize(1, 2);
  left_btn.setColor(RC_BLUE);
  LRemote.addControl(left_btn);
  right_btn.setText("RIGHT");
  right_btn.setPos(2, 2);
  right_btn.setSize(1, 2);
  right_btn.setColor(RC_BLUE);
  LRemote.addControl(right_btn);
  demo1_btn.setText("Demo1");
  demo1_btn.setPos(0, 0);
  demo1_btn.setSize(1, 2);
  demo1_btn.setColor(RC_GREY);
  LRemote.addControl(demo1_btn);
  demo2_btn.setText("Demo2");
  demo2_btn.setPos(2, 0);
  demo2_btn.setSize(1, 2);
  demo2_btn.setColor(RC_GREY);
  LRemote.addControl(demo2_btn);

  demo3_btn.setText("Spin_L");
  demo3_btn.setPos(0, 4);
  demo3_btn.setSize(1, 2);
  demo3_btn.setColor(RC_GREY);
  LRemote.addControl(demo3_btn);
  demo4_btn.setText("Spin_R");
  demo4_btn.setPos(2, 4);
  demo4_btn.setSize(1, 2);
  demo4_btn.setColor(RC_GREY);
  LRemote.addControl(demo4_btn);
  act1_btn.setText("Act1");
  act1_btn.setPos(0, 7);
  act1_btn.setSize(1, 1);
  act1_btn.setColor(RC_GREEN);
  LRemote.addControl(act1_btn);
  act2_btn.setText("Act2");
  act2_btn.setPos(1, 7);
  act2_btn.setSize(1, 1);
  act2_btn.setColor(RC_GREEN);
  LRemote.addControl(act2_btn);
  act3_btn.setText("Act3");
  act3_btn.setPos(2, 7);
  act3_btn.setSize(1, 1);
  act3_btn.setColor(RC_GREEN);
  LRemote.addControl(act3_btn);
  act4_btn.setText("Act4");
  act4_btn.setPos(0, 8);
  act4_btn.setSize(1, 1);
  act4_btn.setColor(RC_GREEN);
  LRemote.addControl(act4_btn);
  act5_btn.setText("Act5");
  act5_btn.setPos(1, 8);
  act5_btn.setSize(1, 1);
  act5_btn.setColor(RC_GREEN);
  LRemote.addControl(act5_btn);
  act6_btn.setText("Act6");
  act6_btn.setPos(2, 8);
  act6_btn.setSize(1, 1);
  act6_btn.setColor(RC_GREEN);
  LRemote.addControl(act6_btn);
  act7_btn.setText("Act7");
  act7_btn.setPos(0, 9);
  act7_btn.setSize(1, 1);
  act7_btn.setColor(RC_GREEN);
  LRemote.addControl(act7_btn);
  act8_btn.setText("Act8");
  act8_btn.setPos(1, 9);
  act8_btn.setSize(1, 1);
  act8_btn.setColor(RC_GREEN);
  LRemote.addControl(act8_btn);
  act9_btn.setText("Act9");
  act9_btn.setPos(2, 9);
  act9_btn.setSize(1, 1);
  act9_btn.setColor(RC_GREEN);
  LRemote.addControl(act9_btn);

  // Start broadcasting our remote contoller
  LRemote.begin();
  Serial.println("LRemote begin ...");
}

void loop() {

  // Battery Voltage
  BatteryVol = analogRead(BatteryPin) * VoltageDivider * 0.1;
  //  Serial.println(BatteryVol);

  if (BatteryVol > BatteryLowVol) {


    // BLE central device, e.g. an mobile app
    if (!LRemote.connected())
    {
      //   Serial.println("Waiting for connection ...");
      delay(10);
    }
    else
    {
      delay(10);
    }

    // Process the incoming BLE write request
    LRemote.process();

    if ( HL_btn.getValue() == 0) {
      init_pos[2] = 65.0; // high
      Move_Stop();
    }
    if ( HL_btn.getValue() == 1 ) {
      init_pos[2] = 55.0; // low
      Move_Stop();
    }

    if (Serial.available() > 0) {
      String servo = Serial.readStringUntil('#');
      Serial.println(servo);
      if (servo == "save" ) {
        int eeprom_address = 0;
        for (int i = 0; i < ALLSERVOS; i++) {
          EEPROM.put(eeprom_address, offset[i]);
          eeprom_address = eeprom_address + sizeof(offset[i]);
          delay (500);
          Serial.print(i);
          Serial.println(" process on going !!");
        }
        Serial.println("Finish!!");
      } else {
        for (int i = 0; i <= ALLSERVOS; i++) {
          String pos = splitString(servo, ',', i);
          motion[i] = pos.toInt();
        }
        String t_time = splitString(servo, ',', ALLSERVOS);
        if (t_time != "") {
          int  Servo_Prg_tmp [][ALLMATRIX] = {
            { motion[0], motion[1], motion[2], motion[3], motion[4], motion[5], motion[6], motion[7], motion[8], motion[9], motion[10], motion[11], motion[12] }
          };
          Servo_PROGRAM_All(Servo_Prg_tmp, 1);
        }
        else {
          Serial.println("command error!!");
        }
      }
    }
    if (home_btn.isValueChanged() && home_btn.getValue())
    {
      //    Serial.println("HOME");
      Move_Stop();
      delay(300);
    }
    if (fwd_btn.isValueChanged() && fwd_btn.getValue())
    {
      set_default_pos();
      Move_Forward();
      Move_Forward();
      Move_Forward();
      Move_Stop();
    }
    if (bwd_btn.isValueChanged() && bwd_btn.getValue())
    {
      set_default_pos();
      Move_Backward();
      Move_Backward();
      Move_Backward();
      Move_Stop();
    }
    if (left_btn.isValueChanged() && left_btn.getValue())
    {
      Move_Turn_L();
      Move_Turn_L();
      Move_Turn_L();
    }
    if (right_btn.isValueChanged() && right_btn.getValue())
    {
      Move_Turn_R();
      Move_Turn_R();
      Move_Turn_R();
    }
    if (demo1_btn.isValueChanged() && demo1_btn.getValue())
    {

    }
    if (demo2_btn.isValueChanged() && demo2_btn.getValue())
    {

    }
    if (demo3_btn.isValueChanged() && demo3_btn.getValue())
    {
      Move_Circle_L_2T();
      Move_Circle_L_2T();
      Move_Stop();
    }
    if (demo4_btn.isValueChanged() && demo4_btn.getValue())
    {
      Move_Circle_R_2T();
      Move_Circle_R_2T();
      Move_Stop();
    }
    if (act1_btn.isValueChanged() && act1_btn.getValue())
    {
      Move_Stop();
    }
    if (act2_btn.isValueChanged() && act2_btn.getValue())
    {
      Move_Stop();
    }
    if (act3_btn.isValueChanged() && act3_btn.getValue())
    {
      Move_Stop();
    }
    if (act4_btn.isValueChanged() && act4_btn.getValue())
    {
      Move_Stop();
    }
    if (act5_btn.isValueChanged() && act5_btn.getValue())
    {
      Move_Stop();
    }
    if (act6_btn.isValueChanged() && act6_btn.getValue())
    {
      Move_Stop();
    }
    if (act7_btn.isValueChanged() && act7_btn.getValue())
    {
      Move_Stop();
    }
    if (act8_btn.isValueChanged() && act8_btn.getValue())
    {
      Move_Stop();
    }
    if (act9_btn.isValueChanged() && act9_btn.getValue())
    {
      Move_Stop();
    }
  }    // Battery check loop
}
