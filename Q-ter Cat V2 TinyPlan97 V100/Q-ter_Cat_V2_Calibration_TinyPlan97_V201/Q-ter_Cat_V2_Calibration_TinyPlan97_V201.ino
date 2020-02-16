//////////////////////////////////////////////////////////////////////
// Q-ter Cat (Using MiniPlan TinyPlan97) by Mason E & D (2020.1)
/////////////////////////////////////////////////////////////////////

// Author : Mason 2019/7/30, masonchen1003@gmail.com
// FB : https://www.facebook.com/mason.chen.1420
// Licensed under the Creative Commons - Attribution - Non-Commercial license.

#include <Servo.h>

// servo offset, pleaee update the offset values
int offset[] PROGMEM = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // default
// int offset[] PROGMEM = { 7,-4, 11, 7,-1,-3,10,7,6,0};  // default

// Servos Matrix
const int ALLMATRIX = 13;        // P2 + P5 + P13 + P10 + P4 + P7 + P12 + P11 + P17 + P16 + Run Time
const int ALLSERVOS = 12;        // P2 + P5 + P13 + P10 + P4 + P7 + P12 + P11 + P17 + P16

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

int tune[] PROGMEM = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // blue
int motion[] PROGMEM = { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 500};

int Servo_home [][ALLMATRIX] = { 50,  90,  90,  130,  130,  90,  90,  50, 90 , 90, 90, 90, 500 };
int Servo_init [][ALLMATRIX] = { 90,  90,  90,  90,  90,  90,  90,  90, 90 , 90, 90, 90, 500 };


void Servo_PROGRAM_Run(int iMatrix[][13],  int iSteps) {
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

///////////////////////////////////////////////////////
// Motion data by Mason

int Motion_data_demo1_Step = 7;
int Motion_data_demo1 [][ALLMATRIX] PROGMEM = {
  // P2, P5, P13, P10, P4, P7, P12, P11, P17, P16, ms
  {50,  90,  90,  130,  130,  90,  90,  50,  90,  90, 500},
  {50, 130,  50,  130, 140, 110,  40,  40, 90,  160, 350},
  {100,  60,  50,  130, 140, 120,  30,  40, 110, 160, 500},
  {40, 100,  50,  130, 140, 120,  30,  40, 70,  130, 500},
  {100,  60,  50,  130, 140, 120,  20,  40, 110, 160, 500},
  {40, 100,  50,  130, 140, 120,  20,  40, 70,  130, 500},
  {50,  90,  90,  130,  130,  90,  90,  50,  90,  90, 800},
};

int user_data_sit_Step = 2;
int user_data_sit  [][ALLMATRIX]   PROGMEM  =  {
  // P2, P5, P13, P10, P4, P7, P12, P11, P17, P16, ms
  {50,  90,  90,  130,  130,  90,  90,  50, 90, 90, 100},
  {20,  30, 150, 160, 160, 150,  30,  20, 50, 50, 200},
};

void setup() {
  Serial.begin(9600);      // initialize serial communication

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

  for (int i = 0; i < ALLSERVOS; i++)
  {
    Running_Servo_POS[i] = 90;
  }

  //  Servo_PROGRAM_Run(Servo_home, 1);
  Servo_PROGRAM_Run(Servo_init, 1);

}

void loop() {

  if (Serial.available() > 0) {
    String servo = Serial.readStringUntil('#');
    for (int i = 0; i <= 12; i++) {
      String pos = splitString(servo, ',', i);
      motion[i] = pos.toInt();
    }
    String t_time = splitString(servo, ',', 12);
    if (t_time != "") {
      int  Servo_Prg_tmp [][ALLMATRIX] = {
        // P2,     P3,     P4,     P5,     P7,     P8,      P9,      P11,      Run Time
        { motion[0], motion[1], motion[2], motion[3], motion[4], motion[5], motion[6], motion[7], motion[8], motion[9], motion[10], motion[11], motion[12] }
      };
      Servo_PROGRAM_Run(Servo_Prg_tmp, 1);
    }
    else {
      // do nothing
      Serial.println("command error!!");
    }
  }

}
