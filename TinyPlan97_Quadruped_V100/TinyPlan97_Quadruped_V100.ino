// TinyPlan97 + Linkit Remote app
// Author : Mason 2018/5/05, masonchen1003@gmail.com
// Editor : Shinwei 2019/7/24, https://github.com/GeorgeChiou

// Arduino for LinkIt 7697
// https://docs.labs.mediatek.com/resource/linkit7697-arduino/en/environment-setup

// Using LinkIt Remote APP
// https://docs.labs.mediatek.com/resource/linkit7697-arduino/en/developer-guide/using-linkit-remote


// FL1 : P13 , FL2 : P10
// RL1 : P11 , RL2 : P12
// FR1 : P5  , FR2 : P2
// RR1 : P4  , RR2 : P7

// FL2                FR2
//     FL1        FR1
//
//       TinyPlan97
//
//     RL1        RR1
// RL2                RR2


#include <Servo.h>
#include <LRemote.h>

// Version
String FW_Version = "TinyPlan97 Quadruped V1.0 (2019/07/24)";

// Battery
#define BatteryPin 14
float BatteryVol = 0;
int BatteryCount = 0;
const float BatteryLowVol = 4.5;  // Low Battery Alarm Voltage
const float R1 = 100;             // 10Kohm, The Voltage devider resistor R1
const float R2 = 47;              // 4.7Kohm, The Voltage devider resistor R2
const float Vmax = 6.2;           // Input voltage Max Voltage
const int   VStep = 1024;         // Each voltage step in the analog pin will indicate this voltage is given.
float VoltageDevider = 0;         // voltageDevider = (ADC/(R2/(R1+R2)))/1024

// Servo
#define ServoPinFL1   13
#define ServoPinFL2   10
#define ServoPinRL1   11
#define ServoPinRL2   12
#define ServoPinFR1   5
#define ServoPinFR2   2
#define ServoPinRR1   4
#define ServoPinRR2   7

// Servos Matrix
const int ALLMATRIX = 9;   // P10 + P13 + P11 + P12 + P2 + P5 + P4 + P7 + Run Time
const int ALLSERVOS = 8;   // P10 + P13 + P11 + P12 + P2 + P5 + P4 + P7

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

// servo offset, pleaee update the offset values
//                      P10, P13, P11, P12, P2, P5, P4, P7, ms
int offset[] PROGMEM = {  0,   0,   0,   0,  0,  0,  0,  0};   // default
//int offset[] PROGMEM = { 0, 0, 0, 0, 0, 0, 0, 0};   // default

int tune[] PROGMEM = { 0, 0, 0, 0, 0, 0, 0, 0};
int motion[] PROGMEM = { 90, 90, 90, 90, 90, 90, 90, 90, 500};

int Servo_Home [][ALLMATRIX] = { 90,  90,  90,  90,  90,  90,  90,  90,  500 };

// Label
LRemoteLabel InformLabel;
LRemoteLabel BatteryLabel;

// Button
LRemoteButton HomeButton;
LRemoteButton ForwardButton;
LRemoteButton BackwardButton;
LRemoteButton TurnLeftButton;
LRemoteButton TurnRightButton;


/*------------------------------------------------------------*/
// Standby
int user_data_standby_Step = 1;
int user_data_standby  [][ALLMATRIX]   PROGMEM  =  {
  // P10, P13, P11, P12, P2, P5, P4, P7, ms
  {130, 50, 130, 50, 50, 130, 50, 130, 500},
};

// Forward
int Motion_data_1_Step = 14;
int Motion_data_1 [][ALLMATRIX] PROGMEM = {
  // P10, P13, P11, P12,  P2,  P5,  P4,  P7, ms
  {   110,  85,  97,  34,  37, 115,  65, 143, 100   },
  {    98,  59,  97,  34,  37, 115,  65, 143, 100   },
  {   135,  51,  97,  34,  37, 115,  65, 143, 100   },
  {   143,  65, 115,  37,  34,  97,  51, 135, 100   },
  {   143,  65, 115,  37,  34,  97,  59,  98, 100   },
  {   143,  65, 115,  37,  34,  97,  85, 110, 100   },
  {   143,  65, 115,  37,  34,  97,  83, 146, 100   },
  {   143,  65, 115,  37,  70,  95,  83, 146, 100   },
  {   143,  65, 115,  37,  82, 121,  83, 146, 100   },
  {   143,  65, 115,  37,  45, 129,  83, 146, 100   },
  {   146,  83, 129,  45,  37, 115,  65, 143, 100   },
  {   146,  83, 129,  66,  37, 115,  65, 143, 100   },
  {   146,  83,  95,  70,  37, 115,  65, 143, 100   },
  {   146,  83,  97,  34,  37, 115,  65, 143, 100   },
};

// Backward
int Motion_data_2_Step = 14;
int Motion_data_2 [][ALLMATRIX] PROGMEM = {
  // P10, P13, P11, P12,  P2,  P5,  P4,  P7, ms
  {   146,  83,  95,  70,  37, 115,  65, 143, 100   },
  {   146,  83, 121,  82,  37, 115,  65, 143, 100   },
  {   146,  83, 129,  45,  37, 115,  65, 143, 100   },
  {   143,  65, 115,  37,  45, 129,  83, 146, 100   },
  {   143,  65, 115,  37,  82, 121,  83, 146, 100   },
  {   143,  65, 115,  37,  70,  95,  83, 146, 100   },
  {   143,  65, 115,  37,  34,  97,  83, 146, 100   },
  {   143,  65, 115,  37,  34,  97,  85, 110, 100   },
  {   143,  65, 115,  37,  34,  97,  59,  98, 100   },
  {   143,  65, 115,  37,  34,  97,  51, 135, 100   },
  {   135,  51,  97,  34,  37, 115,  65, 143, 100   },
  {   114,  51,  97,  34,  37, 115,  65, 143, 100   },
  {   110,  85,  97,  34,  37, 115,  65, 143, 100   },
  {   146,  83,  97,  34,  37, 115,  65, 143, 100   },
};

// Left
int Motion_data_3_Step = 14;
int Motion_data_3 [][ALLMATRIX] PROGMEM = {
  // P10, P13, P11, P12,  P2,  P5,  P4,  P7, ms
  {   146,  83,  95,  70,  37, 115,  65, 143, 100   },
  {   145,  74, 119,  66,  35, 106,  52, 147, 100   },
  {   145,  74, 128,  33,  35, 106,  52, 147, 100   },
  {   145,  74, 128,  33,  35, 106,  52, 147, 100   },
  {   145,  74, 128,  33,  35, 106,  61, 114, 100   },
  {   143,  65, 115,  37,  34,  97,  85, 110, 100   },
  {   143,  65, 115,  37,  34,  97,  83, 146, 100   },
  {   143,  65, 115,  37,  70,  95,  83, 146, 100   },
  {   147,  52, 106,  35,  66, 119,  74, 145, 100   },
  {   147,  52, 106,  35,  33, 128,  74, 145, 100   },
  {   147,  52, 106,  35,  33, 128,  74, 145, 100   },
  {   114,  61, 106,  35,  33, 128,  74, 145, 100   },
  {   110,  85,  97,  34,  37, 115,  65, 143, 100   },
  {   146,  83,  97,  34,  37, 115,  65, 143, 100   },
};

// Right
int Motion_data_4_Step = 14;
int Motion_data_4 [][ALLMATRIX] PROGMEM = {
  // P10, P13, P11, P12,  P2,  P5,  P4,  P7, ms
  {   110,  85,  97,  34,  37, 115,  65, 143, 100   },
  {   141,  52, 106,  35,  33, 128,  74, 145, 100   },
  {   147,  52, 106,  35,  33, 128,  74, 145, 100   },
  {   147,  52, 106,  35,  33, 128,  74, 145, 100   },
  {   147,  52, 106,  35,  66, 119,  74, 145, 100   },
  {   143,  65, 115,  37,  70,  95,  83, 146, 100   },
  {   143,  65, 115,  37,  34,  97,  83, 146, 100   },
  {   143,  65, 115,  37,  34,  97,  85, 110, 100   },
  {   145,  74, 128,  33,  35, 106,  61, 114, 100   },
  {   145,  74, 128,  33,  35, 106,  52, 147, 100   },
  {   145,  74, 128,  33,  35, 106,  52, 147, 100   },
  {   145,  74, 119,  66,  35, 106,  52, 147, 100   },
  {   146,  83,  97,  40,  37, 115,  65, 143, 100   },
  {   146,  83,  97,  34,  37, 115,  65, 143, 100   },
};


/*------------------------------------------------------------*/
void Servo_PROGRAM_Run(int iMatrix[][ALLMATRIX],  int iSteps)
{
  float inter_increment[ALLSERVOS];
  unsigned long target_time;
  unsigned long step_time;

  for ( int MainLoopIndex = 0; MainLoopIndex < iSteps; MainLoopIndex++)
  {
    int run_time = iMatrix [ MainLoopIndex ] [ ALLMATRIX - 1 ];
    if (run_time > Update_time)
    {
      for (int i = 0; i < ALLSERVOS; i++)
      {
        inter_increment[i] = (iMatrix[MainLoopIndex][i] - Running_Servo_POS[i]) / (run_time / Update_time);
      }

      target_time =  millis() + run_time;

      while (millis() < target_time)
      {
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
    }
    else
    {
      //  Move to target angle, run_time < Update_time
      for (int i = 0; i < ALLSERVOS; i++)
      {
        All_Servo[i].writeMicroseconds((iMatrix[MainLoopIndex][i] + tune[i] + offset[i]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN);
        Running_Servo_POS[i] = iMatrix[MainLoopIndex][i];
      }
    }
  }
}


/*------------------------------------------------------------*/
void setup()
{
  // Initialize serial communications at 9600 bps:
  Serial.begin(9600);

  // Battery PIN Set
  pinMode(BatteryPin, INPUT);

  // Initialize PWM Pin
  All_Servo[0].attach(ServoPinFL2, SERVOMIN, SERVOMAX);
  All_Servo[1].attach(ServoPinFL1, SERVOMIN, SERVOMAX);
  All_Servo[2].attach(ServoPinRL1, SERVOMIN, SERVOMAX);
  All_Servo[3].attach(ServoPinRL2, SERVOMIN, SERVOMAX);

  All_Servo[4].attach(ServoPinFR2, SERVOMIN, SERVOMAX);
  All_Servo[5].attach(ServoPinFR1, SERVOMIN, SERVOMAX);
  All_Servo[6].attach(ServoPinRR1, SERVOMIN, SERVOMAX);
  All_Servo[7].attach(ServoPinRR2, SERVOMIN, SERVOMAX);

  // Initialize Servo Home
  for (int i = 0; i < ALLSERVOS; i++)
  {
    Running_Servo_POS[i] = 90;
  }
  Servo_PROGRAM_Run(Servo_Home, 1);

  // Battery Voltage Devider
  VoltageDevider = ((Vmax / (R2 / (R1 + R2))) / VStep);

  // Initialize BLE subsystem & get BLE address
  LBLE.begin();
  while (!LBLE.ready()) {
    delay(100);
  }

  Serial.print("Device Address = [");
  LBLEAddress ble_address;
  String aeg_address;

  ble_address = LBLE.getDeviceAddress();
  aeg_address = ble_address.toString();
  Serial.print(aeg_address);
  Serial.println("]");

  String aeg_name;
  aeg_name = "QDP-" + aeg_address.substring(0, 2) + aeg_address.substring(3, 5);

  // Setup the Remote Control's Name
  LRemote.setName(aeg_name);

  // Setup the Remote Control's UI canvas
  LRemote.setOrientation(RC_PORTRAIT);
  LRemote.setGrid(3, 6);

  // Label
  InformLabel.setText(FW_Version);
  InformLabel.setPos(0, 0);
  InformLabel.setSize(3, 1);
  InformLabel.setColor(RC_GREY);
  LRemote.addControl(InformLabel);

  BatteryLabel.setText("Battery 0.0V");
  BatteryLabel.setPos(0, 4);
  BatteryLabel.setSize(3, 1);
  BatteryLabel.setColor(RC_GREY);
  LRemote.addControl(BatteryLabel);

  // Button
  HomeButton.setText("STANDBY");
  HomeButton.setPos(1, 2);
  HomeButton.setSize(1, 1);
  HomeButton.setColor(RC_ORANGE);
  LRemote.addControl(HomeButton);

  ForwardButton.setText("FORWARD");
  ForwardButton.setPos(1, 1);
  ForwardButton.setSize(1, 1);
  ForwardButton.setColor(RC_BLUE);
  LRemote.addControl(ForwardButton);

  BackwardButton.setText("BACKWARD");
  BackwardButton.setPos(1, 3);
  BackwardButton.setSize(1, 1);
  BackwardButton.setColor(RC_BLUE);
  LRemote.addControl(BackwardButton);

  TurnLeftButton.setText("LEFT");
  TurnLeftButton.setPos(0, 2);
  TurnLeftButton.setSize(1, 1);
  TurnLeftButton.setColor(RC_BLUE);
  LRemote.addControl(TurnLeftButton);

  TurnRightButton.setText("RIGHT");
  TurnRightButton.setPos(2, 2);
  TurnRightButton.setSize(1, 1);
  TurnRightButton.setColor(RC_BLUE);
  LRemote.addControl(TurnRightButton);

  // Start broadcasting our remote contoller
  LRemote.begin();
  Serial.println("LRemote begin ...");
}

/*------------------------------------------------------------*/
void loop()
{
  // BLE central device, e.g. an mobile app
  if (!LRemote.connected())
  {
    Serial.println("Waiting for connection ...");
  }
  else
  {
    // Process the incoming BLE write request
    LRemote.process();

    // Main Delay
    delay(10);
  }


  // Battery Voltage
  if (BatteryCount >= 50)
  {
    BatteryCount = 0;
    BatteryVol = analogRead(BatteryPin) * VoltageDevider * 0.1;
    BatteryLabel.updateText("Battery " + String(BatteryVol, 2) + "V");
  }
  else
  {
    BatteryCount++;
  }


  // STANDBY
  if (HomeButton.isValueChanged() && HomeButton.getValue())
  {
    Servo_PROGRAM_Run(user_data_standby, user_data_standby_Step);
    delay(300);
  }

  // FORWARD
  if (ForwardButton.isValueChanged() && ForwardButton.getValue())
  {
    Servo_PROGRAM_Run(Motion_data_1, Motion_data_1_Step);
    Servo_PROGRAM_Run(Motion_data_1, Motion_data_1_Step);
    Servo_PROGRAM_Run(user_data_standby, user_data_standby_Step);
  }

  // BACKWARD
  if (BackwardButton.isValueChanged() && BackwardButton.getValue())
  {
    Servo_PROGRAM_Run(Motion_data_2, Motion_data_2_Step);
    Servo_PROGRAM_Run(Motion_data_2, Motion_data_2_Step);
    Servo_PROGRAM_Run(user_data_standby, user_data_standby_Step);
  }

  // TURN LEFT
  if (TurnLeftButton.isValueChanged() && TurnLeftButton.getValue())
  {
    Servo_PROGRAM_Run(Motion_data_3, Motion_data_3_Step);
    Servo_PROGRAM_Run(Motion_data_3, Motion_data_3_Step);
    Servo_PROGRAM_Run(user_data_standby, user_data_standby_Step);
  }

  // TURN RIGHT
  if (TurnRightButton.isValueChanged() && TurnRightButton.getValue())
  {
    Servo_PROGRAM_Run(Motion_data_4, Motion_data_4_Step);
    Servo_PROGRAM_Run(Motion_data_4, Motion_data_4_Step);
    Servo_PROGRAM_Run(user_data_standby, user_data_standby_Step);
  }
}
