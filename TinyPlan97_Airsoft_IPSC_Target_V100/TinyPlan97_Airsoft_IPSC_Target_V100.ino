// 最後編輯 2019-12-07 by ShinWei Chiou

// DFPlayer mini mp3 module.
// github as default source provider
// https://github.com/DFRobot/DFPlayer-Mini-mp3

// TinyPlan97 V3.X
// DFplayer RX : P9 , TX : P2
// Battery : P14
// Sensor : P8
// Servo1 : P10


#include <Servo.h>
#include <LRemote.h>
#include <SoftwareSerial.h>

// DFplayer
#include <DFPlayer_Mini_Mp3.h>

// On LinkIt 7697, the RX pin must be one of the EINT pins.
// There are no limitations on TX pin.
SoftwareSerial mp3Serial(9, 2); // RX, TX

const int TargerServoUp = 180;
const int TargerServoDown = 10;

// Battery
#define BatteryPin 14
float BatteryVol = 0;
int BatteryCount = 0;
const float BatteryLowVol = 4.2;
const float R1 = 100;      // V3.0 - 10Kohm,  The Voltage devider resistor R1
const float R2 = 47;       // V3.0 - 4.7Kohm, The Voltage devider resistor R2
//const float R1 = 200;    // V2.0 - 200Kohm, The Voltage devider resistor R1
//const float R2 = 100;    // V2.0 - 100Kohm, The Voltage devider resistor R2
const float Vmax = 6.2;    // Input voltage Max Voltage
const int   VStep = 1024;  // Each voltage step in the analog pin will indicate this voltage is given.
float VoltageDevider = 0;  // voltageDevider = (ADC/(R2/(R1+R2)))/1024

// Servo
#define ServoPin1   10
#define ServoPin2   13
#define ServoPin3   11
#define ServoPin4   12
#define ServoPin5   17
#define ServoPin6   16
#define ServoPin7   5
#define ServoPin8   4
#define ServoPin9   7
#define ServoPin10  3
#define ServoPin11  15

// Create Servo object
Servo TargerServo1;
Servo TargerServo2;
Servo TargerServo3;
Servo TargerServo4;
Servo TargerServo5;
Servo TargerServo6;
Servo TargerServo7;
Servo TargerServo8;
Servo TargerServo9;
Servo TargerServo10;
Servo TargerServo11;

// Sensor
#define Sensor 8

// Time
int StartCountTime = 0;
float RunCountTime1 = 0;
float RunCountTime2 = 0;

// Music
const int VolumeMax = 30;
const int VolumeMin = 5;
int Volume_Value = 15;

// Label
LRemoteLabel TimeLabel1;
LRemoteLabel TimeLabel2;
LRemoteLabel BatteryLabel;

// Button
LRemoteButton StartButton;
LRemoteButton StopButton;
LRemoteButton VolmaxButton;
LRemoteButton VolminButton;


/*------------------------------------------------------------*/
void Targer_ServoUp()
{
  TargerServo1.write(TargerServoUp);
  TargerServo2.write(TargerServoUp);
  TargerServo3.write(TargerServoUp);
  TargerServo4.write(TargerServoUp);
  TargerServo5.write(TargerServoUp);
  TargerServo6.write(TargerServoUp);
  TargerServo7.write(TargerServoUp);
  TargerServo8.write(TargerServoUp);
  TargerServo9.write(TargerServoUp);
  TargerServo10.write(TargerServoUp);
  TargerServo11.write(TargerServoUp);
}


/*------------------------------------------------------------*/
void Targer_ServoDown()
{
  TargerServo1.write(TargerServoDown);
  TargerServo2.write(TargerServoDown);
  TargerServo3.write(TargerServoDown);
  TargerServo4.write(TargerServoDown);
  TargerServo5.write(TargerServoDown);
  TargerServo6.write(TargerServoDown);
  TargerServo7.write(TargerServoDown);
  TargerServo8.write(TargerServoDown);
  TargerServo9.write(TargerServoDown);
  TargerServo10.write(TargerServoDown);
  TargerServo11.write(TargerServoDown);
}


/*------------------------------------------------------------*/
void setup()
{
  // Initialize serial communications at 9600 bps:
  Serial.begin(9600);

  // Set the data rate for the SoftwareSerial port
  mp3Serial.begin(9600);

  // Add for DFplayer
  mp3_set_serial (mp3Serial);

  // PIN Set
  pinMode(Sensor, INPUT_PULLUP);

  // Battery PIN Set
  pinMode(BatteryPin, INPUT);

  // Initialize Servo
  TargerServo1.attach(ServoPin1);
  TargerServo1.write(TargerServoDown);
  TargerServo2.attach(ServoPin2);
  TargerServo2.write(TargerServoDown);
  TargerServo3.attach(ServoPin3);
  TargerServo3.write(TargerServoDown);
  TargerServo4.attach(ServoPin4);
  TargerServo4.write(TargerServoDown);
  TargerServo5.attach(ServoPin5);
  TargerServo5.write(TargerServoDown);
  TargerServo6.attach(ServoPin6);
  TargerServo6.write(TargerServoDown);
  TargerServo7.attach(ServoPin7);
  TargerServo7.write(TargerServoDown);
  TargerServo8.attach(ServoPin8);
  TargerServo8.write(TargerServoDown);
  TargerServo9.attach(ServoPin9);
  TargerServo9.write(TargerServoDown);
  TargerServo10.attach(ServoPin10);
  TargerServo10.write(TargerServoDown);
  TargerServo11.attach(ServoPin11);
  TargerServo11.write(TargerServoDown);

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
  aeg_name = "TAG-" + aeg_address.substring(0, 2) + aeg_address.substring(3, 5);

  // Setup the Remote Control's Name
  LRemote.setName(aeg_name);

  // Setup the Remote Control's UI canvas
  LRemote.setOrientation(RC_LANDSCAPE);
  LRemote.setGrid(7, 4);

  // Add a Label
  TimeLabel1.setText("0.00 Sec");
  TimeLabel1.setPos(0, 0);
  TimeLabel1.setSize(2, 2);
  TimeLabel1.setColor(RC_BLUE);
  LRemote.addControl(TimeLabel1);

  TimeLabel2.setText("0.00 Sec");
  TimeLabel2.setPos(2, 0);
  TimeLabel2.setSize(2, 2);
  TimeLabel2.setColor(RC_BLUE);
  LRemote.addControl(TimeLabel2);

  BatteryLabel.setText("Battery 0.0V");
  BatteryLabel.setPos(0, 3);
  BatteryLabel.setSize(4, 1);
  BatteryLabel.setColor(RC_GREY);
  LRemote.addControl(BatteryLabel);

  // Add a Button
  StartButton.setText("START");
  StartButton.setPos(4, 0);
  StartButton.setSize(3, 3);
  StartButton.setColor(RC_GREEN);
  LRemote.addControl(StartButton);

  StopButton.setText("STOP");
  StopButton.setPos(4, 3);
  StopButton.setSize(3, 1);
  StopButton.setColor(RC_PINK);
  LRemote.addControl(StopButton);

  VolmaxButton.setText("VOL+");
  VolmaxButton.setPos(0, 2);
  VolmaxButton.setSize(2, 1);
  VolmaxButton.setColor(RC_YELLOW);
  LRemote.addControl(VolmaxButton);

  VolminButton.setText("VOL-");
  VolminButton.setPos(2, 2);
  VolminButton.setSize(2, 1);
  VolminButton.setColor(RC_YELLOW);
  LRemote.addControl(VolminButton);

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

  // Process the incoming BLE write request
  LRemote.process();


  // Start Button
  if (StartButton.isValueChanged())
  {
    if (StartButton.getValue() == 1 && StartCountTime == 0)
    {
      RunCountTime1 = 0;
      StartCountTime = 1;
      TimeLabel1.updateText(String(RunCountTime1, 2) + " Sec");
      TimeLabel2.updateText(String(RunCountTime2, 2) + " Sec");

      mp3_set_volume (Volume_Value);
      delay(10);

      Targer_ServoUp();

      mp3_play (7001);
      delay (3500);
      mp3_stop ();

      Targer_ServoDown();

      delay (2000);

      mp3_play (7002);
      delay (500);
      mp3_stop ();
    }
  }


  // Run Time
  if (StartCountTime == 1)
  {
    if (RunCountTime1 >= 300)
    {
      RunCountTime1 = 0;
      StartCountTime = 0;
      TimeLabel1.updateText(String(RunCountTime1, 2) + " Sec");
    }
    else
    {
      RunCountTime1 = RunCountTime1 + 0.01;
      TimeLabel1.updateText(String(RunCountTime1, 2) + " Sec");
    }
  }


  // Stop Button
  if (StopButton.isValueChanged())
  {
    if (StopButton.getValue() == 1)
    {
      Targer_ServoUp();

      RunCountTime1 = 0;
      StartCountTime = 0;
      TimeLabel1.updateText(String(RunCountTime1, 2) + " Sec");
      TimeLabel2.updateText(String(RunCountTime2, 2) + " Sec");

      delay(1000);
      Targer_ServoDown();
    }
  }


  // Adjust Volume Music
  if (VolmaxButton.isValueChanged())
  {
    if (VolmaxButton.getValue() == 1)
    {
      if (Volume_Value < 30)
      {
        Volume_Value = Volume_Value + 5;

        BatteryLabel.updateText("Volume = " + String(Volume_Value, 10));

        mp3_set_volume (Volume_Value);
        delay(10);
      }
      else
      {
        BatteryLabel.updateText("Volume = Max");
      }
    }
  }

  if (VolminButton.isValueChanged())
  {
    if (VolminButton.getValue() == 1)
    {
      if (Volume_Value > 5)
      {
        Volume_Value = Volume_Value - 5;

        BatteryLabel.updateText("Volume = " + String(Volume_Value, 10));

        mp3_set_volume (Volume_Value);
        delay(10);
      }
      else
      {
        BatteryLabel.updateText("Volume = Min");
      }
    }
  }


  // Battery Voltage
  if (BatteryCount >= 50)
  {
    BatteryCount = 0;
    BatteryVol = analogRead(BatteryPin) * VoltageDevider * 0.1;
    BatteryLabel.updateText("Battery " + String(BatteryVol, 2) + "V");
  }
  else {
    BatteryCount++;
  }


  // Targer Sensor
  if (digitalRead(Sensor) == 0  && StartCountTime == 1)
  {
    StartCountTime = 0;
    RunCountTime2 = RunCountTime1;
    TimeLabel1.updateText(String(RunCountTime1, 2) + " Sec");

    delay (2000);
    mp3_play (7003);
    delay (3500);
    mp3_stop ();
  }


  // Main Delay
  delay(10);
}
