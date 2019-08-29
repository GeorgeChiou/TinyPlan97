// 最後編輯 2019-7-23 by ShinWei Chiou

#include <LRemote.h>

// Version
String FW_Version = "TinyPlan97 v3.0 (2019/7/23)";

// Battery
#define BatteryPin  14
float BatteryVol = 0;
int BatteryCount = 0;
const float BatteryLowVol = 4.2;
const float R1 = 100;      // 10Kohm, The Voltage devider resistor R1
const float R2 = 47;       // 4.7Kohm, The Voltage devider resistor R2
const float Vmax = 6.2;    // Input voltage Max Voltage
const int   VStep = 1024;  // Each voltage step in the analog pin will indicate this voltage is given.
float VoltageDevider = 0;  // voltageDevider = (ADC/(R2/(R1+R2)))/1024

// Label
LRemoteLabel InformLabel;
LRemoteLabel BatteryLabel;

/*------------------------------------------------------------*/
void All_Pull_High()
{
  digitalWrite(10, HIGH);
  digitalWrite(13, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(17, HIGH);
  digitalWrite(16, HIGH);

  digitalWrite(2,  HIGH);
  digitalWrite(5,  HIGH);
  digitalWrite(4,  HIGH);
  digitalWrite(7,  HIGH);
  digitalWrite(3,  HIGH);
  digitalWrite(15, HIGH);

  digitalWrite(8,  HIGH);
  digitalWrite(9,  HIGH);
}

void All_Pull_Low()
{
  digitalWrite(10, LOW);
  digitalWrite(13, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(17, LOW);
  digitalWrite(16, LOW);

  digitalWrite(2,  LOW);
  digitalWrite(5,  LOW);
  digitalWrite(4,  LOW);
  digitalWrite(7,  LOW);
  digitalWrite(3,  LOW);
  digitalWrite(15, LOW);

  digitalWrite(8,  LOW);
  digitalWrite(9,  LOW);
}

/*------------------------------------------------------------*/
void setup()
{
  // Initialize serial communications at 9600 bps:
  Serial.begin(9600);

  // Battery PIN Set
  pinMode(BatteryPin, INPUT);

  // PIN Set
  pinMode(10, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  // Battery Voltage Devider
  VoltageDevider = ((Vmax / (R2 / (R1 + R2))) / VStep);

  // Initialize BLE subsystem & get BLE address
  LBLE.begin();
  while (!LBLE.ready()) {
    delay(100);
  }

  // Get the Device Address
  Serial.print("Device Address = [");
  LBLEAddress ble_address;
  String robot_address;

  ble_address = LBLE.getDeviceAddress();
  robot_address = ble_address.toString();
  Serial.print(robot_address);
  Serial.println("]");

  String robot_name;
  robot_name = "TP-" + robot_address.substring(0, 2) + robot_address.substring(3, 5);

  // Setup the Remote Control's Name
  LRemote.setName(robot_name);

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
  BatteryLabel.setPos(0, 1);
  BatteryLabel.setSize(3, 1);
  BatteryLabel.setColor(RC_GREY);
  LRemote.addControl(BatteryLabel);

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


  // Change Output
  if (BatteryCount < 25)
  {
    All_Pull_High();
  }
  else
  {
    All_Pull_Low();
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
}
