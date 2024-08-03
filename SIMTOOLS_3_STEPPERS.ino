// Stepper microstepping settings :  32000

#include <Wire.h>
#include "PCF8574.h"
// #include <LiquidCrystal_I2C.h>

#define ENDSTOPS_INTERUPT_PIN 2
#define SHUTDOWN_INTERUPT_PIN 3
#define STEP_PIN1 4
#define DIR_PIN1 5
#define ENABLE_PIN1 6
#define STEP_PIN2 7
#define DIR_PIN2 8
#define ENABLE_PIN2 9
#define STEP_PIN3 10
#define DIR_PIN3 11
#define ENABLE_PIN3 12
// LiquidCrystal_I2C lcd(0x27, 16, 2);

PCF8574 endstops;
int endstopsState[6] = { 1 };
volatile bool inInterupt = false;
volatile bool inShutdown = false;

const int dataLength = 2; // Długość każdej z osi w bajtach
const char startMarker[] = "START";
const char stopMarker[] = "STOP";
const int startMarkerLength = 5;
const int stopMarkerLength = 4;
byte axis1[dataLength];
byte axis2[dataLength];
byte axis3[dataLength];

int stepPulses1 = 0, stepPulses2 = 0, stepPulses3 = 0;
// 14 bit                                                                                     // The number of pulse that each motor will receive. If the number is negative, it inverts the direction
// unsigned int motor1Target = 8191, motor2Target = 8191, motor3Target = 8191;        // Motors position we want to reach
// unsigned int motor1Position = 8191, motor2Position = 8191, motor3Position = 8191;  // The actual positions
// unsigned int motor1Max = 16384, motor2Max = 16384, motor3Max = 16384;
//15 bit
const uint16_t stepperPulses = 32768;
const uint16_t stepperMax = stepperPulses-1;
const uint16_t stepperMid = stepperPulses/2 -1;
uint16_t motor1Target = stepperMid, motor2Target = stepperMid, motor3Target = stepperMid;        // Motors position we want to reach
uint16_t motor1Position = stepperMid, motor2Position = stepperMid, motor3Position = stepperMid;  // The actual positions
uint16_t motor1Max = stepperMax, motor2Max = stepperMax, motor3Max = stepperMax;

const uint8_t speedNormal = 15;    //us for steps delay
const uint8_t speedHoming = 100;   //us for steps delay
const uint8_t speedShutdown = 50;  //us for steps delay
const uint8_t speedStart = 65;
const uint8_t ramp_pulses = uint8_t (0.08*stepperPulses);
uint8_t pulseWidth = speedNormal;     //From the HBS86h datasheet : For reliable response, pulse width should be longer than 10μs
uint8_t current_x_speed = speedStart;
uint8_t current_y_speed = speedStart;
uint8_t current_z_speed = speedStart;
uint16_t current_x = 0;
uint16_t current_y = 0;
uint16_t current_z = 0;
uint16_t x, y, z;
volatile bool autoHoming = true;
uint8_t homingState = 0;

void setup() {
  delay(2000);  //safety delay to flash the code
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(ENABLE_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);
  pinMode(STEP_PIN3, OUTPUT);
  pinMode(DIR_PIN3, OUTPUT);
  pinMode(ENABLE_PIN3, OUTPUT);

  // lcd.begin();
  // lcd.backlight();
  // lcd.print("Setup");
  void homingInterupt();
  void shutdown();
  endstops.begin(0x20);
  for (int i = 0; i < 6; i++) {
    endstops.pinMode(i, INPUT_PULLUP);
  }
  for (int i = 6; i < 8; i++) {
    endstops.pinMode(i, INPUT);
    endstops.digitalWrite(i, LOW);
  }
  pinMode(ENDSTOPS_INTERUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENDSTOPS_INTERUPT_PIN), homingInterupt, FALLING);
  // pinMode(SHUTDOWN_INTERUPT_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(SHUTDOWN_INTERUPT_PIN), shutdown, FALLING);

  // if(digitalRead(SHUTDOWN_INTERUPT_PIN) == 0)
  // {
  //   shutdown();
  // }

  digitalWrite(ENABLE_PIN1, HIGH); // enable motors
  digitalWrite(ENABLE_PIN2, HIGH);
  digitalWrite(ENABLE_PIN3, HIGH);

  Serial.begin(115200);  //To communicate with Simtools
  if (autoHoming) {
    homing();
  }
  // delay(1000);
  // digitalWrite(ENABLE_PIN1, HIGH); // enable motors
  // digitalWrite(ENABLE_PIN2, HIGH);
  // digitalWrite(ENABLE_PIN3, HIGH);
}

void loop() {
  SerialReader();  //Get the datas from Simtools
  // lcd.clear();
  // lcd.print("Pos:");
  // lcd.setCursor(6, 0);
  // lcd.print("Tar:");
  // lcd.setCursor(0, 1);
  // lcd.print(motor1Position);
  // lcd.setCursor(6, 1);
  // lcd.print(motor1Target);
  LimitManager();
  CommandWorker();                                      //Convert the position targets to pulse number
  MoveSteppers(stepPulses1, stepPulses2, stepPulses3);  //Set directions and send the pulses.
  delayMicroseconds(2137);
}

void MoveSteppers(int Step1, int Step2, int Step3) {
  x = abs(Step1);
  y = abs(Step2);
  z = abs(Step3);
  DirectionManager(Step1, Step2, Step3);  //put the motors in the right directions
  current_x_speed = speedStart;
  current_y_speed = speedStart;
  current_z_speed = speedStart;
  current_x = 0;
  current_y = 0;
  current_z = 0;
  // Serial.print("Step1: ");
  // Serial.print(Step1);
  // Serial.print(" ");
  // Serial.print("Step2: ");
  // Serial.print(Step2);
  // Serial.print(" ");
  // Serial.print("Step3: ");
  // Serial.print(Step3);
  // Serial.println("");
  // Serial.print("x: ");
  // Serial.print(x);
  // Serial.print(" ");
  // Serial.print("y: ");
  // Serial.print(x);
  // Serial.print(" ");
  // Serial.print("y: ");
  // Serial.print(z);
  // Serial.println("");

  while(x > current_x || y > current_y || z > current_z)
  {   
    
    if(current_x < x)
    {
      pulseWidth = current_x_speed;
      monoPulse(STEP_PIN1);
      current_x ++;
      current_x_speed = speedNormal + speedStart*(1 - constrain(current_x/ramp_pulses, 0, 1));
    }
    
    if(current_y < y)
    {
      pulseWidth = current_y_speed;
      monoPulse(STEP_PIN2);
      current_y ++;
      current_y_speed = speedNormal + speedStart*(1 - constrain(current_y/ramp_pulses, 0, 1));
    }
    if(current_z < z)
    {
      pulseWidth = current_z_speed;
      monoPulse(STEP_PIN3);
      current_z ++;
      current_z_speed = speedNormal + speedStart*(1 - constrain(current_z/(ramp_pulses*1.5), 0, 1));
    }
    
  }

  motor1Position = motor1Target;
  motor2Position = motor2Target;
  motor3Position = motor3Target;
  // for (int i = 0; i < min(min(x, y), z); i++) {  //Common moves
  //   triplePulse(STEP_PIN1, STEP_PIN2, STEP_PIN3);
  // }

  // if (x > z && y > z) {
  //   for (int i = 0; i < min(x, y); i++) {  //Common moves
  //     doublePulse(STEP_PIN1, STEP_PIN2);
  //   }
  // }

  // if (x > y && z > y) {
  //   for (int i = 0; i < min(x, z); i++) {  //Common moves
  //     doublePulse(STEP_PIN1, STEP_PIN3);
  //   }
  // }

  // if (y > x && z > x) {
  //   for (int i = 0; i < min(y, z); i++) {  //Common moves
  //     doublePulse(STEP_PIN2, STEP_PIN3);
  //   }
  // }

  // if (max(max(x, y), z) == x) {  //Now only M1 will move
  //   for (int i = 0; i < x - y - z; i++) {
  //     monoPulse(STEP_PIN1);
  //   }
  // }

  // if (max(max(x, y), z) == y) {  //Now only M2 will move
  //   for (int i = 0; i < y - x - z; i++) {
  //     monoPulse(STEP_PIN2);
  //   }
  // }

  // else  //Now only M3 will move
  // {
  //   for (int i = 0; i < z - x - y; i++) {
  //     monoPulse(STEP_PIN3);
  //   }
  // }
}

void CommandWorker() {
  stepPulses1 = motor1Target - motor1Position;
  stepPulses2 = motor2Target - motor2Position;
  stepPulses3 = motor3Target - motor3Position;
  // Serial.print("stepPulses1 ");
  // Serial.print(motor1Target);
  // Serial.print(" ");
  // Serial.print(motor1Position);
  // Serial.print(" ");
  // Serial.println(stepPulses1);
  // Serial.print("stepPulses2 ");
  // Serial.print(motor2Target);
  // Serial.print(" ");
  // Serial.print(motor2Position);
  // Serial.print(" ");
  // Serial.println(stepPulses2);
  // Serial.print("stepPulses3 ");
  // Serial.print(motor3Target);
  // Serial.print(" ");
  // Serial.print(motor3Position);
  // Serial.print(" ");
  // Serial.println(stepPulses3);
}

void monoPulse(int StepPin) {
  digitalWrite(StepPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(StepPin, LOW);
  delayMicroseconds(pulseWidth);
}

void doublePulse(int StepPin1, int StepPin2) {
  digitalWrite(StepPin1, HIGH);
  digitalWrite(StepPin2, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(StepPin1, LOW);
  digitalWrite(StepPin2, LOW);
  delayMicroseconds(pulseWidth);
}

void triplePulse(int StepPin1, int StepPin2, int StepPin3) {
  digitalWrite(StepPin1, HIGH);
  digitalWrite(StepPin2, HIGH);
  digitalWrite(StepPin3, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(StepPin1, LOW);
  digitalWrite(StepPin2, LOW);
  digitalWrite(StepPin3, LOW);
  delayMicroseconds(pulseWidth);
}

void SerialReader() {
  while(Serial.available() > 0) {
    if (findString(startMarker, startMarkerLength)) { // Sprawdź, czy pojawił się znacznik START
      Serial.readBytes(axis1, dataLength);
      Serial.readBytes(axis2, dataLength);
      Serial.readBytes(axis3, dataLength);
      if (findString(stopMarker, stopMarkerLength)) {
        // Konwersja danych z osi na wartości liczbowe
        motor1Target = (axis1[0] << 8) | axis1[1];
        motor2Target = (axis2[0] << 8) | axis2[1];
        motor3Target = (axis3[0] << 8) | axis3[1];
        motor1Target = constrain(motor1Target, 0, 65535);
        motor2Target = constrain(motor2Target, 0, 65535);
        motor3Target = constrain(motor3Target, 0, 65535);
        motor1Target = map(motor1Target, 0, 65535, 0, motor1Max);
        motor2Target = map(motor2Target, 0, 65535, 0, motor2Max);
        motor3Target = map(motor3Target, 0, 65535, 0, motor3Max);
      }
    }
  }
}

bool findString(const char *target, int length) {
  int index = 0;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == target[index]) {
      index++;
      if (index >= length) {
        return true;
      }
    } else {
      index = 0; // Resetuj indeks, jeśli znak nie pasuje
    }
  }
  return false;
}
// void SerialReader() {  // This function is the work of Sirnoname
//   // Simtools output : P<Axis1a><Axis2a><Axis3a>, Data bits : 15 bits, Parity : None, stop bits : 1
//   int Buffer = 0;        // It takes the value of the serial data
//   int BufferCount = -1;  // To count where we are in the serial datas

//   while (Serial.available()) {
//     if (BufferCount == -1) {
//       Buffer = Serial.read();
//       delayMicroseconds(100);
//       if (Buffer != 'P') {
//         BufferCount = -1;  // "P" is the marquer. If we read P, the next data is pitch
//       } else {
//         BufferCount = 0;
//       }
//     } else  //  if(BufferCount>=0)
//     {
//       Buffer = Serial.read();
//       delayMicroseconds(100);
//       commandBuffer[BufferCount] = Buffer;  // The first value next to "P" is saved in commandBuffer in the place "BufferCount"
//       BufferCount++;
//       if (BufferCount > 5) {
//         motor1Target = (commandBuffer[0]) * 256 + commandBuffer[1];
//         motor2Target = (commandBuffer[2]) * 256 + commandBuffer[3];
//         motor3Target = (commandBuffer[4]) * 256 + commandBuffer[5];
//         // Serial.print("Pitch: ");
//         // Serial.println(motor1Target);
//         // Serial.print("Roll: ");
//         // Serial.println(motor2Target);
//         // Serial.print("Yaw: ");
//         // Serial.println(motor3Target);
//         BufferCount = -1;  // Re-initialize BufferCount.
//         break;
//       }
//     }
//   }
// }

void DirectionManager(int Step1, int Step2, int Step3) {
  if (Step1 < 0) {  //DIR_PIN1 = 3
    digitalWrite(DIR_PIN1, LOW);
    // PORTD &= B11110111;        // This code is faster than ;
  } else {
    digitalWrite(DIR_PIN1, HIGH);  //
                                   // PORTD |= B00001000;
  }

  if (Step2 < 0) {
    digitalWrite(DIR_PIN2, LOW);
  } else {
    digitalWrite(DIR_PIN2, HIGH);
  }

  if (Step3 < 0) {
    digitalWrite(DIR_PIN3, LOW);
  } else {
    digitalWrite(DIR_PIN3, HIGH);
  }
  delayMicroseconds(5);
}

void LimitManager() {

  if (motor1Target > motor1Max) {
    motor1Target = motor1Max;
  }
  if (motor2Target > motor2Max) {
    motor2Target = motor2Max;
  }
  if (motor3Target > motor3Max) {
    motor3Target = motor3Max;
  }
}

void homingInterupt() {
  if (inInterupt) return;
  detachInterrupt(digitalPinToInterrupt(ENDSTOPS_INTERUPT_PIN));
  inInterupt = true;
  interrupts();
  for (int i = 0; i < 6; i++) {
    endstopsState[i] = endstops.digitalRead(i);
  }
  inInterupt = false;
  //printEndstopsState();
  attachInterrupt(digitalPinToInterrupt(ENDSTOPS_INTERUPT_PIN), homingInterupt, FALLING);
}

void printEndstopsState()
{
  for (int i = 0; i < 6; i++) {
    Serial.print(endstopsState[i]);
    Serial.print(", ");
  }
  Serial.println("");
}
void homing() {
  // Serial.print("Homing start: ");
  // Serial.println(homingState);
  pulseWidth = speedHoming;
  while (homingState < 6) {

    switch (homingState) {
      case 0:
          if (endstopsState[0] == 1) {
            digitalWrite(DIR_PIN1, LOW);
            monoPulse(STEP_PIN1);
          } else {
            motor1Position = 0;
            homingState++;
          }
          //Serial.println("case 0");
          break;
      case 1:
          if (endstopsState[1] == 1) {
            digitalWrite(DIR_PIN1, HIGH);
            monoPulse(STEP_PIN1);
            motor1Position++;
          } else {
            motor1Max = motor1Position;
            motor1Target = (motor1Max / 2)-1;
            CommandWorker();
            MoveSteppers(stepPulses1, 0, 0);
            homingState++;
            pulseWidth = speedHoming;
            delay(100);
            digitalWrite(ENABLE_PIN1, LOW);
            delay(10);
            digitalWrite(ENABLE_PIN1, HIGH);
            delay(1000);
          }
          //Serial.println("case 1");
          break;
      case 2:
          if (endstopsState[2] == 1) {
            digitalWrite(DIR_PIN2, LOW);
            monoPulse(STEP_PIN2);
          } else {
            motor2Position = 0;
            homingState++;
          }
          //Serial.println("case 2");
          break;
      case 3:
          if (endstopsState[3] == 1) {
            digitalWrite(DIR_PIN2, HIGH);
            monoPulse(STEP_PIN2);
            motor2Position++;
          } else {
            motor2Max = motor2Position;
            motor2Target = (motor2Max / 2) - 1;
            CommandWorker();
            MoveSteppers(0, stepPulses2, 0);
            homingState++;
            pulseWidth = speedHoming;
            delay(100);
            digitalWrite(ENABLE_PIN2, LOW);
            delay(10);
            digitalWrite(ENABLE_PIN2, HIGH);
            delay(1000);
          }
          //Serial.println("case 3");
          break;
      case 4:
          if (endstopsState[4] == 1) {
            digitalWrite(DIR_PIN3, LOW);
            monoPulse(STEP_PIN3);
          } else {
            motor3Position = 0;
            homingState++;
          }
          //Serial.println("case 4");
          break;
      case 5:
          if (endstopsState[5] == 1) {
            digitalWrite(DIR_PIN3, HIGH);
            monoPulse(STEP_PIN3);
            motor3Position++;
          } else {
            motor3Max = motor3Position;
            motor3Target = (motor3Max / 2) - 1;
            CommandWorker();
            MoveSteppers(0, 0, stepPulses3);
            homingState++;
            delay(100);
            digitalWrite(ENABLE_PIN3, LOW);
            delay(10);
            digitalWrite(ENABLE_PIN3, HIGH);
            delay(1000);
          }
          //Serial.println("case 5");
          break;
    }
  }
  // Serial.print("Homing end: ");
  // Serial.println(homingState);
  pulseWidth = speedNormal;
  // Serial.println(motor1Max);
  // Serial.println(motor2Max);
  // Serial.println(motor3Max);
}

void shutdown() {
  // shutdown trigger centering, after centering it disables motors
    if (inShutdown) return;
    if(digitalRead(SHUTDOWN_INTERUPT_PIN) == 0)
    {
      inShutdown = true;
      digitalWrite(ENABLE_PIN1, LOW);
      digitalWrite(ENABLE_PIN2, LOW);
      digitalWrite(ENABLE_PIN3, LOW);
      //Serial.println("Shutdown");
    }
  // if (inShutdown && digitalRead(SHUTDOWN_INTERUPT_PIN) == LOW)
  //   {
  //     Serial.println("Shutdown 1");
  //     return;
  //   }
  // if (inShutdown && digitalRead(SHUTDOWN_INTERUPT_PIN) == HIGH && homingState >= 6)
  // {
  //   inShutdown = false;
  //   digitalWrite(ENABLE_PIN1, HIGH); // enable motors
  //   digitalWrite(ENABLE_PIN2, HIGH);
  //   digitalWrite(ENABLE_PIN3, HIGH);
  //   Serial.println("Shutdown 2");
  //   return;
  // } 

  // if (inShutdown && digitalRead(SHUTDOWN_INTERUPT_PIN) == LOW && homingState < 6)
  // {
  //   Serial.println("Shutdown 3");
  //   inShutdown = true;
  //   digitalWrite(ENABLE_PIN1, LOW);
  //   digitalWrite(ENABLE_PIN2, LOW);
  //   digitalWrite(ENABLE_PIN3, LOW);
  //   return;
  // } 
  
  // if (inShutdown && digitalRead(SHUTDOWN_INTERUPT_PIN) == LOW && homingState >= 6)
  // {
  //   Serial.println("Shutdown 4");
  //   inShutdown = true;
  //   pulseWidth = speedShutdown;
  //   motor1Target = motor1Max / 2;
  //   motor2Target = motor2Max / 2;
  //   motor3Target = motor3Max / 2;
  //   CommandWorker();
  //   MoveSteppers(stepPulses1, stepPulses2, stepPulses3);
  //   digitalWrite(ENABLE_PIN1, LOW);
  //   digitalWrite(ENABLE_PIN2, LOW);
  //   digitalWrite(ENABLE_PIN3, LOW);
  //   pulseWidth = speedNormal;
  //   return;
  // } 

}