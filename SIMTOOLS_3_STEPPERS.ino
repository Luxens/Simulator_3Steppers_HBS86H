// Stepper microstepping settings :  32000

#include <Wire.h>
#include "PCF8574.h"
//PORTD
#define ENDSTOPS_INTERUPT_PIN 2
#define SHUTDOWN_INTERUPT_PIN 3
#define STEP_PIN1 4
#define DIR_PIN1 5
#define ENABLE_PIN1 6
#define STEP_PIN2 7
//PORTB
#define DIR_PIN2 8
#define ENABLE_PIN2 9
#define STEP_PIN3 10
#define DIR_PIN3 11
#define ENABLE_PIN3 12

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
const uint16_t stepperPulses = 4096;
const uint16_t SimToolsPulses = 65535;
const uint16_t stepperMax = stepperPulses-1;
const uint16_t stepperMid = stepperPulses/2 -1;
uint16_t motor1Target = stepperMid, motor2Target = stepperMid, motor3Target = stepperMid;        // Motors position we want to reach
uint16_t motor1Position = stepperMid, motor2Position = stepperMid, motor3Position = stepperMid;  // The actual positions
uint16_t motor1Max = stepperMax, motor2Max = stepperMax, motor3Max = stepperMax;

const uint16_t speedNormal = 150;    //us for steps delay 10 minimum //15
const uint16_t speedHoming = 500;   //us for steps delay
const uint16_t speedShutdown = 500;  //us for steps delay
const uint16_t speedStart = 300; //55
const uint16_t ramp_pulses = uint16_t (0.05*stepperPulses); //0.1
uint16_t pulseWidth = speedNormal;     //From the HBS86h datasheet : For reliable response, pulse width should be longer than 10μs
uint16_t current_x_speed = speedStart;
uint16_t current_y_speed = speedStart;
uint16_t current_z_speed = speedStart;
uint16_t current_x = 0;
uint16_t current_y = 0;
uint16_t current_z = 0;
uint16_t x, y, z;
bool prevDir1 = 1;
bool prevDir2 = 1;
bool prevDir3 = 1;
uint16_t xRamp = 0;
uint16_t yRamp = 0;
uint16_t zRamp = 0;
bool autoHoming = true;
uint8_t homingState = 0;
 
void setup() {
  delay(2000);  //safety delay to flash the code
  DDRD |=0b11110000; //set outputs/inputs
  DDRB |=0b00011111; //set default state
  DDRC &= ~0b00000111; //set outputs/inputs
  PORTC |= 0b00000111; //set default state
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

  PORTD |= 1<<ENABLE_PIN1;
  PORTB |= (1<<(ENABLE_PIN2-8) | 1<<(ENABLE_PIN3-8));

  Serial.begin(115200);  //To communicate with Simtools
  if (autoHoming) {
    homing();
  }
}

void loop() {
  if(Serial.available())
  {
    SerialReader();  //Get the datas from Simtools
  }
  TargetDelta();                                      //Convert the position targets to pulse number
  x = abs(stepPulses1);
  y = abs(stepPulses2);
  z = abs(stepPulses3);
  current_x = 0;
  current_y = 0;
  current_z = 0;
  DirectionManager(stepPulses1, stepPulses2, stepPulses3);  //put the motors in the right directions
  while(!Serial.available())
  {
    if(current_x < x)
    {
      pulseWidth = current_x_speed;
      monoPulse(STEP_PIN1);
      current_x ++;
      current_x_speed = speedNormal + speedStart*(1 - constrain(xRamp/ramp_pulses, 0, 1));
      xRamp ++;
      if(stepPulses1 > 0)
      {
        motor1Position ++;
      }
      else
      {
        motor1Position --;
      }
    }
    
    if(current_y < y)
    {
      pulseWidth = current_y_speed;
      monoPulse(STEP_PIN2);
      current_y ++;
      current_y_speed = speedNormal + speedStart*(1 - constrain(yRamp/ramp_pulses, 0, 1));
      yRamp ++;
      if(stepPulses2 > 0)
      {
        motor2Position ++;
      }
      else
      {
        motor2Position --;
      }
    }
    if(current_z < z)
    {
      pulseWidth = current_z_speed;
      monoPulse(STEP_PIN3);
      current_z ++;
      current_z_speed = speedNormal + speedStart*(1 - constrain(zRamp/(ramp_pulses*2), 0, 1));
      zRamp ++;
      if(stepPulses3 > 0)
      {
        motor3Position ++;
      }
      else
      {
        motor3Position --;
      }
    }
    
    
    }
    if(x <= 1)
    {
      xRamp = 0;
    }
    if(y <= 1)
    {
      yRamp = 0;
    }
    if(z <= 1)
    {
      zRamp = 0;
    }
    delayMicroseconds(2137 - 420); //dont know why but delay is necessary here
 }

void DirectionManager(int Step1, int Step2, int Step3) {
  if (Step1 < 0) {
    PORTD &= ~(1<<DIR_PIN1);
  } else {
    PORTD |= 1<<DIR_PIN1;
  }

  if (Step2 < 0) {
    PORTB &= ~(1<<(DIR_PIN2-8));
  } else {
    PORTB |= 1<<(DIR_PIN2-8);
  }

  if (Step3 < 0) {
    PORTB &= ~(1<<(DIR_PIN3-8));
  } else {
    PORTB |= 1<<(DIR_PIN3-8);
  }

  if((prevDir1 && (stepPulses1 < 0)) || (!prevDir1 && (stepPulses1 > 0)))
  {
    current_x_speed = speedNormal + speedStart;
    prevDir1 = !prevDir1;
    xRamp = 0;
  }
  if((prevDir2 && (stepPulses2 < 0)) || (!prevDir2 && (stepPulses2 > 0)))
  {
    current_y_speed = speedNormal + speedStart;
    prevDir2 = !prevDir2;
    yRamp = 0;
  }
  if((prevDir3 && (stepPulses3 < 0)) || (!prevDir3 and (stepPulses3 > 0)))
  {
    current_z_speed = speedNormal + speedStart;
    prevDir3 = !prevDir3;
    zRamp = 0;
  }
  //delayMicroseconds(5);
}

void TargetDelta() {
  Limiter();
  stepPulses1 = motor1Target - motor1Position;
  stepPulses2 = motor2Target - motor2Position;
  stepPulses3 = motor3Target - motor3Position;
}

void monoPulse(int StepPin) {
  if(StepPin<8)
  {
    PORTD |= 1<<StepPin;
    delayMicroseconds(pulseWidth);
    PORTD &= ~(1<<StepPin);
    delayMicroseconds(pulseWidth);
  }
  else 
  {
    PORTB |= 1<<(StepPin-8);
    delayMicroseconds(pulseWidth);
    PORTB &= ~(1<<(StepPin-8));
    delayMicroseconds(pulseWidth);
  }
}

void SerialReader() {
  while(Serial.available() > 0) {
    if (findString(startMarker, startMarkerLength)) { // Find START marker
      Serial.readBytes(axis1, dataLength);
      Serial.readBytes(axis2, dataLength);
      Serial.readBytes(axis3, dataLength);
      if (findString(stopMarker, stopMarkerLength)) {
        // Decode data
        motor1Target = (axis1[0] << 8) | axis1[1];
        motor2Target = (axis2[0] << 8) | axis2[1];
        motor3Target = (axis3[0] << 8) | axis3[1];
        motor1Target = constrain(motor1Target, 0, SimToolsPulses);
        motor2Target = constrain(motor2Target, 0, SimToolsPulses);
        motor3Target = constrain(motor3Target, 0, SimToolsPulses);
        motor1Target = map(motor1Target, 0, SimToolsPulses, 0, motor1Max);
        motor2Target = map(motor2Target, 0, SimToolsPulses, 0, motor2Max);
        motor3Target = map(motor3Target, 0, SimToolsPulses, 0, motor3Max);
      }
    }
  }
}

// void SerialReader() {
//   while(Serial.available() > 0) {
//     if (findString(startMarker, startMarkerLength)) { // Find START marker
//       Serial.readBytes(axis1, dataLength);
//       Serial.readBytes(axis2, dataLength);
//       Serial.readBytes(axis3, dataLength);
//       if (findString(stopMarker, stopMarkerLength)) {
//         // Decode data
//         motor1Target = (axis1[0] << 8) | axis1[1];
//         motor2Target = (axis2[0] << 8) | axis2[1];
//         motor3Target = (axis3[0] << 8) | axis3[1];
//         motor1Target = constrain(motor1Target, 0, SimToolsPulses);
//         motor2Target = constrain(motor2Target, 0, SimToolsPulses);
//         motor3Target = constrain(motor3Target, 0, SimToolsPulses);
//         motor1Target = map(motor1Target, 0, SimToolsPulses, 0, motor1Max);
//         motor2Target = map(motor2Target, 0, SimToolsPulses, 0, motor2Max);
//         motor3Target = map(motor3Target, 0, SimToolsPulses, 0, motor3Max);
//       }
//     }
//   }
// }

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
      index = 0; // Index reset
    }
  }
  return false;
}

void Limiter() {

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

void MoveSteppers(int Step1, int Step2, int Step3) 
{
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

  while(x > current_x || y > current_y || z > current_z)
  {   
    
    if(current_x < x)
    {
      pulseWidth = current_x_speed;
      monoPulse(STEP_PIN1);
      current_x ++;
      current_x_speed = speedHoming + speedStart*(1 - constrain(current_x/ramp_pulses, 0, 1));
    }
    
    if(current_y < y)
    {
      pulseWidth = current_y_speed;
      monoPulse(STEP_PIN2);
      current_y ++;
      current_y_speed = speedHoming + speedStart*(1 - constrain(current_y/ramp_pulses, 0, 1));
    }
    if(current_z < z)
    {
      pulseWidth = current_z_speed;
      monoPulse(STEP_PIN3);
      current_z ++;
      current_z_speed = speedHoming + speedStart*(1 - constrain(current_z/(ramp_pulses*1), 0, 1));
    }
    
  }

  motor1Position = motor1Target;
  motor2Position = motor2Target;
  motor3Position = motor3Target;
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
  pulseWidth = speedHoming;
  while (homingState < 6) {

    switch (homingState) {
      case 0:
          if (endstopsState[0] == 1) {
            PORTD &= ~(1<<DIR_PIN1);
            monoPulse(STEP_PIN1);
          } else {
            motor1Position = 0;
            homingState++;
          }
          break;
      case 1:
          if (endstopsState[1] == 1) {
            PORTD |= 1<<DIR_PIN1;
            monoPulse(STEP_PIN1);
            motor1Position++;
          } else {
            motor1Max = motor1Position;
            motor1Target = (motor1Max / 2)-1;
            TargetDelta();
            MoveSteppers(stepPulses1, 0, 0);
            homingState++;
            pulseWidth = speedHoming;
            delay(30);
            PORTD &= ~(1<<ENABLE_PIN1);
            delay(3);
            PORTD |= 1<<ENABLE_PIN1;
            delay(300);
          }
          break;
      case 2:
          if (endstopsState[2] == 1) {
            PORTB &= ~(1<<(DIR_PIN2-8));
            monoPulse(STEP_PIN2);
          } else {
            motor2Position = 0;
            homingState++;
          }
          break;
      case 3:
          if (endstopsState[3] == 1) {
            PORTB |= 1<<(DIR_PIN2-8);
            monoPulse(STEP_PIN2);
            motor2Position++;
          } else {
            motor2Max = motor2Position;
            motor2Target = (motor2Max / 2) - 1;
            TargetDelta();
            MoveSteppers(0, stepPulses2, 0);
            homingState++;
            pulseWidth = speedHoming;
            delay(30);
            PORTB &= ~(1<<(ENABLE_PIN2-8));
            delay(3);
            PORTB |= 1<<(ENABLE_PIN2-8);
            delay(300);
          }
          break;
      case 4:
          if (endstopsState[4] == 1) {
            PORTB &= ~(1<<(DIR_PIN3-8));
            monoPulse(STEP_PIN3);
          } else {
            motor3Position = 0;
            homingState++;
          }
          break;
      case 5:
          if (endstopsState[5] == 1) {
            PORTB |= 1<<(DIR_PIN3-8);
            monoPulse(STEP_PIN3);
            motor3Position++;
          } else {
            motor3Max = motor3Position;
            motor3Target = (motor3Max / 2) - 1;
            TargetDelta();
            MoveSteppers(0, 0, stepPulses3);
            homingState++;
            delay(30);
            PORTB &= ~(1<<(ENABLE_PIN3-8));
            delay(3);
            PORTB |= 1<<(ENABLE_PIN3-8);
            delay(300);
          }
          break;
    }
  }
  pulseWidth = speedNormal;
}

void shutdown() {
  // shutdown trigger centering, after centering it disables motors
    if (inShutdown) return;
    if(digitalRead(SHUTDOWN_INTERUPT_PIN) == 0)
    {
      inShutdown = true;
      PORTD &= ~(1<<ENABLE_PIN1);
      PORTB &= ~(1<<(ENABLE_PIN2-8) | 1<<(ENABLE_PIN3-8));
    }
}