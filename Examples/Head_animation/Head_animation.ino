/*
||
|| Credits: The used head is basically the head designed by James Bruton.
|| I have adjusted the eyes assemby in order to add eyes up and down movement. (see CAD map for .step file).
|| Please download the head.stp James Brutonfrom github as basis. 
|| Replace the eye assemblies with my assembly and enjoy.
|| 
|| For the control od the head with up-down eyes you need:
|| 1. The robot with 9 servo's
|| 2. Arduino Uno
|| 3. a dual joystick shield HU-M16 (meArm joystick shield)
|| 4. a PCA9685 board (I2C 16 servo-driver)
|| PS1: On the PCA9685 are Vcc, Gnd and A4 and A5 available.
||
|| How to use program:
|| Programm starts in mode 1. (Red led on joystick board lights full)
|| When in mode 1 you can change to mode 2 by pressing  the right joystick button.
|| In mode 2 the red led on joystick board is dimmed.
|| Pressing the right joystick button again brings you back to mode 1.
|| By pressing the left joystick button (interrupt based) the programm comes in mode 3 (red light off).
|| By pressing the left joystick button again the program wil finish te animation and returns to mode 1.
||
|| Mode 1: manual mode eyes-control (Red led on joystick board lights full)
||         Left joystick controls eyemovement left-right and up-down
||         Right joystick controls eye lashes left and right
|| Mode 2: manual mode base-control (Red led on joystick board is dimmed)
||         Left joystick controls left-right and base left-up-down
||         Right joystick controls base right-up-down
|| Mode 3: animation mode (Red led om joystick board is off)
||
*/

#include <Wire.h>
#include <Controlled-PWMServoDriver-I2C.h>

// connection joysticks
#define joystick1X_Pin      A0 // x
#define joystick1Y_Pin      A1 // y
#define joystick1Button_Pin  2 // Push button

#define joystick2X_Pin      A3 // x
#define joystick2Y_Pin      A2 // y
#define joystick2Button_Pin  4 // Push button

#define potMidden  512   // midposition joystick

#define led_Pin      3
#define wacht        0

// Motor direction
#define cw        true
#define ccw      false

// Sequencenumbers on PCA9685 board
#define idxEyeLRLe   0  
#define idxEyeLRRi   1
#define idxEyeHLLe   2
#define idxEyeHLRi   3
#define idxEyelashLe 4
#define idxEyelashRi 5
#define idxBaseUpLe  6
#define idxBaseUpRi  7
#define idxBase      8


Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver();  // Default I2C adres: 0x40

ControlledServos servoEyeLRLe;
ControlledServos servoEyeLRRi; 
ControlledServos servoEyeHLLe; 
ControlledServos servoEyeHLRi; 
ControlledServos servoEyelashLe;
ControlledServos servoEyelashRi;
ControlledServos servoBaseUpLe; 
ControlledServos servoBaseUpRi; 
ControlledServos servoBase;

bool manualControl = true;
bool stopLoop = false;
bool startProcedure = true;
bool eyeControl = true;   // vs Base control

void button1Pressed() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  
  if (interrupt_time - last_interrupt_time > 200)
    manualControl = !manualControl;
  
  if (manualControl) {
    digitalWrite(led_Pin, HIGH);
    startProcedure = true;
    eyeControl = true;
  } else {
    digitalWrite(led_Pin, LOW);  
  }
  last_interrupt_time = interrupt_time;
}

void moveAll() {
    while (moving())
      update();
}      

/*
|| Checks if any joint is moving.
*/
boolean moving() {
  boolean result = false;
  result |= servoEyeLRLe.moving();
  result |= servoEyeLRRi.moving();
  result |= servoEyeHLLe.moving();
  result |= servoEyeHLRi.moving();
  result |= servoEyelashLe.moving();
  result |= servoEyelashRi.moving();
  result |= servoBaseUpLe.moving();
  result |= servoBaseUpRi.moving();
  result |= servoBase.moving();

  return result;
}

/*
|| Updates all servos. Can be used to asynchronously control the servos.
*/
bool update() {
  bool processing = false;
  processing |= servoEyeLRLe.update();
  processing |= servoEyeLRRi.update();
  processing |= servoEyeHLLe.update();
  processing |= servoEyeHLRi.update();
  processing |= servoEyelashLe.update();
  processing |= servoEyelashRi.update();
  processing |= servoBaseUpLe.update();
  processing |= servoBaseUpRi.update();
  processing |= servoBase.update();

  return processing;
}

void rollEyes(bool rotationDir) {
  int8_t x1;
  int8_t y1;
  int8_t rotDir = rotationDir ? 1 : -1;

  for (uint16_t angle = 0; angle <= 360; angle += 15) {
    x1 = 90 + 30 * cos(PI * angle/180 );
    y1 = 90 - rotDir * 20 * sin(PI * angle/180);
    setAll(x1, x1, y1, y1, 90, 90, 90, 90, 90);
    moveAll(); 
  }
}

void blinkEye(ControlledServos &aServo){
    uint8_t orgRate = aServo.getRate();

    aServo.setRate(2);
    aServo.moveTo(180);
    while (aServo.moving())
      aServo.update();
      
    aServo.moveTo(90);
    while (aServo.moving())
      aServo.update();
 
    aServo.setRate(orgRate);
}

void verplaatsServo(ControlledServos &aServo, int aHoek) {
  aServo.moveTo(aHoek);
}  

void setAll(uint8_t servoEyeLRLeAngle, uint8_t servoEyeLRRiAngle, uint8_t servoEyeHLLeAngle,
            uint8_t servoEyeHLRiAngle, uint8_t servoEyelashLeAngle, uint8_t servoEyelashRiAngle,
            uint8_t servoBaseUpLeAngle, uint8_t servoBaseUpRiAngle, uint8_t servoBaseAngle)
{
  verplaatsServo(servoEyeLRLe, servoEyeLRLeAngle);
  verplaatsServo(servoEyeLRRi, servoEyeLRRiAngle);
  verplaatsServo(servoEyeHLLe, servoEyeHLLeAngle);
  verplaatsServo(servoEyeHLRi, servoEyeHLRiAngle);

  verplaatsServo(servoEyelashLe, servoEyelashLeAngle);
  verplaatsServo(servoEyelashRi, servoEyelashRiAngle);

  verplaatsServo(servoBaseUpLe, servoBaseUpLeAngle);
  verplaatsServo(servoBaseUpRi, servoBaseUpRiAngle);
  verplaatsServo(servoBase, servoBaseAngle);
}

void setSpeed(uint16_t servoEyeLRLeSpeed, uint16_t servoEyeLRRiSpeed, uint16_t servoEyeHLLeSpeed,
              uint16_t servoEyeHLRiSpeed, uint16_t servoEyelashLeSpeed, uint16_t servoEyelashRiSpeed,
              uint16_t servoBaseUpLeSpeed, uint16_t servoBaseUpRiSpeed, uint16_t servoBaseSpeed) 
{
  servoEyeLRLe.setRate(servoEyeLRLeSpeed);   // intern: Hoeksnelheid is in msPerDegree
  servoEyeLRRi.setRate(servoEyeLRRiSpeed);
  servoEyeHLLe.setRate(servoEyeHLLeSpeed);
  servoEyeHLRi.setRate(servoEyeHLRiSpeed);

  servoEyelashLe.setRate(servoEyelashLeSpeed);
  servoEyelashRi.setRate(servoEyelashRiSpeed);

  servoBaseUpLe.setRate(servoBaseUpLeSpeed);
  servoBaseUpRi.setRate(servoBaseUpRiSpeed);
  servoBase.setRate(servoBaseSpeed);
}

int VerwerkPositie(ControlledServos &aServo, uint16_t joystickPin) {
  int16_t joystickPos;
  int8_t servoPos = aServo.getAngle();

  joystickPos = analogRead(joystickPin);
  if (abs(joystickPos - potMidden) > 400) {
    if (joystickPos > potMidden) 
      servoPos -= 1;
    else 
      servoPos += 1;
    aServo.moveTo(servoPos);  
  }
}

int VerwerkPositie(ControlledServos &aServo1, ControlledServos &aServo2, uint16_t joystickPin) {
  VerwerkPositie(aServo1, joystickPin);
  VerwerkPositie(aServo2, joystickPin);
}

void setup() {

#ifdef SERIAL  
  Serial.begin(9600);
  while (!Serial);
#endif
  const long pi = 4 * atan(1);

  pwm0.begin();
  pwm0.setPWMFreq(50);

  servoEyeLRLe.initServo (pwm0, idxEyeLRLe, true);
  servoEyeLRRi.initServo (pwm0, idxEyeLRRi, true);
  servoEyeHLLe.initServo (pwm0, idxEyeHLLe, true);
  servoEyeHLRi.initServo (pwm0, idxEyeHLRi, true);

  servoEyelashLe.initServo(pwm0, idxEyelashLe, true);
  servoEyelashRi.initServo(pwm0, idxEyelashRi, true);

  servoBaseUpLe.initServo  (pwm0, idxBaseUpLe, true);
  servoBaseUpRi.initServo  (pwm0, idxBaseUpRi, true);
  servoBase.initServo   (pwm0, idxBase, true);

  delay(1000);  // in order to process the complete init procedure

  pinMode(joystick1Button_Pin, INPUT_PULLUP);
  pinMode(joystick2Button_Pin, INPUT_PULLUP);
  pinMode(led_Pin, OUTPUT);
  digitalWrite(led_Pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(joystick1Button_Pin), button1Pressed, LOW);

  servoEyeLRLe.setMinMaxDirTrim (60, 120, ccw);
  servoEyeLRRi.setMinMaxDirTrim (60, 120, ccw, -6);  
  servoEyeHLLe.setMinMaxDirTrim (70, 110, cw);
  servoEyeHLRi.setMinMaxDirTrim (70, 110, ccw, -4);

  servoEyelashLe.setMinMaxDirTrim(90, 135,  cw, 10);
  servoEyelashRi.setMinMaxDirTrim(30, 90,  ccw, 7);

  servoBaseUpLe.setMinMaxDirTrim  (40, 140,  cw);
  servoBaseUpRi.setMinMaxDirTrim  (40, 140, ccw, -8);
  servoBase.setMinMaxDirTrim   (30, 150,  cw);

  setSpeed(4, 4, 4, 4, 5, 5, 7, 7, 5);  // deg per s
  setAll(90, 90, 90, 90, 90, 90, 90, 90, 90);
  moveAll();
}

void loop() {
  if (manualControl) {
    if (startProcedure)
      setSpeed(5, 5, 5, 5, 5, 5, 7, 7, 7);  
      
    if (digitalRead(joystick2Button_Pin) == LOW) {
       eyeControl = !eyeControl;
       if (!eyeControl) 
         analogWrite(led_Pin, 63);
       else
         digitalWrite(led_Pin, HIGH);
       delay(250);
    }

    if (eyeControl) {
      VerwerkPositie(servoEyeLRLe, servoEyeLRRi, joystick1X_Pin);
      VerwerkPositie(servoEyeHLLe, servoEyeHLRi, joystick1Y_Pin);
      VerwerkPositie(servoEyelashLe, joystick2X_Pin);
      VerwerkPositie(servoEyelashRi, joystick2Y_Pin);
    } else {
      VerwerkPositie(servoBaseUpLe, joystick1Y_Pin);
      VerwerkPositie(servoBaseUpRi, joystick2Y_Pin);
      VerwerkPositie(servoBase, joystick1X_Pin);
    }
    moveAll(); 

  } else {
    setAll(0, 0, 90, 90, 90, 90, 90, 90, 90);
    moveAll();

    setAll(180, 180, 90, 90, 90, 90, 90, 90, 90);
    moveAll(); 
    setAll(90, 90, 90, 90, 90, 90, 90, 90, 90);
    moveAll(); 

    setAll(90, 90, 180, 180, 90, 90, 90, 90, 90);
    moveAll(); 
    setAll(90, 90, 90, 90, 90, 90, 90, 90, 90);
    moveAll(); 

    setAll(90, 90, 90, 90, 120, 60, 90, 90, 90);
    moveAll(); 

    setAll(90, 90, 90, 90, 120, 60, 50, 130, 30);
    moveAll(); 

    setAll(90, 90, 90, 90, 110, 120, 130, 30, 150);
    moveAll(); 

    setAll(90, 90, 90, 90, 90, 90, 90, 90, 150);
    moveAll(); 

    blinkEye(servoEyelashLe);
    delay(500);
    blinkEye(servoEyelashRi);
    delay(500);

    setAll(90, 90, 90, 90, 90, 90, 90, 90, 90);
    moveAll(); 

    rollEyes(cw);
    rollEyes(ccw);

    setAll(90, 90, 90, 90, 90, 90, 90, 90, 90);
    moveAll(); 

    delay(2000);
  }
}
