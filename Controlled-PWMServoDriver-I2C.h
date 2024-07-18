/*
|| Credits: Library depends heavily on controlledServo library by Brett Hagman.
|| 
|| This library controlles individual servo's connected to the I2C PCA9685 board and uses
|| the Adafruit_PWMServoDriver for the physical connection
|| 
|| The control internally is based on the pulselength, but the interface is in degrees.
|| Major controls:
|| 1. Speed per servo
|| 2. Offset from middle position in degrees (trim)
|| 3. Rotation direction (cw or ccw) for easy control in programm.
|| 4. The library is expected to control multiple PCA9685 boards, but that is not tested.
||
|| @2
||   The offset (trim) allows us to control servo's as if the middle is always 90 deg.
|| @3
||   In the enclosed CAD-head is the eyelash from left en right eye mirrored, but
||   by specifying cw en ccw for the servo's we can control them equally.
*/

#include <arduino.h>
#include <Adafruit_PWMServoDriver.h>

#define pulseMin 120  //   0 degree
#define pulseMid 310  //  90 degree
#define pulseMax 500  // 180 degree

// Motor rotation direction
#define cw   true
#define ccw  false

class ControlledServos {
  public:
// Instantiation
    ControlledServos();
    initServo(Adafruit_PWMServoDriver &pwm, uint8_t servoNum, bool moveServo = false);
	
// Servo setttings  
    // Sets the target angle.
    void setAngle(uint8_t angle);
    // Gets the current angle that the servo is set to.
    uint8_t getAngle();
    // Gets the current target angle.
    uint8_t getTargetAngle();
    void setTrim(int8_t trim);
    void setDir(bool dir);
    int8_t getTrim();

    // Sets the limits and trim for the servo
    void setMin(uint8_t minAngle);
    void setMax(uint8_t maxAngle);
    void setMid(uint8_t midAngle);
    void setMinMaxDirTrim(uint8_t maxAngle, uint8_t minAngle, bool dir, int8_t trim = 0);

// Move functions
	  // Executes a move. 
    void move(boolean blocking = false);
    // Sets the next angle to move to, then executes.
    void moveTo(uint8_t angle, boolean blocking = false);	
    // Sets the next angle to move and move.
    void moveToNow(uint8_t angle);
  
    // Checks if the servo is still moving.
    boolean moving();

// Speed settings
    // Sets the rate at which the servo turns (in milliseconds per degree)
    void setRate(uint16_t rate);
    // Gets the current rate.
    uint16_t getRate();

// asynchronous operation
    // For asynchronous operation, this is called to update the servo position.
    bool update();
	
  private:
    Adafruit_PWMServoDriver *_pwm;
    uint8_t _servoNum;
    uint16_t _currentPulse;
    uint16_t _targetPulse;
    uint16_t _msPerDegree;
    uint16_t _minPulse;
    uint16_t _maxPulse;
    int16_t _trimPulse;
    bool _direction;
    uint32_t _lastUpdateTime;
};
