#include <Controlled-PWMServoDriver-I2C.h>

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ControlledServos::ControlledServos() {
  _pwm = NULL;
  _servoNum = -1;
  _targetPulse = pulseMid;
  _currentPulse = pulseMid;
  _msPerDegree = 1;
  _minPulse = pulseMin;
  _maxPulse = pulseMax;
  _trimPulse = 0;
  _direction = cw;
  _lastUpdateTime = 0;
}

ControlledServos::initServo(Adafruit_PWMServoDriver &pwm, uint8_t servoNum, bool moveServo = false) {
  ControlledServos();
  _pwm = &pwm;
  _servoNum = servoNum;
  if (moveServo)
    _pwm->setPWM(_servoNum, 0, pulseMid);
}

/*
|| Sets the target pulse (angle [0-180] as input)
*/
void ControlledServos::setAngle(uint8_t angle) {
  if (_direction)
   _targetPulse = map(angle, 0, 180, pulseMin, pulseMax) + _trimPulse;
  else
   _targetPulse = map(angle, 180, 0, pulseMin, pulseMax) + _trimPulse;
  _targetPulse = constrain(_targetPulse, _minPulse, _maxPulse);
}

/*
|| Get current angle.
*/
uint8_t ControlledServos::getAngle() { 
  if (_direction)
    return round(mapfloat(_currentPulse - _trimPulse, pulseMin, pulseMax, 0, 180));
  else
    return round(mapfloat(_currentPulse - _trimPulse, pulseMin, pulseMax, 180, 0));
}  

uint8_t ControlledServos::getTargetAngle() { 
  return round(mapfloat(_targetPulse - _trimPulse, pulseMin, pulseMax, 0, 180));
}

/*
|| Sets the limits for the servo
*/
void ControlledServos::setMin(uint8_t minAngle) { 
  _minPulse = round(mapfloat(minAngle, 0, 180, pulseMin, pulseMax)) + _trimPulse;
  _minPulse = constrain(_minPulse, pulseMin, pulseMax);
}

void ControlledServos::setMax(uint8_t maxAngle) { 
  uint16_t tmpPulse = round(mapfloat(maxAngle, 0, 180, pulseMin, pulseMax)) + _trimPulse;
  _maxPulse = constrain(tmpPulse, pulseMin, pulseMax);
}

void ControlledServos::setDir(bool direction) { 
  _direction = direction;
}

void ControlledServos::setTrim(int8_t trim) { 
  _trimPulse = constrain(trim, -90, 90);
  _trimPulse = round(mapfloat(_trimPulse, -90, 90, -(pulseMax-pulseMin), pulseMax-pulseMin));
}

void ControlledServos::setMinMaxDirTrim(uint8_t minAngle, uint8_t maxAngle, bool dir, int8_t trimAngle = 0) { 
  setTrim(trimAngle);  // must be first as Min and Max are adjusted accordingly
  setMin(minAngle);
  setMax(maxAngle);
  setDir(dir);
}

int8_t ControlledServos::getTrim() { 
  return round(mapfloat(_trimPulse, -(pulseMax-pulseMin), pulseMax-pulseMin, -90, 90));
}    

/*
|| Executes a move.
*/
void ControlledServos::move(boolean blocking) {
  _lastUpdateTime = millis();
  // If blocking is true, let's wait here until the servo is at the target.
  if (blocking)
    while (update());
}

/*
|| Sets the next angle to move to, then executes.
*/
void ControlledServos::moveTo(uint8_t angle, boolean blocking) {
  setAngle(angle);  // angle is converted in setAngle
  move(blocking);
}

void ControlledServos::moveToNow(uint8_t angle) { 
  moveTo(angle, true);   // angle is converted to pulse in setAngle
}

/*
|| Checks if the servo is still moving.
*/
boolean ControlledServos::moving() {
  return _currentPulse != _targetPulse;
}

/*
|| Speed settings
*/
void ControlledServos::setRate(uint16_t rate) { 
  _msPerDegree = rate > 0 ? rate : 1; 
}

uint16_t ControlledServos::getRate() { 
  return _msPerDegree; 
}

/*
|| For asynchronous operation, this is called to update the servo position.
*/
bool ControlledServos::update() {
  // This is our processing state - i.e. do we still need to move the servo to the target?
  bool processing = false;
  uint32_t currentTime = millis();
//  uint16_t currentPulse = _pwm->getPWM(_servoNum, true);    // Slows libspeed down
  if (_currentPulse != _targetPulse) {
    // Don't bother processing unless we are ready to move at least 1 degree (i.e. _msPerDegree ms has passed).
    if (currentTime < (_lastUpdateTime + _msPerDegree)) {
      return true;
    }

    int16_t pulseDifference = _targetPulse - _currentPulse;
    int8_t direction = pulseDifference < 0 ? -1 : 1;
    uint16_t newPulse = _currentPulse + ((currentTime - _lastUpdateTime) / _msPerDegree) * direction;
    if (direction < 0) {
      if (newPulse < _targetPulse)
        newPulse = _targetPulse;
    } else {
      if (newPulse > _targetPulse)
        newPulse = _targetPulse;
    }   
    _currentPulse = constrain(newPulse, pulseMin, pulseMax);
    _pwm->setPWM(_servoNum, 0, _currentPulse);

    processing = true;
  }
  _lastUpdateTime = currentTime;
  return processing;
}
