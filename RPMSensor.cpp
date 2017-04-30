#include "Arduino.h"
#include "RPMSensor.h"

RPMSensor::RPMSensor()
{
  _currentRPM = 0;
  pinMode(PIN_RPM, INPUT);
  _lastHall = digitalRead(PIN_RPM);
  _lastSwitch = millis();
}

int RPMSensor::CheckAndGetRPM()
{
  int currentHall = digitalRead(PIN_RPM);
  if ( currentHall == _lastHall )
    return _currentRPM;
  unsigned int currentTime = millis();

  unsigned int span = currentTime - _lastSwitch;
  _currentRPM = span * 30000;

  _lastHall = currentHall;
  _lastSwitch = currentTime;

  return _currentRPM;
}

