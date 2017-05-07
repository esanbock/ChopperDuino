#include "Arduino.h"
#include "IMU.h"

IMU::IMU( )
{
  x = 0;
  y = 0;
  z = 0;
  _haveChange = false;
}

IMU::~IMU()
{

}

void IMU::ReadValues()
{
  _haveChange = false;
  int rx = analogRead(PIN_IMU_X);
  if( rx != x )
  {
      x = rx;
      _haveChange = true;
  }
  
  int ry = analogRead(PIN_IMU_Y);
  if( ry != y )
  {
    y = ry;
    _haveChange = true;
  }
  
  int rz = analogRead(PIN_IMU_Z);
  if( rz != z )
  {
    z = rz;
    _haveChange = true;
  }

  temp = analogRead(PIN_IMU_TEMP);
}

boolean IMU::HaveChange()
{
  return _haveChange;
}


