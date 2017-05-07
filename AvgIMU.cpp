#include "Arduino.h"
#include "IMU.h"
#include "AvgIMU.h"

double AvgIMU::avg(int* array, int count )
{
  double sum = 0;
  for ( int i = 0; i < count; i++ )
  {
    sum += array[i];
  }
  return sum / count;
}

double AvgIMU::movingAvg(double currentVal, int newVal, int count )
{
  return ( currentVal * count - currentVal + newVal ) / count;
}

AvgIMU::AvgIMU( )
{

}

AvgIMU::~AvgIMU()
{

}

void AvgIMU::ReadValues()
{
  _realX = analogRead(PIN_IMU_X);
  _realY = analogRead(PIN_IMU_Y);
  _realZ = analogRead(PIN_IMU_Z);

  x = movingAvg(x, _realX, AVGSIZE);
  y = movingAvg(y, _realY, AVGSIZE);
  z = movingAvg(z, _realZ, AVGSIZE);

  temp = analogRead(PIN_IMU_TEMP);
}

boolean AvgIMU::HaveChange()
{
  boolean change = false;

  if ( (int)x != _px )
  {
    _px = (int)x;
    change = true;
  }
  if ( (int) y != _py )
  {
    _py = (int)y;
    change = true;
  }
  if ( (int)z != _pz )
  {
    _pz = (int)z;
    change =  true;
  }
  return change;
}


