#include <Arduino.h>
#include <PID_v1.h>
#include "ProcessController.h"
#include "SimpleController.h"

// _pxPID = new PID(&_imu->x, &_currentAileron, &_target_x, 1, 10,   1, REVERSE);
SimpleController::SimpleController(double* pInput, double *pOutput, double* setPoint, double p, double i, double d, int dir)
{
  _pInput = pInput;
  _pOutput = pOutput;
  _p = p;
  
  if( dir == REVERSE )
    _direction = -1;
  else
    _direction = 1;

  _setPoint = setPoint;
}

SimpleController::~SimpleController()
{
  
}

bool SimpleController::Compute()
{
  if( _mode == MANUAL )
    return false;
    
  if( _pInput == _setPoint )
    return false;

  double diff = *_setPoint - *_pInput;

  *_pOutput += diff * _p * _direction;

  // range checks
  *_pOutput = min( *_pOutput, _outputHigh );
  *_pOutput = max( *_pOutput, _outputLow );
  
  return true;
}

void SimpleController::SetOutputLimits( double low, double high )
{
  _outputLow = low;
  _outputHigh= high;
}

void SimpleController::SetMode( int mode )
{
  _mode = mode;
}

