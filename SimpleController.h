class SimpleController : public ProcessController
{
private:
  double _outputLow = 0;
  double _outputHigh = 255;
  int _direction;
  double* _setPoint;
  double _p;
  double* _pInput;
  double* _pOutput;
  int _mode;

public:
// _pxPID = new PID(&_imu->x, &_currentAileron, &_target_x, 1, 10,   1, REVERSE);
  SimpleController(double* pInput, double *pOutput, double* setPoint, double p, double i, double d, int dir);
  virtual bool Compute();
  virtual void SetOutputLimits( double low, double high );
  virtual void SetMode( int mode );
};

