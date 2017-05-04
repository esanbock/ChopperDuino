class PidController : public virtual ProcessController, protected PID
{
public:
  PidController(double* pInput, double *pOutput, double* setPoint, double p, double i, double d, int dir)
    :PID(pInput, pOutput, setPoint, p, i, d, dir)
    {
      
    }

  virtual bool Compute()
  {
    return PID::Compute();
  }
  virtual void SetOutputLimits(double low, double high)
  {
    PID::SetOutputLimits( low, high );
  }
  virtual void SetMode( int mode )
  {
    PID::SetMode(mode);
  }
};

