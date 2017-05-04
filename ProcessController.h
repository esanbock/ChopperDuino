class ProcessController
{
public:
  virtual bool Compute()=0;
  virtual void SetOutputLimits(double low, double high)=0;
  virtual void SetMode( int mode )=0;
  virtual ~ProcessController(){};
};

