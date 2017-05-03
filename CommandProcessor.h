class Command
{
public:
  int Value;
public:
  Command()
  {
    CommandType = None;
  }

  enum CommandType
  {
    None,
    Throttle,
    Yaw,
    Status,
    Voltage,
    Roll,
    Pitch,
    NavigationOnOff,
    SetHome,
    Echo
  };
  enum CommandValueType
  {
    Relative,
    Absolute
  } 
  ;

  CommandType CommandType ;
  CommandValueType ChangeType;
public:
  int GetValue()
  {
    return Value;
  }
};



class CommandProcessor
{
  private:
    Command _command;
  protected:
    unsigned long _lastCommTime;
  public:
    CommandProcessor() ;

    virtual void Begin() = 0;
    virtual int GetChar() = 0;
    virtual int Available() = 0;
    virtual void PrintLine( const char * szLine ) = 0;
    virtual void Print( const char* szChars ) = 0;
    virtual void Print( int num) = 0;
    virtual void PrintLine( int num ) = 0;

    unsigned long GetLastCommTime();
    int ReadNum();
    int GetCharBlocking();
    void DumpIMU( IMU& imu, double targetX, double targetY, double targetZ );
    void DumpThrottle(int t);
    void DumpTailRotor( int tail, int targetZ );
    virtual Command& GetCommand();
    void DumpCollective( double collective );
};

