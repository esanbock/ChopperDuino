
#include <PWMServo.h>
#include <HardwareSerial.h>
#include <PID_v1.h>

static const int SERVO_MIN=70;
static const int SERVO_MAX=110;
static const int SERVO_STARTANGLE=90;
static const int TAILROTOR_MIN = 100;
static const int TAILROTOR_MAX = 254;
static const int THROTTLE_MAX = 255;

const int PIN_IMU_X = 20;
const int PIN_IMU_Y = 19;
const int PIN_IMU_Z = 18;
const int PIN_IMU_TEMP = 17;
const int PIN_THROTTLE = 10;
const int PIN_TAIL = 12;
const int PIN_SERVO_LEFT = SERVO_PIN_A;
const int PIN_SERVO_RIGHT = SERVO_PIN_B;
const int PIN_SERVO_ELEVATOR = SERVO_PIN_C;
const int PIN_VOLTAGE = 16;
const int MIN_VOLTAGE = 590;
const int PIN_LED = 11;
const int PIN_RX=7;
const int PIN_TX=8;

class IMU
{
public:
  double x;
  double y;
  double z;
  int temp;

private:
  static const int AVGSIZE = 40;
  int* _histX;
  int* _histY;
  int* _histZ;
  int _curHist;
  
  int realX;
  int realY;
  int realZ;

  int px;
  int py;
  int pz;

protected:

double avg(int* array, int count )
{
  double sum=0;
  for( int i=0; i < count; i++ )
  {
    sum += array[i];
  }
  return sum / count;
}

public:

  IMU( )
  { 
    _curHist = 0;
    _histX = new int[AVGSIZE];
    _histY = new int[AVGSIZE];
    _histZ = new int[AVGSIZE];

    memset(_histX,0,sizeof(int) * AVGSIZE);
    memset(_histY,0,sizeof(int) * AVGSIZE);
    memset(_histZ,0,sizeof(int) * AVGSIZE);
  }

  void ReadValues()
  {
    realX = analogRead(PIN_IMU_X);
    realY = analogRead(PIN_IMU_Y);
    realZ = analogRead(PIN_IMU_Z);
    
    _curHist++;
    if( _curHist >= AVGSIZE - 1 )
      _curHist = 0;
    
    _histX[_curHist] = realX;
    _histY[_curHist] = realY;
    _histZ[_curHist] = realZ;
    
     x = avg(_histX, AVGSIZE);
     y = avg(_histY, AVGSIZE);
     z = avg(_histZ, AVGSIZE);
    
    temp = analogRead(PIN_IMU_TEMP);
  }

  boolean HaveChange()
  {
    boolean change = false;

    if( (int)x != px )
    {
      px = (int)x;
      change = true;
    }
    if( (int) y!= py )
    {
      py = (int)y;
      change = true;
    }  
    if( (int)z != pz )
    {
      pz = (int)z;
      change =  true;
    }
    return change;
  }

  ~IMU()
  {
    delete _histX;
    delete _histY;
    delete _histZ;
  }

};

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
    Bank,
    Pitch,
    NavigationOnOff,
    SetHome,
    SetXYPerMs,
    SetZPerMs,
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
  CommandProcessor() 
  {

  }

  virtual void Begin()=0;
  virtual int GetChar()=0;
  virtual int Available()=0;
  virtual void PrintLine( const char * szLine )=0;
  virtual void Print( const char* szChars )=0;
  virtual void Print( int num)=0;
  virtual void PrintLine( int num )=0;
  
  unsigned long GetLastCommTime()
  {
    return _lastCommTime;
  }
  
  int ReadNum()
  {
      char num[5];
      num[3] = NULL;
      num[4] = NULL;
      num[0] = GetCharBlocking();
      num[1] = GetCharBlocking();
      num[2] = GetCharBlocking();
      if( num[0] == '-' )
        num[3] = GetCharBlocking();

      
      return atoi( num );
  }
  
  int GetCharBlocking()
  {
    while( Available() < 1 )
    {
      delay( 100 );
    }
    return GetChar();
  }

  void DumpIMU( IMU& imu, double targetX, double targetY, double targetZ )
  {
    Print(":S ");
    Print("x:");
    Print(imu.x);
    Print(" y:");
    Print(imu.y);
    Print(" z:");
    Print(imu.z);
/*    Print(" tx:");
    Print(targetX);
    Print(" ty:");
    Print(targetY);
    Print(" tz:");
    Print(targetZ);*/
    Print(" t:");
    PrintLine(imu.temp);
  }
  
  void DumpThrottle(int t, int zAdjusts)
  {
    Print( "Throttle=" );
    Print( t );
    Print( " ZadjPerms=" );
    PrintLine( zAdjusts );
  }
  
  
void DumpTailRotor( int tail, int targetZ )
{
    Print("_currentTailRotor=");
    PrintLine(tail);
  }

  
  void Log( const char * szLine )
  {
    Serial.print(szLine);
  }

  void Log( byte b )
  {
    Serial.print( b );
  }

  virtual Command& GetCommand()
  {
    while( Available() )
    {
      _lastCommTime = millis();
      byte b = GetChar();

      if( b == ':' )
      {
        int commandType = GetCharBlocking();
        byte val;
        
        switch( commandType )
        {
        case 'T':
          _command.CommandType = Command::Throttle;
          _command.Value = ReadNum();
          _command.ChangeType = Command::Absolute;
          return _command;
        case 'Y':
          _command.CommandType = Command::Yaw;
          _command.Value = ReadNum();
          _command.ChangeType = Command::Absolute;
          return _command;
        case 'S':
          _command.CommandType = Command::Status;
          return _command;
        case 'V':
          _command.CommandType = Command::Voltage;
          return _command;
        case 'B':
          _command.CommandType = Command::Bank;
          _command.Value = ReadNum();
          _command.ChangeType = Command::Absolute;
          return _command;
        case 'P':
          _command.CommandType = Command::Pitch;
          _command.Value = ReadNum();
          _command.ChangeType = Command::Absolute;
          return _command;
        case 'N':
          _command.CommandType = Command::NavigationOnOff;
          _command.Value = ReadNum();
          _command.ChangeType = Command::Absolute;
          if( _command.Value == 1 || _command.Value == 0 )
            return _command;
        case 'H':
          _command.CommandType = Command::SetHome;
          return _command;
        case 'X':
          _command.CommandType = Command::SetXYPerMs;
          _command.Value = ReadNum();
          _command.ChangeType = Command::Absolute;
          return _command;
        case 'Z':
          _command.CommandType = Command::SetZPerMs;
          _command.Value = ReadNum();
          _command.ChangeType = Command::Absolute;
          return _command;
        case 'E':
          _command.CommandType = Command::Echo;
          _command.Value = ReadNum();
          _command.ChangeType = Command::Absolute;
          return _command;
        default:
          PrintLine("NAK:");
          break;
        }
      }

    }
    _command.CommandType = Command::None;
    return _command;
  }
};


class SerialCommandProcessor : 
public CommandProcessor
{
private:
public:
  SerialCommandProcessor()
  {
  }
  void Begin()
  {
    Serial.begin(38400);
    PrintLine("start serial");
  }
  
  int GetChar()
  {
     return Serial.read();
  }
  
  int Available()
  {
    return Serial.available();
  }
  
  void PrintLine( const char * szLine )
  {
    Serial.println( szLine );
  }
  void PrintLine( int num )
  {
    Serial.println( num );
  }
  void Print( const char* szChars )
  {
    Serial.print( szChars );
  }
  
  void Print( double num )
  {
    Serial.print( num );
  }
  
  void Print( int num )
  {
    Serial.print( num );
  }

};

class UartCommandProcessor : 
public CommandProcessor
{
private:
  HardwareSerial _uart;
public:
  UartCommandProcessor()
  {
  }
  void Begin()
  {
    _uart.begin(38400);
    PrintLine("start uart");
  }
  
  int GetChar()
  {
     return _uart.read();
  }
  
  int Available()
  {
    return _uart.available();
  }
  
  void PrintLine( const char * szLine )
  {
    _uart.println( szLine );
  }
  void PrintLine( int num )
  {
    _uart.println( num );
  }
  void Print( const char* szChars )
  {
    _uart.print( szChars );
  }
  
  void Print( int num )
  {
    _uart.print( num );
  }

};

//SerialCommandProcessor serialCommandProcessor;
//CommandProcessor& commandProcessor = serialCommandProcessor;
UartCommandProcessor uartCommandProcessor;
CommandProcessor& commandProcessor = uartCommandProcessor;

class Navigator
{
private:
    IMU* _imu;
    PWMServo _leftServo;
    PWMServo _rightServo;
    PWMServo _elevatorServo;
    
    int _currentPitch;
    double _currentAileron;
    double _currentElevator;
    double _prevAileron;
    double _prevElevator;
    int _currentThrottle;

    double _prevTail;
    

    
    int _override_x;
    int _override_y;
    int _override_z;
    
    int _lastXYAdjust;
    int _lastZAdjust;
    unsigned long _xyAdjustsPerMs;
    
    //Define Variables we'll be connecting to

    //Specify the links and initial tuning parameters
    
    PID* _pxPID;
    PID* _pyPID;
    PID* _pzPID;
  
public:
    double _currentTailRotor;
    double _target_x;
    double _target_y;
    double _target_z;
    unsigned long _zAdjustsPerMs;
    
public:
    boolean NavigationEnabled; 
  void SetHome()
  {
    _imu->ReadValues();
    _target_x = _imu->x;
    _target_y = _imu->y;
    _target_z = _imu->z;
  }
protected:
      int protectServo( int val )
      {
        return max( SERVO_MIN,min( SERVO_MAX, val ));
      }
      
  boolean AdjustBank()
  {
    if( !NavigationEnabled )
    {
      if( _currentAileron == _override_x )
      {
        return false;
      }
      _currentAileron = _override_x;
      return true;
    }
    
    if( _override_x != SERVO_STARTANGLE )
    {
      _currentAileron = _override_x;
      return true;
    }
    
    if( _imu->x ==  _target_x)
      return false;
      
    _pxPID->Compute();
    
    if( _currentAileron != _prevAileron )
    {
      _prevAileron = _currentAileron;
      return true;
      
    }


      
    return true;
  }
  
  void UpdateBank()
  {
    int aileron = protectServo( _currentAileron + (SERVO_STARTANGLE - _currentElevator) + _currentPitch  );
    
    _leftServo.write( aileron );
    _rightServo.write( aileron );
  }
  
  boolean AdjustPitch()
  {
    if( !NavigationEnabled  )
    {
      if( _currentElevator == _override_y )
      {
        return false;
      }
      _currentElevator = _override_y;
      return true;
    }
    
    if( _override_y != SERVO_STARTANGLE)
    {
      _currentElevator = _override_y;
      return true;
    }
    
    if( _imu->y == _target_y )
      return false;
      
    _pyPID->Compute();
    
    if( _prevElevator != _currentElevator )
    {
      _prevElevator = _currentElevator;
      return true;
    }

      
    return false;
  }
 
  void UpdatePitch()
  {
    int elevator = protectServo( _currentElevator + _currentPitch  );

    _elevatorServo.write(elevator);
  }
  

  
  boolean AdjustYaw()
  {
    if( !NavigationEnabled  )
    {
      _currentTailRotor = _override_z;
      return true;
    }
        
    if( _override_z != 0 )
    {
      _currentTailRotor = _override_z;
      return true;
    }
    
    if( _imu->z == _target_z )
      return false;
    
    _pzPID->Compute();
    if( _prevTail != _currentTailRotor )
    {
      _prevTail = _currentTailRotor;
      return true;
    }
    else
      return false;
  }
  
  void UpdateYaw()
  {
    _currentTailRotor = ProtectTailRotor( _currentTailRotor );
    analogWrite(PIN_TAIL, _currentTailRotor );
  }
  
  int ProtectTailRotor (int val )
  {
    int minVal = 0;
    if( NavigationEnabled )
      minVal = TAILROTOR_MIN;
    
    if( val < TAILROTOR_MIN )
      return minVal;
    if( val > TAILROTOR_MAX )
      return TAILROTOR_MAX;
    return val;
  }
  
   void UpdateThrottle()
  {
    analogWrite(PIN_THROTTLE, _currentThrottle);
  }
  
public: 
  Navigator( IMU* imu )
  {
    _currentPitch = 0;
    _currentAileron = SERVO_STARTANGLE;
    _currentElevator = SERVO_STARTANGLE;

    _override_x = SERVO_STARTANGLE;
    _override_y = SERVO_STARTANGLE;
    
    _xyAdjustsPerMs = 50;
    _zAdjustsPerMs = 300;
    
    _imu = imu;
    
    _pxPID = new PID(&_imu->x, &_currentAileron, &_target_x, 2,5,1, DIRECT);
    _pxPID->SetOutputLimits(SERVO_MIN, SERVO_MAX);
    _pyPID = new PID(&_imu->y, &_currentElevator, &_target_y, 2,5,1, REVERSE);
    _pyPID->SetOutputLimits(SERVO_MIN, SERVO_MAX);
    _pzPID = new PID(&_imu->z, &_currentTailRotor, &_target_z, 2,5,1, REVERSE);
    _pzPID->SetOutputLimits(TAILROTOR_MIN, TAILROTOR_MAX);
  }
  
  void Initialize()
  {
      // servo pins
    pinMode(PIN_SERVO_LEFT, OUTPUT);
    pinMode(PIN_SERVO_RIGHT, OUTPUT);
    pinMode(PIN_SERVO_ELEVATOR,OUTPUT);
    pinMode(PIN_THROTTLE, OUTPUT);
    pinMode(PIN_TAIL, OUTPUT);
    pinMode(PIN_RX, INPUT);
    pinMode(PIN_TX, OUTPUT);

    _leftServo.attach(PIN_SERVO_LEFT);
    _rightServo.attach(PIN_SERVO_RIGHT);
    _elevatorServo.attach(PIN_SERVO_ELEVATOR);
    
    UpdatePitch();
    UpdateBank();
    
    SetThrottle(0);
    Yaw(0);
  }
  
  int GetCurrentThrottle()  
  {
     return _currentThrottle;
  }
  
  void Navigate()
  {
      if( AdjustBank() )
      {
        UpdateBank();
      }
      if( AdjustPitch() )
      {
        UpdatePitch();
        UpdateBank();
      }
      if( AdjustYaw() )
        UpdateYaw();
  }
  
  void SetXYPerMs( unsigned int adjustsPerMs )
  {
    _xyAdjustsPerMs = adjustsPerMs;
  }
  
  void SetZPerMs( unsigned long adjustsPerMs )
  {
    _zAdjustsPerMs = adjustsPerMs;
  }
  
  void EmergencyLanding()
  {
    while( _currentThrottle > 0 )
    {
      SetThrottle( _currentThrottle - 1 );
      delay( 50 );
      Navigate();
    }
    NavigationEnabled = false;
    Yaw(0);
  }
  
  void NavigationOnOff( int value )
  {      
      if( value == 0 )
      {
        NavigationEnabled = false;
        _pxPID->SetMode(MANUAL);
        _pyPID->SetMode(MANUAL);
        _pzPID->SetMode(MANUAL);
      }
      if( value == 1 )
      {
        NavigationEnabled = true;
        _pxPID->SetMode(AUTOMATIC);
        _pyPID->SetMode(AUTOMATIC);
        _pzPID->SetMode(AUTOMATIC);
      }
      
  }

  
  // value in degress.  Change both servos
  void Bank( int val )
  {
      _override_x = val;
  }
  
  // value in degrees.  90 is straight
  void Pitch( int val )
  {
      _override_y = val;
  }
  
    // -255 - 255
  void Yaw( int val )
  {
    _override_z = val;
    _override_z = ProtectTailRotor(_override_z);
  }
  
  // 0 - 255
  void SetThrottle( int val )
  {
    _currentThrottle = val;
    UpdateThrottle();
    AdjustPitchFromThrottle();
    UpdatePitch();
    UpdateBank();
  }
  
  void AdjustPitchFromThrottle()
  {
    _currentPitch = _currentThrottle * 5;
    UpdatePitch();
  }
  

};

IMU guide;

Navigator nav( &guide);

void setup() 
{
  // initialize the digital pin as an output.
  // Teensy onboard LED
  pinMode(PIN_LED, OUTPUT);
  // IMU pins
  pinMode(PIN_IMU_X, INPUT);
  pinMode(PIN_IMU_Y, INPUT);
  pinMode(PIN_IMU_Z, INPUT);
  pinMode(PIN_IMU_TEMP, INPUT);
  // voltage
  pinMode(PIN_VOLTAGE,INPUT);
  
  // signal that we're up
  BlinkOn();
  delay(1000);              // wait for a second
  BlinkOff();   // set the LED off

  // start processing commands
  commandProcessor.Begin();

  // IMU
  nav.SetHome();
  
  // NAV
  nav.Initialize();

}

void DumpVoltage()
{
  commandProcessor.Print( "V=" );
  commandProcessor.PrintLine( analogRead( PIN_VOLTAGE ) );
}

int prevVoltage;
void DumpVoltageIfChanged()
{
  int voltNow = analogRead( PIN_VOLTAGE );
  if( abs(voltNow - prevVoltage) > 50 )
  {
    prevVoltage = voltNow;
    commandProcessor.Print( "V=" );
    commandProcessor.PrintLine(  voltNow );
  }
}

void BlinkOff()
{
  digitalWrite(PIN_LED, LOW); 
}

void BlinkOn()
{
  digitalWrite(PIN_LED, HIGH);   // set the LED on
}

void loop() 
{  
  guide.ReadValues();
  boolean change = guide.HaveChange();
  nav.Navigate();
  
  if( change == true )
  {
    commandProcessor.DumpIMU(guide, nav._target_x, nav._target_y, nav._target_z);
  }
   
  Command& command = commandProcessor.GetCommand();

  switch( command.CommandType )
  {
    case Command::Status:
      commandProcessor.DumpIMU(guide, nav._target_x, nav._target_y, nav._target_z);
      commandProcessor.DumpThrottle(nav.GetCurrentThrottle(), nav._zAdjustsPerMs);
      commandProcessor.DumpTailRotor(nav._currentTailRotor, nav._target_z);
      break;
    case Command::Bank:
      nav.Bank( command.Value );
      break;
    case Command::Pitch:
      nav.Pitch( command.Value );
      break;
    case Command::Throttle:
      nav.SetThrottle( command.Value );
      // Safety check for low voltage
      if( nav.GetCurrentThrottle() == 0 )
      {
        int currentVolts =  analogRead( PIN_VOLTAGE );
        if(  currentVolts < MIN_VOLTAGE )
        {
          commandProcessor.Print("Low voltage! ");
          commandProcessor.PrintLine( currentVolts );
        }
      }
      else
      {
        //nav.SetThrottle( command.Value );
      }
      break;
    case Command::Yaw:
      nav.Yaw( command.Value );
      break;
    case Command::Voltage:
      DumpVoltage();
      break;
    case Command::NavigationOnOff:
      nav.NavigationOnOff( command.Value );
      commandProcessor.Print("Navigation set to ");
      commandProcessor.PrintLine( command.Value );
      break;
    case Command::SetHome:
      nav.SetHome();
      commandProcessor.PrintLine("New home set");
      break;
    case Command::SetXYPerMs:
      nav.SetXYPerMs( command.Value );
      commandProcessor.PrintLine("New xy delay = ");
      commandProcessor.PrintLine( command.Value );
      break;
    case Command::SetZPerMs:
      nav.SetZPerMs( command.Value );
      commandProcessor.PrintLine("New z delay = ");
      commandProcessor.PrintLine( command.Value );
      break;
    case Command::Echo:
      commandProcessor.Print(":ER");
      commandProcessor.PrintLine(command.Value);
    default:
      break;
  }
  
  // Safety check for run-away heli
  if( command.CommandType != Command::None )
  {
    BlinkOn();
  }
  else
  {
    BlinkOff();
    unsigned long curTime = millis();
    
    // if auto enabled, do emergency landing
    /*if( nav.NavigationEnabled && (curTime - commandProcessor.GetLastCommTime()) > 3000 )
    {
     if( nav.GetCurrentThrottle() > 0 )
     {
        nav.EmergencyLanding();
        commandProcessor.Print(" Current time = " );
        commandProcessor.Print( curTime );
        commandProcessor.Print(" LastComm time = " );
        commandProcessor.Print( commandProcessor.GetLastCommTime() );
        commandProcessor.PrintLine("Emergency landing due to lost COMM.  I hope nobody died.");
        
     }
    }*/
  
  }
}






