#include <PWMServo.h>
#include <PID_v1.h>

#include "IMU.h"
#include "CommandProcessor.h"
#include "UartCommandProcessor.h"

static const int SERVO_MIN = 70;
static const int SERVO_MAX = 110;
static const int SERVO_STARTANGLE = 90;
static const int TAILROTOR_MIN = 100;
static const int TAILROTOR_MAX = 254;
static const int THROTTLE_MAX = 255;


const int PIN_THROTTLE = 10;
const int PIN_TAIL = 12;
const int PIN_SERVO_LEFT = SERVO_PIN_A; // pin 14, SERVO_2
const int PIN_SERVO_RIGHT = SERVO_PIN_B; // pin 15, SERVO_3
const int PIN_SERVO_ELEVATOR = SERVO_PIN_C; // pin 4, SERVO_1
const int PIN_VOLTAGE = 16;
const int MIN_VOLTAGE = 590;
const int PIN_LED = 11;
const int PIN_RX = 7;
const int PIN_TX = 8;

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

    int _currentCollective;

    // servos
    double _currentAileron;
    double _currentElevator;
    double _prevAileron;
    double _prevElevator;

    // throttle
    int _currentThrottle;
    double _prevTail;

    int _override_x;
    int _override_y;
    int _override_z;

    //pid
    PID* _pxPID;
    PID* _pyPID;
    PID* _pzPID;

    // blinker
    boolean _blinkerOn;

  public:
    double _currentTailRotor;
    double _target_x;
    double _target_y;
    double _target_z;

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
      return max( SERVO_MIN, min( SERVO_MAX, val ));
    }

    boolean AdjustRoll()
    {
      // if navigation is off
      if ( !NavigationEnabled )
      {
        if ( _currentAileron == _override_x )
        {
          return false;
        }
        _currentAileron = _override_x;
        return true;
      }


      // manual override while navigating
      if ( _override_x != SERVO_STARTANGLE )
      {
        _pxPID->SetMode(MANUAL);
        _currentAileron = _override_x;
        return true;
      }
      else
      {
        // it's ok to do this repeatedly
        _pxPID->SetMode(AUTOMATIC);
      }

      // use the PID
      if ( _pxPID->Compute() )
      {

        if ( _currentAileron != _prevAileron )
        {
          _prevAileron = _currentAileron;
        }
        return true;
      }
      return false;
    }

    void UpdateRoll()
    {
      int elevatorOffset = _currentElevator - SERVO_STARTANGLE;
      int leftAileron = protectServo( _currentAileron - _currentCollective + elevatorOffset );
      int rightAileron = protectServo( _currentAileron + _currentCollective - elevatorOffset );

      _leftServo.write( leftAileron );
      _rightServo.write( rightAileron );
    }

    boolean AdjustPitch()
    {
      if ( !NavigationEnabled  )
      {
        if ( _currentElevator == _override_y )
        {
          return false;
        }
        _currentElevator = _override_y;
        return true;
      }

      if ( _override_y != SERVO_STARTANGLE)
      {
        _pyPID->SetMode(MANUAL);
        _currentElevator = _override_y;
        return true;
      }
      else
      {
        _pyPID->SetMode(AUTOMATIC);
      }

      if ( _pyPID->Compute() )
      {

        if ( _prevElevator != _currentElevator )
        {
          _prevElevator = _currentElevator;
        }
        return true;
      }

      return false;
    }

    void UpdatePitch()
    {
      int elevator = protectServo( _currentElevator + _currentCollective  );

      _elevatorServo.write(elevator);
    }



    boolean AdjustYaw()
    {
      if ( !NavigationEnabled  )
      {
        _currentTailRotor = _override_z;
        return true;
      }

      if ( _override_z != 0 )
      {
        _pzPID->SetMode(MANUAL);
        _currentTailRotor = _override_z;
        return true;
      }

      _pzPID->SetMode(AUTOMATIC);
      if( _pzPID->Compute() )
      {
        if ( _prevTail != _currentTailRotor )
        {
          _prevTail = _currentTailRotor;
          return true;
        }
      }
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
      if ( NavigationEnabled )
        minVal = TAILROTOR_MIN;

      if ( val < TAILROTOR_MIN )
        return minVal;
      if ( val > TAILROTOR_MAX )
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
      _currentCollective = 0;
      _blinkerOn = false;

      _currentAileron = SERVO_STARTANGLE;
      _currentElevator = SERVO_STARTANGLE;

      _override_x = SERVO_STARTANGLE;
      _override_y = SERVO_STARTANGLE;

      _imu = imu;

      _pxPID = new PID(&_imu->x, &_currentAileron, &_target_x, 1, 10,   1, REVERSE);
      _pxPID->SetOutputLimits(SERVO_MIN, SERVO_MAX);
      _pyPID = new PID(&_imu->y, &_currentElevator, &_target_y, 1, 10, 1, REVERSE);
      _pyPID->SetOutputLimits(SERVO_MIN, SERVO_MAX);
      _pzPID = new PID(&_imu->z, &_currentTailRotor, &_target_z, 1, 10, 1, REVERSE);
      _pzPID->SetOutputLimits(TAILROTOR_MIN, TAILROTOR_MAX);
    }

    void Initialize()
    {
      // servo pins
      pinMode(PIN_SERVO_LEFT, OUTPUT);
      pinMode(PIN_SERVO_RIGHT, OUTPUT);
      pinMode(PIN_SERVO_ELEVATOR, OUTPUT);
      pinMode(PIN_THROTTLE, OUTPUT);
      pinMode(PIN_TAIL, OUTPUT);
      pinMode(PIN_RX, INPUT);
      pinMode(PIN_TX, OUTPUT);

      _leftServo.attach(PIN_SERVO_LEFT);
      _rightServo.attach(PIN_SERVO_RIGHT);
      _elevatorServo.attach(PIN_SERVO_ELEVATOR);

      UpdatePitch();
      UpdateRoll();

      SetThrottle(0);
      Yaw(0);
    }

    int GetCurrentThrottle()
    {
      return _currentThrottle;
    }

    void Navigate()
    {
      if ( !IsLinkOk() )
      {
        EmergencyLanding();
        return;
      }

      if ( AdjustRoll() )
      {
        UpdateRoll();
      }
      if ( AdjustPitch() )
      {
        UpdatePitch();
        UpdateRoll();
      }
      if ( AdjustYaw() )
        UpdateYaw();
    }

    void EmergencyLanding()
    {
      commandProcessor.PrintLine("Emergency Landing!!!");
      while ( _currentThrottle > 0 )
      {
        SetThrottle( _currentThrottle - 1 );
        delay( 50 );
      }
      NavigationEnabled = false;
      Yaw(0);
    }

    void NavigationOnOff( int value )
    {
      if ( value == 0 )
      {
        NavigationEnabled = false;
        _pxPID->SetMode(MANUAL);
        _pyPID->SetMode(MANUAL);
        _pzPID->SetMode(MANUAL);
      }
      if ( value == 1 )
      {
        NavigationEnabled = true;
        _pxPID->SetMode(AUTOMATIC);
        _pyPID->SetMode(AUTOMATIC);
        _pzPID->SetMode(AUTOMATIC);
      }

    }


    // value in degress.  Change both servos
    void Roll( int val )
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
    }

    // 0 - 255
    void SetThrottle( int val )
    {
      _currentThrottle = val;
      UpdateThrottle();
      AdjustCollective();
      UpdatePitch();
      UpdateRoll();
      commandProcessor.DumpCollective(_currentCollective);
    }

    void AdjustCollective()
    {
      _currentCollective =  _currentThrottle * 0.078431372549; //   ((SERVO_MAX - SERVO_STARTANGLE) / THROTTLE_MAX);
    }

    bool IsLinkOk()
    {
      if ( (millis() - commandProcessor.GetLastCommTime()) > 3000 )
        return false;

      return true;
    }

    void DumpVoltage()
    {
      commandProcessor.Print( "V=" );
      commandProcessor.PrintLine( analogRead( PIN_VOLTAGE ) );
    }


    void BlinkOff()
    {
      if ( _blinkerOn == false )
        return;

      digitalWrite(PIN_LED, LOW);
      _blinkerOn = false;
    }

    void BlinkOn()
    {
      if ( _blinkerOn == true )
        return;

      digitalWrite(PIN_LED, HIGH);   // set the LED on
      _blinkerOn = true;
    }


    void ProcessCommand(Command& command)
    {

      switch ( command.CommandType )
      {
        case Command::Status:
          commandProcessor.DumpIMU(*_imu, _target_x, _target_y, _target_z);
          commandProcessor.DumpThrottle(GetCurrentThrottle());
          commandProcessor.DumpTailRotor(_currentTailRotor, _target_z);
          commandProcessor.DumpCollective(_currentCollective);
          break;
        case Command::Roll:
          Roll( command.Value );
          break;
        case Command::Pitch:
          Pitch( command.Value );
          break;
        case Command::Throttle:
          // Safety check for low voltage
          /*if ( GetCurrentThrottle() == 0 )
          {
            int currentVolts =  analogRead( PIN_VOLTAGE );
            if (  currentVolts < MIN_VOLTAGE )
            {
              commandProcessor.Print("Low voltage! ");
              commandProcessor.PrintLine( currentVolts );
            }
            return;
          }*/
          SetThrottle( command.Value );
          break;

        case Command::Yaw:
          Yaw( command.Value );
          break;

        case Command::Voltage:
          DumpVoltage();
          break;

        case Command::NavigationOnOff:
          NavigationOnOff( command.Value );
          commandProcessor.Print("Navigation set to ");
          commandProcessor.PrintLine( command.Value );
          break;

        case Command::SetHome:
          SetHome();
          commandProcessor.PrintLine("New home set");
          break;

        case Command::Echo:
          commandProcessor.Print(":ER");
          commandProcessor.PrintLine(command.Value);
          break;

        case Command::None:
          BlinkOff();
          return;

        default:
          break;
      }

      // a fun blinker
      BlinkOn();
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
  pinMode(PIN_VOLTAGE, INPUT);

  // signal that we're up
  nav.BlinkOn();
  delay(1000);              // wait for a second
  nav.BlinkOff();   // set the LED off

  // start processing commands
  commandProcessor.Begin();

  // IMU
  nav.SetHome();

  // NAV
  nav.Initialize();

}


int prevVoltage;
void DumpVoltageIfChanged()
{
  int voltNow = analogRead( PIN_VOLTAGE );
  if ( abs(voltNow - prevVoltage) > 50 )
  {
    prevVoltage = voltNow;
    commandProcessor.Print( "V=" );
    commandProcessor.PrintLine(  voltNow );
  }
}


void loop()
{
  guide.ReadValues();
  nav.Navigate();

  if ( guide.HaveChange() )
  {
    commandProcessor.DumpIMU(guide, nav._target_x, nav._target_y, nav._target_z);
  }
  DumpVoltageIfChanged();

  Command& command = commandProcessor.GetCommand();
  nav.ProcessCommand(command);

}







