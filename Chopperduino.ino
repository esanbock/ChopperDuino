#include <PWMServo.h>
#include <PID_v1.h>
#include "ProcessController.h"
#include "SimpleController.h"
#include "PidController.h"
#include "IMU.h"
#include "AvgIMU.h"
#include "CommandProcessor.h"
#include "UartCommandProcessor.h"

// parameters
static const int SERVO_MIN = 70;
static const int SERVO_MAX = 110;
static const int SERVO_STARTANGLE = 90;
static const int TAILROTOR_MIN = 100;
static const int TAILROTOR_MAX = 254;
static const int THROTTLE_MAX = 255;
static const int VOLTAGE_TRIP = 0;

// pins
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
    ProcessController* _pPidController[3];

    // blinker
    boolean _blinkerOn;

  public:
    double _currentTailRotor;
    double _target_x = 500;
    double _target_y = 500;
    double _target_z = 500;

  public:
    boolean NavigationEnabled;
    void SetHome()
    {
      _imu->ReadValues();
      _target_x = _imu->x;
      _target_y = _imu->y;
      _target_z = _imu->z;

      commandProcessor.NewHome(_target_x, _target_y, _target_z);
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
        _pPidController[0]->SetMode(MANUAL);
        _currentAileron = _override_x;
        return true;
      }
      else
      {
        // it's ok to do this repeatedly
        _pPidController[0]->SetMode(AUTOMATIC);
      }

      // use the PID
      if ( _pPidController[0]->Compute() )
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
      //int elevatorOffset = _currentElevator - SERVO_STARTANGLE;
      int leftAileron = protectServo( _currentAileron - _currentCollective );
      int rightAileron = protectServo( _currentAileron + _currentCollective );

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
        _pPidController[1]->SetMode(MANUAL);
        _currentElevator = _override_y;
        return true;
      }
      else
      {
        _pPidController[1]->SetMode(AUTOMATIC);
      }

      if ( _pPidController[1]->Compute() )
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
        _pPidController[2]->SetMode(MANUAL);
        _currentTailRotor = _override_z;
        return true;
      }

      _pPidController[2]->SetMode(AUTOMATIC);
      if( _pPidController[2]->Compute() )
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

      _pPidController[0] = new SimpleController(&_imu->x, &_currentAileron, &_target_x, 1, 10,   1, DIRECT);
      _pPidController[0]->SetOutputLimits(SERVO_MIN, SERVO_MAX);
      _pPidController[1] = new SimpleController(&_imu->y, &_currentElevator, &_target_y, 1, 10, 1, DIRECT);
      _pPidController[1]->SetOutputLimits(SERVO_MIN, SERVO_MAX);
      _pPidController[2] = new SimpleController(&_imu->z, &_currentTailRotor, &_target_z, 1, 10, 1, DIRECT);
      _pPidController[2]->SetOutputLimits(TAILROTOR_MIN, TAILROTOR_MAX);
    }

    virtual ~Navigator()
    {
      delete _pPidController[0];
      delete _pPidController[1];
      delete _pPidController[2];
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

    void EmergencyLanding()
    {
      commandProcessor.NotifyEmergency();
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
        _pPidController[0]->SetMode(MANUAL);
        _pPidController[1]->SetMode(MANUAL);
        _pPidController[2]->SetMode(MANUAL);
      }
      if ( value == 1 )
      {
        NavigationEnabled = true;
        _pPidController[0]->SetMode(AUTOMATIC);
        _pPidController[1]->SetMode(AUTOMATIC);
        _pPidController[2]->SetMode(AUTOMATIC);
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


    int _currentVoltage;

    void UpdateVoltage()
    {
      _currentVoltage = analogRead( PIN_VOLTAGE );
    }
    
    void DumpVoltageIfChanged()
    {
      static int lastMillis=0;
      int m = millis();
      if( m - lastMillis < 1000 )
      {
        return;
      }
      lastMillis = m;
      
      int voltNow = analogRead( PIN_VOLTAGE );
      if ( abs(voltNow - _currentVoltage) > VOLTAGE_TRIP && voltNow > 0 )
      {
        _currentVoltage = voltNow;
        commandProcessor.DumpVoltage(  voltNow );
      }
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
          SetThrottle( command.Value );
          break;

        case Command::Yaw:
          Yaw( command.Value );
          break;

        case Command::Voltage:
          UpdateVoltage();
          commandProcessor.DumpVoltage(_currentVoltage);
          break;

        case Command::NavigationOnOff:
          NavigationOnOff( command.Value );
          break;

        case Command::SetHome:
          SetHome();
          break;

        case Command::Echo:
          commandProcessor.RespondToEcho(command.Value);
          break;

        case Command::SetPid:
          _pPidController[command.Value]->SetTunings(command.kP,command.kI,command.kD);
          commandProcessor.DumpPidValues(command.Value, command.kP, command.kI, command.kD);
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

    void Navigate()
    {
      if ( !IsLinkOk() )
      {
        EmergencyLanding();
        return;
      }

      // the pitch impacts the roll
      bool doRoll = AdjustRoll();
      bool doPitch = AdjustPitch();
      if ( doRoll )
      {
        UpdateRoll();
      }
      if ( doPitch )
      {
        UpdatePitch();
      }
      if ( AdjustYaw() )
        UpdateYaw();

      if( doRoll || doPitch )
      {
        commandProcessor.DumpMotors(_currentAileron, _currentElevator, _currentTailRotor );
      }
    }


};

AvgIMU guide;

Navigator nav( &guide);

void setup()
{
  // initialize the digital pin as an output.
  // Teensy onboard LED
  pinMode(PIN_LED, OUTPUT);
  // voltage
  pinMode(PIN_VOLTAGE, INPUT);

  // signal that we're up
  nav.BlinkOn();
  delay(1000);              // wait for a second
  nav.BlinkOff();   // set the LED off

  // start processing commands
  commandProcessor.Begin();

  // IMU
  //nav.SetHome();

  // NAV
  nav.Initialize();

}


void loop()
{
  Command& command = commandProcessor.GetCommand();
  nav.ProcessCommand(command);
  
  guide.ReadValues();
  nav.Navigate();

  if ( guide.HaveChange() )
  {
    commandProcessor.DumpIMU(guide, nav._target_x, nav._target_y, nav._target_z);
  }
  
  nav.DumpVoltageIfChanged();


}







