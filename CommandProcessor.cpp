#include "Arduino.h"
#include "IMU.h"
#include "CommandProcessor.h"

CommandProcessor::CommandProcessor()
{

}

unsigned long CommandProcessor::GetLastCommTime()
{
  return _lastCommTime;
}

int CommandProcessor::ReadNum()
{
  char num[5];
  num[3] = (char)NULL;
  num[4] = (char)NULL;
  num[0] = GetCharBlocking();
  num[1] = GetCharBlocking();
  num[2] = GetCharBlocking();
  if ( num[0] == '-' )
    num[3] = GetCharBlocking();


  return atoi( num );
}

int CommandProcessor::GetCharBlocking()
{
  while ( Available() < 1 )
  {
    delay( 100 );
  }
  return GetChar();
}

void CommandProcessor::DumpIMU( IMU& imu, double targetX, double targetY, double targetZ )
{
  Print(":S ");
  Print("x:");
  Print(imu.x);
  Print(" y:");
  Print(imu.y);
  Print(" z:");
  Print(imu.z);
  Print(" t:");
  PrintLine(imu.temp);
}

void CommandProcessor::DumpCollective( double collective )
{
  Print (":C");
  PrintLine( collective ); 
}

void CommandProcessor::DumpThrottle(int t)
{
  Print( "Throttle=" );
  PrintLine( t );
}




void CommandProcessor::DumpTailRotor( int tail, int targetZ )
{
  Print("_currentTailRotor=");
  PrintLine(tail);
}

Command& CommandProcessor::GetCommand()
{
  while ( Available() )
  {
    _lastCommTime = millis();
    byte b = GetChar();

    if ( b == ':' )
    {
      int commandType = GetCharBlocking();

      switch ( commandType )
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
          _command.CommandType = Command::Roll;
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
          if ( _command.Value == 1 || _command.Value == 0 )
            return _command;
        case 'H':
          _command.CommandType = Command::SetHome;
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



