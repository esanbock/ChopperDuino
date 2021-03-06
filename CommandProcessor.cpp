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
  if ( Available() < 1 )
  {
    delay( 10 );
  }
  return GetChar();
}

void CommandProcessor::DumpIMU( IMU& imu, double targetX, double targetY, double targetZ )
{
  String response = ":S ";
  response += (int)imu.x;
  response += ",";
  response += (int)imu.y;
  response += "," ;
  response += (int)imu.z;
  response += ",";
  response += (int)imu.temp;
  PrintLine(response, true);
}

void CommandProcessor::DumpMotors( int currentAileron, int currentElevator, int currentTailRotor )
{
  String response = ":M ";
  response += currentAileron;
  response += ",";
  response += currentElevator;
  response += "," ;
  response += currentTailRotor;
  PrintLine(response, true);
}

void CommandProcessor::DumpPidValues( int pidNum, double kP, double kI, double kD )
{
  String response = "New PID values for pid ";
  response += pidNum;
  response += ": ";
  response += kP;
  response += ",";
  response += kI;
  response += ",";
  response += kD;
  PrintLine(response);
}

void CommandProcessor::NewHome( double targetX, double targetY, double targetZ )
{
  String response = ":NH ";
  response += (int)targetX;
  response += ",";
  response += (int)targetY;
  response += "," ;
  response += (int)targetZ;
  PrintLine(response);
}
void CommandProcessor::DumpCollective( int collective )
{
  String response = ":C";
  response += collective;
  PrintLine( response ); 
}

void CommandProcessor::DumpThrottle(int t)
{
  String response = ( "Throttle=" );
  response += t;
  PrintLine( response );
}


void CommandProcessor::DumpVoltage(int v)
{
  String response = "V=";
  response += v;
  PrintLine( response );
}

void CommandProcessor::DumpTailRotor( int tail, int targetZ )
{
  String response = "_currentTailRotor=";
  response += tail;
  PrintLine(response);
}

void CommandProcessor::RespondToEcho( int e )
{
  String response = ":ER";
  response += e;
  PrintLine(response,true);
}

void CommandProcessor::NotifyEmergency()
{
  PrintLine("!!!",true);
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
        case 'Q':
          return ParsePidTuneCommand();
        case 'L':
          _command.CommandType = Command::Lift;
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

Command& CommandProcessor::ParsePidTuneCommand()
{
  _command.CommandType = Command::None;

  _command.Value = GetStream().parseInt();
    char c = GetCharBlocking();
  if( c == ',' )
  {
    _command.kP = GetStream().parseFloat();
    char c = GetCharBlocking();
    if( c != ',' )
    {
      _command.kI = GetStream().parseFloat();
      c = GetCharBlocking();
      if( c == ',' )
      {
        _command.kD = GetStream().parseFloat();
        _command.CommandType = Command::SetPid;
        return _command;
      }
    }
  }
  PrintLine("NAK: ParsePidTuneCommand");
  return _command;        
}



