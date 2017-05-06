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
  String response = ":S x:";
  response += (int)imu.x;
  response += " y:";
  response += (int)imu.y;
  response += " z:" ;
  response += (int)imu.z;
  response += " t:";
  response += (int)imu.temp;
  PrintLine(response);
}

void CommandProcessor::NewHome( double targetX, double targetY, double targetZ )
{
  String response = ":NH x:";
  response += (int)targetX;
  response += " y:";
  response += (int)targetY;
  response += " z:" ;
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
  PrintLine(response);
}

void CommandProcessor::NotifyEmergency()
{
  PrintLine("!!!");
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



