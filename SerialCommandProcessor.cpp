#include "Arduino.h"
#include "IMU.h"
#include "CommandProcessor.h"
#include "SerialCommandProcessor.h"

#include "usb_api.h"

SerialCommandProcessor::SerialCommandProcessor()
{
}

void SerialCommandProcessor::Begin()
{
  Serial.begin(38400);
  PrintLine("start serial");
}

int SerialCommandProcessor::GetChar()
{
  return Serial.read();
}

int SerialCommandProcessor::Available()
{
  return Serial.available();
}

void SerialCommandProcessor::PrintLine( const char * szLine )
{
  Serial.println( szLine );
}
void SerialCommandProcessor::PrintLine( int num )
{
  Serial.println( num );
}
void SerialCommandProcessor::Print( const char* szChars )
{
  Serial.print( szChars );
}

void SerialCommandProcessor::Print( double num )
{
  Serial.print( num );
}

void SerialCommandProcessor::Print( int num )
{
  Serial.print( num );
}

