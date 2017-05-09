#include "Arduino.h"
#include "IMU.h"
#include "CommandProcessor.h"
#include "HardwareSerial.h"
#include "UartCommandProcessor.h"

UartCommandProcessor::UartCommandProcessor()
{
}
void UartCommandProcessor::Begin()
{
  _uart.begin(38400);
  PrintLine("start uart");
}

int UartCommandProcessor::GetChar()
{
  return _uart.read();
}

int UartCommandProcessor::Available()
{
  return _uart.available();
}

void UartCommandProcessor::PrintLine( const char * szLine )
{
  _uart.println( szLine );
  _uart.flush();
}

void UartCommandProcessor::PrintLine( String line )
{
  _uart.println( line );
  _uart.flush();
}

void UartCommandProcessor::PrintLine( String line, bool optional )
{
  if( optional )
  {
    if( _uart.availableForWrite() < (int) line.length() )
      return;
  }
  _uart.println( line );
  _uart.flush();
}

void UartCommandProcessor::PrintLine( int num )
{
  _uart.println( num );
  _uart.flush();
}
void UartCommandProcessor::Print( const char* szChars )
{
  _uart.print( szChars );
}

void UartCommandProcessor::Print( int num )
{
  _uart.print( num );
}

