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
}
void UartCommandProcessor::PrintLine( int num )
{
  _uart.println( num );
}
void UartCommandProcessor::Print( const char* szChars )
{
  _uart.print( szChars );
}

void UartCommandProcessor::Print( int num )
{
  _uart.print( num );
}

