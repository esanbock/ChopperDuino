class UartCommandProcessor : public CommandProcessor
{
  private:
    HardwareSerial _uart;
  public:
    UartCommandProcessor();
    void Begin();
    int GetChar();
    int Available();
    void PrintLine( const char * szLine );
    void PrintLine( String line );
    void PrintLine( int num );
    void Print( const char* szChars );
    void Print( int num );
};

