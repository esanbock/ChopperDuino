class SerialCommandProcessor : public CommandProcessor
{
  private:
  public:
    SerialCommandProcessor();
    void Begin();
    int GetChar();
    int Available();
    void PrintLine( const char * szLine );
    void PrintLine( int num );
    void Print( const char* szChars );
    void Print( double num );
    void Print( int num );
};
