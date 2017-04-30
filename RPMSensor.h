const int PIN_RPM=13;

class RPMSensor
{

public:
  RPMSensor();
  int CheckAndGetRPM();
protected:

private:
  unsigned long _lastSwitch;
  int _lastHall;
  int _currentRPM;
};
