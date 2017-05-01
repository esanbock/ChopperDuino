const int PIN_IMU_X = 19;
const int PIN_IMU_Y = 20;
const int PIN_IMU_Z = 18;
const int PIN_IMU_TEMP = 17;

class IMU
{
  public:
    double x;
    double y;
    double z;
    int temp;

    IMU( );
    void ReadValues();
    boolean HaveChange();

    ~IMU();

  private:
    static const int AVGSIZE = 40;

    int _realX;
    int _realY;
    int _realZ;

    double _px;
    double _py;
    double _pz;

  protected:

    double avg(int* array, int count );
    double movingAvg(double currentVal, int newVal, int count );

};
