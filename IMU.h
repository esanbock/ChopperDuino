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
    virtual void ReadValues();
    virtual boolean HaveChange();

    virtual ~IMU();

  private:
    boolean _haveChange;

  protected:

};
