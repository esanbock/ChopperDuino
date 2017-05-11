
class AvgIMU : public IMU
{
  public:
    AvgIMU( );
    virtual void ReadValues();
    virtual boolean HaveChange();

    virtual ~AvgIMU();

  private:
    static const int AVGSIZE = 20;

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
