#ifndef Motor_h
#define Motor_h
#include "Arduino.h"
class Motor
{
  public:
    Motor(int DIRpin,int DIRpin_S, int SPEEDpin);
    void set(int SPEED);
    void update();
    void setProperties(int maxSPEED, int minSPEED, int Step);
    void Debug();
    int getSPEED();
  private:
    int _DIR;
    int _DIR_S;    
    int _SPEED;
    int _maxSPEED;
    int _minSPEED;
    int _step;
    
    int _cSPEED;
    int _tSPEED;
};

#endif
