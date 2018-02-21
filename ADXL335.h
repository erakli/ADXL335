/*
  ADXL335.h - Library for reading the ADXL335 Accelerometer
  Created by Derek Chafin, September 14, 2011
  Released into the public domain.
*/

#ifndef ADXL335_h
#define ADXL335_h

#include "Arduino.h"

#define ADC_AMPLITUDE 1024.0 //amplitude of the 10bit-ADC of Arduino is 1024LSB

class ADXL335
{
  public:
    ADXL335(int pin_x, int pin_y, int pin_z, float aref, 
            float zero_x = 0.0, float zero_y = 0.0, float zero_z = 0.0);

    void setThreshold(float deadzone);
    bool getFreefall();

    float getX();
    float getY();
    float getZ();

    float getRho();
    float getPhi();
    float getTheta();

    void update();

  private:
    float geta2d(float gx, float gy);
    float geta3d(float gx, float gy, float gz);

    void processDeadzone(float *gv);
    float getGravity(int reading, int axis);

    float _getRho(float ax, float ay, float az);
    float _getPhi(float ax, float ay, float az);
    float _getTheta(float ax, float ay, float az);

    int _pin_x;
    int _pin_y;
    int _pin_z;

    float _aref;
    float _mvG;
    float _bias[3] = {0};

    float _deadzone;

    float _xg;
    float _yg;
    float _zg;
};
#endif
