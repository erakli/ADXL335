/*
  ADXL335.cpp - Library for reading the ADXL335 Accelerometer
  Created by Derek Chafin, September 14, 2011
  Released into the public domain.
*/

#include "ADXL335.h"

const float rad2deg = 180.0 / M_PI;

bool FuzzyIsNull(float f) {
    return (abs(f) <= 0.00001f);
}


ADXL335::ADXL335(int pin_x, int pin_y, int pin_z, float aref, 
                 float zero_x, float zero_y, float zero_z)
    : _pin_x(pin_x)
    , _pin_y(pin_y)
    , _pin_z(pin_z)
    , _aref(aref)
    , _mvG(aref / 10.0) // datasheet says 1G typically corresponds to 330 mV voltage difference
    , _deadzone(0.0)
{
    float default_bias = aref / 2.0;

    if ( !FuzzyIsNull(zero_x) )
        _bias[0] = zero_x;
    else
        _bias[0] = default_bias;

    if ( !FuzzyIsNull(zero_y) )
        _bias[1] = zero_y;
    else
        _bias[1] = default_bias;

    if ( !FuzzyIsNull(zero_z) )
        _bias[2] = zero_z;
    else
        _bias[2] = default_bias;
}


//begin public methods

void ADXL335::setThreshold(float deadzone)
{
    _deadzone = deadzone;
}


//gets whether the device is in free fall
bool ADXL335::getFreefall()
{
    //if all three vectors read zero then return true, otherwise; false.
    return (_xg == 0.0 && _yg == 0.0 && _zg == 0.0);
}


float ADXL335::getX()
{
    return _xg;
}


float ADXL335::getY()
{
    return _yg;
}


float ADXL335::getZ()
{
    return _zg;
}


float ADXL335::getRho()
{
    return _getRho(_xg, _yg, _zg);
}


float ADXL335::getPhi()
{
    return _getPhi(_xg, _yg, _zg);
}


float ADXL335::getTheta()
{
    return _getTheta(_xg, _yg, _zg);
}


void ADXL335::update()
{
    _xg = getGravity(analogRead(_pin_x), 0);
    _yg = getGravity(analogRead(_pin_y), 1);
    _zg = getGravity(analogRead(_pin_z), 2);
}

//end public methods



//begin private methods

float ADXL335::geta2d(float gx, float gy)
{
    float a = gx * gx;
    a = fma(gy, gy, a);

    return sqrt(a);
}


//gets the magnitude of the 3d vector
//the formula is a^2 = x^2 + y^2 + z^2
float ADXL335::geta3d(float gx, float gy, float gz)
{
    //use floating point multiply-add cpu func
    //sometimes we get better precision
    float a = gx * gx;
    a = fma(gy, gy, a);
    a = fma(gz, gz, a);

    return sqrt(a);
}


void ADXL335::processDeadzone(float *gv)
{
    if (abs(*gv) < _deadzone)
    {
        *gv = 0.0;
    }
}


float ADXL335::getGravity(int reading, int axis)
{
    float voltage = reading * _aref;
    voltage /= ADC_AMPLITUDE;

    //minus the zero g bias
    //then divide by mv/g
    //which when Vs = 3.3V, V/g = 0.330
    float gv = (voltage - _bias[axis]) / _mvG;

    //minus the null zone
    processDeadzone(&gv);

    return gv;
}


float ADXL335::_getRho(float ax, float ay, float az)
{
    return geta3d(_xg, _yg, _zg);
}


float ADXL335::_getPhi(float ax, float ay, float az)
{
    return atan2(ay, ax) * rad2deg;
}


float ADXL335::_getTheta(float ax, float ay, float az)
{
    float rho = _getRho(ax, ay, az);

    if (FuzzyIsNull(rho))
        return NAN;
    else
        return acos(az / rho) * rad2deg;
}

//end private methods
