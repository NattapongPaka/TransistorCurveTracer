// Arduino Signal Filtering Library
// Copyright 2012-2013 Jeroen Doggen (jeroendoggen@gmail.com)

#ifndef iirFilter_h
#define iirFilter_h
#include <inttypes .h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Filter.h>

class iirFilter
{
  public:
    iirFilter();
    void begin();

    float run(float data);

  private:
    float _y[3];
};
#endif
