// Arduino Signal Filtering Library
// Copyright 2012-2013 Jeroen Doggen (jeroendoggen@gmail.com)


#include "iirFilter.h"

/// Constructor
iirFilter::iirFilter()
{
	_y[0]=0;
	_y[1]=0;
	_y[2]=0;
}

/// Begin function: set default filter options
void iirFilter::begin()
{
}

//int iirFilter::run(int data)
//{
//	// not an iirFilter implementation (just a placeholder)
//	_y[0] = _y[1];
//	long tmp = ((((data * 3269048L) >>  2)          //= (3.897009118e-1 * data)
//		+ ((_y[0] * 3701023L) >> 3)                   //+(  0.2205981765*v[0])
//		)+1048576) >> 21;                             // round and downshift fixed point /2097152
//	_y[1]= (int)tmp;
//	return (int)(_y[0] + _y[1]);                    // 2^
//}

float iirFilter::run(float data)
{
	_y[0] = _y[1];
	_y[1] = _y[2];
	_y[2] = (3.131413019611e-2 * data)
		+ ( -1.5592073000 * _y[0])
		+ (  2.4339507792 * _y[1]);
	return 		(_y[0] + _y[2])	+2 * _y[1];
}
