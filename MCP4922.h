#ifndef MCP4922_h
#define MCP4922_h

#include <inttypes .h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define _filterConstant 0.90

class MCP4922{
	private:	
		int _DacA;
		int _DacB;
		int _SDI;
		int _SCK;
		int _CS1;
		int _CS2;

		float gainError(float v);
		float _newVal;
		float smooth(float data, float filterVal, float smoothedVal);

	public:		
		MCP4922(int __SDI, int __SCK,int __CS1,int __CS2);
		void set_ChanelA(int data);
		void set_ChanelB(int data);
		void set_Volt_ChanelA(float v);
		void set_Volt_ChanelB(float v);

		void get_Volt_ChanelA(float v);
		void get_Volt_ChanelB(float v);
		float get_Volt_Filter(float v);		
		void init();

};
#endif