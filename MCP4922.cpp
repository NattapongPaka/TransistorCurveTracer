#include "MCP4922.h"

const float error1 = 80.0;	//1-5V
const float error2 = 70.0;	//6-7V
const float error3 = 60.0;	//8-9V
const float error4 = 50.0;	//10-11V
const float error5 = 40.0;	//12-13V
const float error6 = 20.0;	//14-15V

MCP4922::MCP4922(int __SDI, int __SCK,int __CS1,int __CS2){
	this->_SDI = __SDI;
	this->_SCK = __SCK;
	this->_CS1 = __CS1;
	this->_CS2 = __CS2;

	pinMode(_SDI,OUTPUT);
	pinMode(_SCK,OUTPUT);
	pinMode(_CS1,OUTPUT);
	pinMode(_CS2,OUTPUT);
}

void MCP4922::init(){
	byte clr;
	digitalWrite(_CS1,HIGH); //disable device
	digitalWrite(_CS2,HIGH); //disable device
	SPCR = (1<<SPE)|(1<<MSTR) | (0<<SPR1) | (0<<SPR0);
	clr=SPSR;
	clr=SPDR;
	delay(10);
}

void MCP4922::set_ChanelA(int data){
	// splits int sample in to two bytes
	byte dacSPI0 = 0;
	byte dacSPI1 = 0;
	dacSPI0 = (data >> 8) & 0x00FF; //byte0 = takes bit 15 - 12
	dacSPI0 |= 0x10;				//Set CHA

	dacSPI1 = data & 0x00FF;		//byte1 = takes bit 11 - 0
	dacSPI0 |= (1<<5);				// set gain of 1

	digitalWrite(_CS1,LOW);
	SPDR = dacSPI0;					// Start the transmission
	while (!(SPSR & (1<<SPIF)))		// Wait the end of the transmission
	{
	};

	SPDR = dacSPI1;
	while (!(SPSR & (1<<SPIF)))		// Wait the end of the transmission
	{
	};
	digitalWrite(_CS1,HIGH);
	//digitalWrite(CS1,LOW);
	delay(100);
}

void MCP4922::set_ChanelB(int data){
	// splits int sample in to two bytes
	byte dacSPI0 = 0;
	byte dacSPI1 = 0;
	dacSPI0 = (data >> 8) & 0x00FF; //byte0 = takes bit 15 - 12
	dacSPI0 |= 0x10;

	dacSPI1 = data & 0x00FF;		//byte1 = takes bit 11 - 0
	dacSPI0 |= (1<<5);				// set gain of 1

	digitalWrite(_CS2,LOW);
	SPDR = dacSPI0;					// Start the transmission
	while (!(SPSR & (1<<SPIF)))		// Wait the end of the transmission
	{
	};

	SPDR = dacSPI1;
	while (!(SPSR & (1<<SPIF)))		// Wait the end of the transmission
	{
	};
	digitalWrite(_CS2,HIGH);
	//digitalWrite(CS1,LOW);
	delay(30);
}

void MCP4922::set_Volt_ChanelA(float v){
	float dac;
	if(v == 0){
		dac = 0;
	}else{
		dac = ((v*4095.0) / 20.0) - gainError(v);
		//dac = ceil(dac);
	}
	this->_DacA = dac;
	set_ChanelA((int)dac);
}

void MCP4922::set_Volt_ChanelB(float v){
	float dac;
	if(v == 0){
		dac = 0;
	}else{
		dac = ((v*4095.0) / 20.0) - error1;
		//dac = ceil(dac);
	}
	this->_DacB = dac;
	set_ChanelB((int)dac);
}

float MCP4922::gainError(float v){
	if(v > 0 && v <= 5){
		return error1;
	}

	if(v == 6 || v == 7){
		return error2;
	}

	if(v == 8 || v == 9){
		return error3;
	}

	if(v == 10 || v == 11){
		return error4;
	}

	if(v == 12 || v == 13){
		return error5;
	}

	if(v == 14 || v == 15){
		return error6;
	}

	if(v > 15){
		return 20;
	}
}

float MCP4922::get_Volt_Filter(float v){
	this->_newVal = smooth(v,_filterConstant,this->_newVal);
	return this->_newVal;
}

float MCP4922::smooth(float data, float filterVal, float smoothedVal){
	smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
	return smoothedVal;
}