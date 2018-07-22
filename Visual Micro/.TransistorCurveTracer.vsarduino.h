/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Mega 2560 or Mega ADK, Platform=avr, Package=arduino
*/

#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
#define __AVR_ATmega2560__
#define ARDUINO 105
#define ARDUINO_MAIN
#define __AVR__
#define __avr__
#define F_CPU 16000000L
#define __cplusplus
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__

#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define __attribute__(x)
#define NOINLINE __attribute__((noinline))
#define prog_void
#define PGM_VOID_P int
            
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}

//
//
void init_rl_num();
void init_adb();
void init_input_analog();
void adbEventHandler(Connection * connection, adb_eventType event, uint16_t length, uint8_t * data);
void serialEvent();
bool get_command(String strCommand);
bool get_command_setting();
void write_data(String key,float value);
void write_droid(String tr);
void start_find_pin();
void st_curve_npn_to_processing();
void st_curve_pnp_to_processing();
long getDecimal(float val);
void addSerialVoltSet(String msg);
void st_curve_npn_to_adk();
void st_curve_pnp_to_adk();
void print(String name,float s);
void clear_gain();
void relay_on_npn(String row);
void relay_on_pnp(String row);
void tr_result(int row);
void tr_answer();
void relay_off();
void relay_on(String row);

#include "D:\arduino-1.0.5-r2\hardware\arduino\cores\arduino\arduino.h"
#include "D:\arduino-1.0.5-r2\hardware\arduino\variants\mega\pins_arduino.h" 
#include "C:\Users\USER\Documents\Arduino\TransistorCurveTracer\TransistorCurveTracer.ino"
#include "C:\Users\USER\Documents\Arduino\TransistorCurveTracer\MCP4922.cpp"
#include "C:\Users\USER\Documents\Arduino\TransistorCurveTracer\MCP4922.h"
#include "C:\Users\USER\Documents\Arduino\TransistorCurveTracer\eRCaGuy_analogReadXXbit.cpp"
#include "C:\Users\USER\Documents\Arduino\TransistorCurveTracer\eRCaGuy_analogReadXXbit.h"
#include "C:\Users\USER\Documents\Arduino\TransistorCurveTracer\iirFilter.cpp"
#include "C:\Users\USER\Documents\Arduino\TransistorCurveTracer\iirFilter.h"
#endif
