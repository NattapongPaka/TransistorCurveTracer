#include <SPI.h>
#include <Adb.h>
#include <ArduinoJson\ArduinoJson.h>
#include "eRCaGuy_analogReadXXbit.h"
#include "MCP4922.h"
//*****************************************************************
//Analog pin mapping
//#define AI1 A0                       //Analog input monitor 1
//#define AI2 A1                       //Analog input monitor 2
//#define AI3 A2                       //Analog input monitor 1
//#define AI4 A3                       //Analog input monitor 2
//*****************************************************************
//Delay pin
#define RL1 22
#define RL2 24
#define RL3 26

#define RL4 28
#define RL5 30
#define RL6 32

#define RL7 34
#define RL8 36
#define RL9 23

#define RL10 25
#define RL11 27
#define RL12 29

#define RL13 31
#define RL14 33
#define RL15 35

#define RL16 37

#define RD1 39
#define RD2 41
#define RD3 43
#define RD4 45
#define RD5 47

#define STATUS 45
#define LED 13 
#define RESET 12
//*****************************************************************
//IC MCP4922 pin mapping ADK
#define SDI 51						//MOSI - SDI
#define SCK 52						//SCK
#define CS1 48						//CS1
#define CS2 53						//CS2
//*****************************************************************
//instantiate an object of this library class call it "adc"
eRCaGuy_analogReadXXbit adc;

//mcp4922 object
MCP4922 mcp4922(SDI,SCK,CS1,CS2);
//*****************************************************************
//ADB Connection
Connection * connection;
//*****************************************************************
//String command
//Set command
const String CMD_START			= "start"; 
const String CMD_START_NPN		= "start_npn"; 
const String CMD_START_PNP		= "start_pnp"; 
const String CMD_SET_VCE_MAX	= "set_vce_max";
const String CMD_SET_VCE_STEP	= "set_vce_step";
const String CMD_SET_IB_MAX		= "set_ib_max";
const String CMD_SET_IB_STEP	= "set_ib_step";
const String CMD_SET_VBB		= "set_vbb";
const String CMD_SET_VCC		= "set_vcc";
const String CMD_SET_PIN_NPN	= "set_pin_npn";
const String CMD_SET_PIN_PNP	= "set_pin_pnp";
const String CMD_SETTING		= "setting";
const String CMD_NEXT_LINE		= "next_line";
const String CMD_END			= "end";
const String CMD_END_LINE		= "endline";
//Get command
const String CMD_GET_VBB_VALUE = "get_vbb_value";
const String CMD_GET_VCC_VALUE = "get_vcc_value";
const String CMD_GET_VBB_ANALOG = "get_vbb_analog";
const String CMD_GET_VCC_ANALOG = "get_vcc_analog";
const String CMD_GET_VRB_ANALOG = "get_vrb_analog";
const String CMD_GET_VRC_ANALOG = "get_vrc_analog";
//Checking command
const String CMD_CHK_PIN	= "check_pin";
const String CMD_SEND_DATA	= "send_data";
const String CMD_RECONNECT	= "reconnect";
//*****************************************************************
//constants required to determine the voltage at the pin
const float MAX_READING_10_bit	= 1023.0;
const float MAX_READING_11_bit	= 2046.0;
const float MAX_READING_12_bit	= 4092.0;
//*****************************************************************
//Divider constant variable
const int R1 = 2000000;
const int R2 = 1000000;
const float DivRatio1 = 0.2855;
const float DivRatio2 = 0.2755;
//*****************************************************************
//Transistor constant variable
const float Vbe = 0.7;
const float Rc = 100;
const float Rb = 10000;
//const int Vce_max = 15;		//V
//const float Ib_max = 1000;	//1000 uA = 1 * 10^-6
//Transistor voltage variable
float Vbb = 0;
float Vcc = 0;
float Vrb = 0;
float Vrc = 0;
float Vce = 0;
//Transistor current variable
float Ic = 0;
float Ib = 0;
//*****************************************************************
//Transistor gain
//NPN
float gain_npn = 0;
float maxGain_npn = 0;
int bestConfig_npn = -1;
//PNP
float gain_pnp = 0;
float maxGain_pnp = 0;
int bestConfig_pnp = -1;
//NEG Count
int npn_neg = 0;
int pnp_neg = 0;
//*****************************************************************
//Analog num variable
float adc0_num = 0;
float adc1_num = 0;
float adc2_num = 0;
float adc3_num = 0;
//*****************************************************************
//Test Mode
//float error_Vbb = 0;
//float error_Vcc = 0;
//int value;
//int filtered;
//float newVal = 0.0;
//float filterConstant = 0.90; // filter constant
//*****************************************************************
//Relay pin number
const int rl_num[] = {	
22,24,26,
28,30,32,
34,36,23,
25,27,29,
31,33,35,
37,41,43,
45,47
};
//*****************************************************************
//Relay status
//NPN
String rl_npn_pintable[] = {
"0001101000110001",
"0001110000101001",
"0001110001000101",
"0001101010000101",
"0001100110001001",
"0001100101010001"};
//PNP
String rl_pnp_pintable[] = {
"1000001000100010",
"0100010000100010",
"0010010001000010",
"0010001010000010",
"0100000110000010",
"1000000101000010"};
//Pin All
String rl_pintable[] = {
//NPN
"0001101000110001",
"0001110000101001",
"0001110001000101",
"0001101010000101",
"0001100110001001",
"0001100101010001",
//PNP
"1000001000100010",
"0100010000100010",
"0010010001000010",
"0010001010000010",
"0100000110000010",
"1000000101000010"
};
//Relay pin name
String tr_pin[] = {	
"ebc",
"bec",
"bce",
"cbe",
"ceb",
"ecb"	
};
//Relay time
int delay_npn = 30;
int delay_pnp = 30;
//*****************************************************************
//Serial Variable
String sRev = "";
struct st_data {
	String key;
	float value;
}serial_data;
//*****************************************************************
//ADK Variable
bool isReady = true;
bool isRunningCurve_NPN;
bool isRunningCurve_PNP;
bool isFindPin;
bool isNPN;
bool isPNP;
int  iCount = 0;
bool isNPN_Notfound;
bool isPNP_Notfound;
int iCountNPN=0;
int iCountPNP=0;
String vce_max_v;	
String vce_step_v;
String ib_max_micro;
String ib_step_micro;

int iVce_max_v;	
int iVce_step_v;
int iIb_max_micro;
int iIb_step_micro;

int ib_step;
int vce_count;	
int Vbb_volt;
//*****************************************************************
//ADK String Buffer Volt
const int sizeCurve = 273;
const int sizeSerialVoltSet = 15;
String serialVoltSet[sizeSerialVoltSet];
//*****************************************************************
//ADK Settings Curve
//*****************************************************************
//Json Object
DynamicJsonBuffer jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
//*****************************************************************
//Start Program
void setup() {
	Serial.begin(115200);
	init_adb();
	mcp4922.init();
	init_rl_num();
	init_input_analog();	
	pinMode(RESET,OUTPUT);	
}
//*****************************************************************
//Main Program
void loop() {
	//NPN 
	if(isRunningCurve_NPN){		
		Vbb = 0.7;
		//float countIb_step;
		//Vbb = pow(settings.ib_step,-6)*Rb;

		int countIb_Step = 0;
		float Volt = 0;
		//while(countIb_Step < iIb_max_micro){
		//	countIb_Step += iIb_step_micro;
		//	Volt = (countIb_Step / 10000.0) * 100.0;
		//	Serial.println(Volt);
		//}
		//iVce_max_v = vce_max_v.toInt();
		//iVce_step_v = vce_step_v.toInt();

		iIb_max_micro = ib_max_micro.toInt()*100;
		iIb_step_micro = ib_step_micro.toInt()*10;

		serialVoltSet[sizeSerialVoltSet];	
		iCount = 0;
		while(Vbb < 1.3f){
			
			countIb_Step += iIb_step_micro;			
			st_curve_npn_to_adk();

			//Vbb = (countIb_Step / 10000.0) * 100.0;
			//countIb_step +=  pow(settings.ib_step,-6)*Rb;
			Vbb += 0.1f;
			//Vbb = countIb_step;
			Serial.println(Vbb);
			iCount = 0;	
			serialVoltSet[sizeSerialVoltSet];
			Serial.println();
			write_droid(CMD_END_LINE);
		}
		isRunningCurve_NPN = false;
		isReady = true;	
		write_droid(CMD_END);
	}
	//PNP
	if(isRunningCurve_PNP){
		Vbb = 0.9;
		serialVoltSet[sizeSerialVoltSet];
		iCount = 0;
		while(Vbb < 2.2f){
			st_curve_pnp_to_adk();
			Vbb += 0.3;
			iCount = 0;	
			serialVoltSet[sizeSerialVoltSet];	
			Serial.println();
			write_droid(CMD_END_LINE);
		}
		isRunningCurve_PNP = false;
		isReady = true;	
		write_droid(CMD_END);
	}
	//Find pin auto
	if(isFindPin){
		start_find_pin();	
		isFindPin = false;	
		isReady = true;
	}
	//Poll adb
	if(isReady){
		ADB::poll();
	}
}
/******************************************************************
/*
/*	Initilizing 
/*
/******************************************************************/
// Init relay
void init_rl_num(){
	for(int i=22;i<38;i++){
		pinMode(i,OUTPUT);
	}
	pinMode(RD1,OUTPUT);
	pinMode(RD2,OUTPUT);
	pinMode(RD3,OUTPUT);
	pinMode(RD4,OUTPUT);
	pinMode(RD5,OUTPUT);
}
/******************************************************************/
// Init adb
void init_adb(){
	pinMode(13,OUTPUT);  
	ADB::init();
	// Open an ADB stream to the phone's shell. Auto-reconnect. Use any unused port number eg:4568
	connection = ADB::addConnection("tcp:4568", true, adbEventHandler);  
}
// Init setting curve
//void init_setting_curve(){
//	setting.vce_max_v = 15;
//	setting.vce_step_v = 1;
//	setting.vce_count = 4095/45;		// = 91
//	setting.ib_max_micro = 1000;		// microAmp
//	setting.ib_step_micro = 100;		// microAmp
//}
// Init input analog pin
void init_input_analog(){
	pinMode(A0,INPUT);
	pinMode(A1,INPUT);
	pinMode(A2,INPUT);
	pinMode(A3,INPUT);
	pinMode(STATUS,OUTPUT);
}
/******************************************************************
/*
/*	ADB event input command & value from android
/*
/******************************************************************/
void adbEventHandler(Connection * connection, adb_eventType event, uint16_t length, uint8_t * data)
{
	String cmd = "";
	switch(event){

	case ADB_CONNECTION_RECEIVE:
		for(int i=0;i<length;i++){
			cmd += (char)data[i];
		}
		sRev = cmd;
		Serial.println(sRev);

		if(get_command_setting()){

			iVce_max_v = vce_max_v.toInt();
			iVce_step_v = vce_step_v.toInt();

			iIb_max_micro = ib_max_micro.toInt()*100;
			iIb_step_micro = ib_step_micro.toInt()*10;

			int countIb_Step;
			float Volt;
			while(countIb_Step < iIb_max_micro){
				countIb_Step += iIb_step_micro;
				Volt = (countIb_Step / 10000.0) * 100.0;
				Serial.println(Volt);
			}

			//Serial.println(iVce_max_v);
			//Serial.println(iVce_step_v);
			//Serial.println(iIb_max_micro);
			//Serial.println(iIb_step_micro);
		}

		if(get_command(CMD_SET_VBB)){
			//Vbb = serial_data.value;
		}

		if(get_command(CMD_CHK_PIN)){
			isFindPin = true;
		}

		if(get_command(CMD_START)){
			Serial.print(" NPN: ");
			Serial.print(isNPN);
			Serial.print(":\t");
			Serial.print(bestConfig_npn);

			Serial.print(" PNP: ");
			Serial.print(isPNP);
			Serial.print(":\t");
			Serial.print(bestConfig_pnp);

			Serial.println();

			if(isNPN){
				relay_on(rl_pintable[bestConfig_npn]);
				Serial.print(rl_pintable[bestConfig_npn]);
				isRunningCurve_NPN = true;
			}
			if(isPNP){
				relay_on(rl_pintable[bestConfig_pnp]);
				Serial.print(rl_pintable[bestConfig_pnp]);
				digitalWrite(RD5,HIGH);
				digitalWrite(RD3,HIGH);
				digitalWrite(RD4,HIGH);
				isRunningCurve_PNP = true;
			}			
		}
		break;
	
	case ADB_CONNECTION_FAILED:
		//isReady = true;
		digitalWrite(RD1,LOW);
		relay_off();
		isRunningCurve_NPN = false;
		isRunningCurve_PNP = false;
		isFindPin = false;
		Serial.println("ADB_CONNECTION_FAILED");
		break;

	case ADB_CONNECTION_OPEN:
		//isReady = true;
		digitalWrite(RD1,HIGH);
		Serial.println("ADB_CONNECTION_OPEN");
		break;

	case ADB_CONNECTION_CLOSE:
		//isReady = true;
		digitalWrite(RD1,LOW);
		relay_off();
		isRunningCurve_NPN = false;
		isRunningCurve_PNP = false;
		isFindPin = false;
		Serial.println("ADB_CONNECTION_CLOSE");
		break;

	case ADB_CONNECT:
		isReady = true;
		digitalWrite(RD1,HIGH);
		relay_off();
		isRunningCurve_NPN = false;
		isRunningCurve_PNP = false;
		isFindPin = false;
		Serial.println("ADB_CONNECT");
		break;

	case ADB_DISCONNECT:
		isReady = true;
		digitalWrite(RD1,LOW);
		relay_off();
		isRunningCurve_NPN = false;
		isRunningCurve_PNP = false;
		isFindPin = false;
		Serial.println("ADB_DISCONNECT");
		break;
	}
}
/******************************************************************
/*																  
/*	Serial Interrupt when the Console says something			  
/*																  
/******************************************************************/
void serialEvent(){
	if(Serial.available() > 0){
		sRev = Serial.readStringUntil('\n');
		if(get_command(CMD_CHK_PIN)){			
			start_find_pin();					
		}

		if(get_command("show_setting")){
			
		}

		if(get_command("next_ib")){
			
			float v;
			v = 0.9;
			while(Ib < 100.0){
				mcp4922.set_Volt_ChanelB(v);
				//Read Analog
				adc0_num = adc.analogReadXXbit(A0,12,25);	//VCC	-> AN1
				adc1_num = adc.analogReadXXbit(A1,12,25);	//VRC	-> AN2

				adc2_num = adc.analogReadXXbit(A2,12,25);	//VBB	-> AN3
				adc3_num = adc.analogReadXXbit(A3,12,25);	//VRB	-> AN4
						
				Vbb =  (adc2_num / 4095) * 15;
				Vrb =  (adc3_num / 4095) * 15;
				Ib = (Vbb-Vrb)/Rb;

				write_data("IB",(Ib*pow(10,6)));
				v += 0.1;
				Ib = Ib*pow(10,6);
			}
			write_data("VBB",v);
		}

		if(get_command(CMD_SET_VBB)){
			Vbb = serial_data.value;
			
		}

		if(get_command(CMD_GET_VBB_VALUE)){
			write_data(CMD_GET_VBB_VALUE,Vbb);
			Serial.println();
		}

		if(get_command(CMD_START_NPN)){
			st_curve_npn_to_processing();
		}

		if(get_command(CMD_START_PNP)){
			st_curve_pnp_to_processing();
		}		

		if(get_command(CMD_SET_PIN_NPN)){
			relay_on(rl_npn_pintable[((int)serial_data.value)-1]);
		}

		if(get_command(CMD_SET_PIN_PNP)){
			relay_on(rl_pnp_pintable[((int)serial_data.value)-1]);
			digitalWrite(RD5,HIGH);
			digitalWrite(RD3,HIGH);
			digitalWrite(RD4,HIGH);
		}
	}
}
/******************************************************************
/*
/*	Command Checking
/*
/******************************************************************/
bool get_command(String strCommand){
	int ikey = sRev.indexOf(',');
	if(ikey == -1){
		sRev.trim();
		if(sRev.equalsIgnoreCase(strCommand)){
			Serial.println("-1 "+sRev);
			return true;
		}
	}else{
		String sCom = sRev.substring(0,ikey);
		sCom.trim();
		String sValue = sRev.substring(ikey+1,sRev.length());
		sValue.trim();
		serial_data.key = sCom;
		serial_data.value = atof(sValue.c_str());
		if(sCom.equalsIgnoreCase(strCommand)){
			return true;
		}
		Serial.println("0 "+sRev);
	}
	return false;
}

bool get_command_setting(){
	if(sRev.startsWith("S")){
		int start = sRev.indexOf("\r");
		int end = sRev.lastIndexOf("}")+1;
		String jSonString = sRev.substring(start,end);
		Serial.println(jSonString);
		char ch[jSonString.length()+1];
		jSonString.toCharArray(ch,sizeof(ch),0);

		JsonObject& root = jsonBuffer.parseObject(ch);
		vce_max_v = root["VCE_MAX"];
		vce_step_v = root["VCE_STEP"];
		ib_max_micro = root["IB_MAX"];
		ib_step_micro = root["IB_STEP"];
		
		return true;
	}
}
/******************************************************************
/*
/*	Serial write data console
/*
/******************************************************************/
//Write data to processing
void write_data(String key,float value){
	Serial.print(key);
	Serial.print(":");
	Serial.print(value,6);
	Serial.print("\t");
	Serial.flush();
}
//Write data to adk
void write_droid(String tr){
	char ch[tr.length()+1];
	tr.toCharArray(ch,sizeof(ch),0);	
	connection->writeString(ch);
	delay(50);
	ADB::poll();
	delay(60);
}
/******************************************************************
/*
/*	Create Json String
/*
/******************************************************************/
//String buildJson(String num1,String num2,String num3,String num4,String comma) {
// String data;
//   
//  data = "{";
//  data+="\"adc0_num\": \"";
//  data+=num1;
//  //data+= ",";
//  data+="\" , \"adc1_num\": \"";
//  data+=num2;
//  //data+= ",";
//  data+="\" , \"adc2_num\": \"";
//  data+=num3;
// // data+= ",";
//  data+="\" , \"adc4_num\": \"";
//  data+=num4;
//  data+="\"}";
//  
//  data+= comma;
//
//  return data;
//}

/******************************************************************
/*
/*	Start find pin auto
/*
/******************************************************************/
void start_find_pin(){
//Clear value
	clear_gain();
	//Show 
	Serial.println("Start check pin : ");
	//isPinFound = false;
	for(int i=0;i<12;i++){
		//Show row count
		if(i < 6) {
			print("NPN ROW",i+1);
			relay_on_npn(rl_pintable[i]);
			//tr_result(i);
		}
		else {
			print("PNP ROW",i-5);
			relay_on_pnp(rl_pintable[i]);
			//tr_result(i);
		}	
		tr_result(i);
	}			
	relay_off();
	tr_answer();
	//clear_gain();
}
/******************************************************************
/*
/*	Start curve tracer to processing
/*
/******************************************************************/
//NPN to processing
void st_curve_npn_to_processing(){
	//mcp4922.set_Volt_ChanelB(0.9);
	//float countIB = 0 , avgIB = 0;
	mcp4922.set_Volt_ChanelB(Vbb);
	for(float i = 0;i < 4096; i+= sizeCurve){	// Count 4095 / 45 = 91
		//Set Volt
		mcp4922.set_ChanelA(i);
		//delay(20);
		//Read Analog 0,1
		adc0_num = adc.analogReadXXbit(A0,12,25) ;	//VCC	-> AN1
		adc1_num = adc.analogReadXXbit(A1,12,25) ;	//VRC	-> AN2
		Vcc = (adc0_num / 4095) * 15;
		Vrc = (adc1_num / 4095) * 15;
		Ic = (Vcc-Vrc)/Rc;
		//Read Analog 2,3
		adc2_num = adc.analogReadXXbit(A2,12,25);	//VBB	-> AN3
		adc3_num = adc.analogReadXXbit(A3,12,25);	//VRB	-> AN4
		Vbb =  (adc2_num / 4095) * 15;
		Vrb =  (adc3_num / 4095) * 15;
		Ib = (Vbb-Vrb)/Rb;

		write_data(CMD_GET_VCC_ANALOG,adc0_num);
		write_data(CMD_GET_VRC_ANALOG,adc1_num);
		write_data(CMD_GET_VBB_ANALOG,adc2_num);		
		write_data(CMD_GET_VRB_ANALOG,adc3_num);
				
		//print("VBB",Vbb);
		//print("VRB",Vrb);
		//print("IB",Ib);
		//print("VCC",Vcc);
		//print("VRC",Vrc);
		//print("IC",Ic);

		Serial.println();
	}
	mcp4922.set_ChanelA(0);
	//delay(20);
}
//PNP to processing
void st_curve_pnp_to_processing(){
	mcp4922.set_Volt_ChanelB(Vbb);
	for(float i = 0;i < 4095; i+= 45){	// Count 4095 / 45 = 91
		//Set Volt
		mcp4922.set_ChanelA(i);
			
		digitalWrite(RD2,LOW);						//	Set gnd a / gnd vcc
		//Read Analog
		adc0_num = adc.analogReadXXbit(A0,12,25);	//VCC	-> AN1
		adc1_num = adc.analogReadXXbit(A1,12,25);	//VRC	-> AN2
		Vcc = (adc0_num / 4095) * 15;
		Vrc = (adc1_num / 4095) * 15;
		Ic = (Vcc-Vrc)/Rc;

		digitalWrite(RD2,HIGH);						//Set gnd a / gnd vcc
		adc2_num = adc.analogReadXXbit(A2,12,25);	//VBB	-> AN3
		adc3_num = adc.analogReadXXbit(A3,12,25);	//VRB	-> AN4
		Vbb = (adc2_num / 4095) * 15;
		Vrb = (adc3_num / 4095) * 15;
		Ib = (Vbb-Vrb)/Rb;

		write_data(CMD_GET_VCC_ANALOG,adc0_num);
		write_data(CMD_GET_VRC_ANALOG,adc1_num);
		write_data(CMD_GET_VBB_ANALOG,adc2_num);		
		write_data(CMD_GET_VRB_ANALOG,adc3_num);
		
		Serial.println();
	}
	//Clear Vcc = 0
	mcp4922.set_ChanelA(0);
}
/******************************************************************
/*
/*	Start curve tracer to adk
/*
/******************************************************************/
//Convert float to string
long getDecimal(float val){
	int intPart = int(val);
	long decPart = 1000*(val-intPart); //I am multiplying by 1000 assuming that the foat values will have a maximum of 3 decimal places
	//Change to match the number of decimal places you need
	if(decPart>0)return(decPart);           //return the decimal part of float number if it is available 
	else if(decPart<0)return((-1)*decPart); //if negative, multiply by -1
	else if(decPart=0)return(1);           //return 0 if decimal part of float number is not available
}
//Add volt to array buffer
void addSerialVoltSet(String msg){
	serialVoltSet[iCount] = msg;
	iCount++;
}
//Start for adk
void st_curve_npn_to_adk(){
	String stringVal = "";
	Serial.print("Start Curve:");	
	Serial.print(Vbb);		
	Serial.println();		
	mcp4922.set_Volt_ChanelB(Vbb);
	//delay(50);
	for(int i = 0;i < 4095; i+= sizeCurve){	// Count 4095 / 45 = 91
											// Count 4095 / 273 = 15
		//Set Volt
		mcp4922.set_ChanelA(i);
		//Read Analog
		adc0_num = adc.analogReadXXbit(A0,12,25);	//VCC	-> AN1
		adc1_num = adc.analogReadXXbit(A1,12,25);	//VRC	-> AN2
		adc2_num = adc.analogReadXXbit(A2,12,25);	//VBB	-> AN3
		adc3_num = adc.analogReadXXbit(A3,12,25);	//VRB	-> AN4

		char ch1[10];
		char ch2[10];
		char ch3[10];
		char ch4[10];
		dtostrf(adc0_num,8,2,ch1);
		dtostrf(adc1_num,8,2,ch2);
		dtostrf(adc2_num,8,2,ch3);
		dtostrf(adc3_num,8,2,ch4);

		//Convert float to string	
		//stringVal = "";
		//stringVal.concat("\t"+String(int(adc0_num))+ "."+String(getDecimal(adc0_num)));
		//stringVal.concat("\t"+String(int(adc1_num))+ "."+String(getDecimal(adc1_num)));
		//stringVal.concat("\t"+String(int(adc2_num))+ "."+String(getDecimal(adc2_num)));
		//stringVal.concat("\t"+String(int(adc3_num))+ "."+String(getDecimal(adc3_num)));

		stringVal = "";
		stringVal.concat("\t"+String(ch1));
		stringVal.concat("\t"+String(ch2));
		stringVal.concat("\t"+String(ch3));
		stringVal.concat("\t"+String(ch4));
		//Add string volt to string buffer array
		addSerialVoltSet(stringVal);		
		
		Serial.print(stringVal);	
		Serial.println();		
	}
	//digitalWrite(RD1,HIGH);
	//Clear Vcc = 0
	mcp4922.set_ChanelA(0);
	Serial.print("Read Package serialVoltSet");
	Serial.println();
	for(int i=0;i < sizeSerialVoltSet;i++){		
		write_droid(serialVoltSet[i]);
		Serial.print(serialVoltSet[i]);
		Serial.println();
	}
	Serial.print("Package Complete...");
	Serial.println();
}

void st_curve_pnp_to_adk(){
	String stringVal = "";
	Serial.print("Start Curve:");	
	Serial.print(Vbb);		
	Serial.println();		
	mcp4922.set_Volt_ChanelB(Vbb);
	for(int i = 0;i < 4095; i+= 45){	// Count 4095 / 45 = 91
		//Set Volt
		mcp4922.set_ChanelA(i);
		//Read Analog
		digitalWrite(RD2,LOW);						//	Set gnd a / gnd vcc
		//Read Analog
		adc0_num = adc.analogReadXXbit(A0,12,25);	//VCC	-> AN1
		adc1_num = adc.analogReadXXbit(A1,12,25);	//VRC	-> AN2
		
		digitalWrite(RD2,HIGH);						//Set gnd a / gnd vcc
		adc2_num = adc.analogReadXXbit(A2,12,25);	//VBB	-> AN3
		adc3_num = adc.analogReadXXbit(A3,12,25);	//VRB	-> AN4
	
		//adc0_num = floor(adc0_num);
		//adc1_num = floor(adc1_num);
		//adc2_num = floor(adc2_num);
		//adc3_num = floor(adc3_num);
		
		//Convert float to string	
		/*stringVal = "";
		stringVal.concat("\t"+String(int(adc0_num)));
		stringVal.concat("\t"+String(int(adc1_num)));
		stringVal.concat("\t"+String(int(adc2_num)));
		stringVal.concat("\t"+String(int(adc3_num)));*/
		stringVal = "";
		stringVal.concat("\t"+String(int(adc0_num))+ "."+String(getDecimal(adc0_num)));
		stringVal.concat("\t"+String(int(adc1_num))+ "."+String(getDecimal(adc1_num)));
		stringVal.concat("\t"+String(int(adc2_num))+ "."+String(getDecimal(adc2_num)));
		stringVal.concat("\t"+String(int(adc3_num))+ "."+String(getDecimal(adc3_num)));
		//Add string volt to string buffer array
		addSerialVoltSet(stringVal);		
		
		Serial.print(stringVal);	
		Serial.println();		
	}
	//digitalWrite(RD1,HIGH);
	//Clear Vcc = 0
	mcp4922.set_ChanelA(0);
	Serial.print("Read Package serialVoltSet");
	Serial.println();
	for(int i=0;i < sizeSerialVoltSet;i++){		
		write_droid(serialVoltSet[i]);
		Serial.print(serialVoltSet[i]);
		Serial.println();
	}
	Serial.print("Package Complete...");
	Serial.println();
}
/******************************************************************
/*
/*	Print serial string
/*
/******************************************************************/
void print(String name,float s){
	Serial.print(name);
	Serial.print(":");
	Serial.print(s,6);
	Serial.print("\t");
}
/******************************************************************
/*
/*	Clear gain count
/*
/******************************************************************/
void clear_gain(){
	maxGain_npn = 0;
	gain_npn = 0;
	bestConfig_npn = -1;

	maxGain_pnp = 0;
	gain_pnp = 0;
	bestConfig_pnp = -1;

	npn_neg = 0;
	pnp_neg = 0;
}
/******************************************************************
/*
/*	Transistor checking type npn
/*
/******************************************************************/
void relay_on_npn(String row){
	//Step 1	->		Set volt
	mcp4922.set_Volt_ChanelB(1);	//Vbb
	mcp4922.set_Volt_ChanelA(1);	//Vcc
	//Step2		->		Set relay on/off
	for(int i=0;i<row.length();i++){
		if(row[i] == '1'){
			digitalWrite(rl_num[i],HIGH);
		}else{
			digitalWrite(rl_num[i],LOW);			
		}
		delay(delay_npn);
	}	
	//Step3		->		Read analog
	int bitResoluton = 25;
	adc0_num = adc.analogReadXXbit(A0,12,bitResoluton);	//VCC	-> AN1
	adc1_num = adc.analogReadXXbit(A1,12,bitResoluton);	//VRC	-> AN2
	delay(10);
	adc2_num = adc.analogReadXXbit(A2,12,bitResoluton);	//VBB	-> AN3
	adc3_num = adc.analogReadXXbit(A3,12,bitResoluton);	//VRB	-> AN4
	delay(10);
	Vcc = (adc0_num / 4095) * 15;
	Vrc = (adc1_num / 4095) * 15;
	Ic = (Vcc-Vrc)/Rc;

	Vbb = (adc2_num / 4095) * 15;
	Vrb = (adc3_num / 4095) * 15;
	Ib = (Vbb-Vrb)/Rb;
	//Convert to mA
	//Ic = Ic * pow(10,3);
	//Ib = Ib * pow(10,5);

	gain_npn = Ic/Ib;
	if(Ib < 0.00001){
		gain_npn = -1;
	}

	if(Ib == 0.000001){
		gain_npn = -1;
		iCountNPN++;
	}

	if(gain_npn < 0){
		npn_neg++;
	}

	print("VBB",Vbb);
	print("VRB",Vrb);
	print("IB",Ib);
	print("VCC",Vcc);
	print("VRC",Vrc);
	print("IC",Ic);
	print("Gain",gain_npn);	

	
}
/******************************************************************
/*
/*	Transistor checking type pnp
/*
/******************************************************************/
void relay_on_pnp(String row){
	//Step 1	->		Set volt
	mcp4922.set_Volt_ChanelB(1);	//Vbb
	mcp4922.set_Volt_ChanelA(1);	//Vcc
	//Step2		->		Set relay on/off
	for(int i=0;i<row.length();i++){
		if(row[i] == '1'){
			digitalWrite(rl_num[i],HIGH);
		}else{
			digitalWrite(rl_num[i],LOW);
		}
		delay(delay_pnp);
	}
	//Set com+
	digitalWrite(RD5,HIGH);	//	Set divider com+
	digitalWrite(RD3,HIGH);	//	Set com+	
	digitalWrite(RD4,HIGH);		//	
	//**********************//
	digitalWrite(RD2,LOW);	//	Set gnd a / gnd vcc
	delay(10);
	//Read Analog
	int bitResoluton = 30;
	adc0_num = adc.analogReadXXbit(A0,12,bitResoluton);	//VCC	-> AN1
	adc1_num = adc.analogReadXXbit(A1,12,bitResoluton);	//VRC	-> AN2
	Vcc = (adc0_num / 4095) * 15;;
	Vrc = (adc1_num / 4095) * 15;
	Ic = (Vcc-Vrc)/Rc;

	digitalWrite(RD5,HIGH);	//	Set com+
	digitalWrite(RD3,HIGH);	//	Set com+	
	digitalWrite(RD4,HIGH);		//	Set com+
	//**********************//
	digitalWrite(RD2,HIGH);	//	Set gnd b / gnd vbb
	delay(10);
	adc2_num = adc.analogReadXXbit(A2,12,bitResoluton);	//VBB	-> AN3
	adc3_num = adc.analogReadXXbit(A3,12,bitResoluton);	//VRB	-> AN4
	Vbb =  (adc2_num / 4095) * 15;;
	Vrb = (adc3_num / 4095) * 15;
	Ib = (Vbb-Vrb)/Rb;
	//Convert to mA
	//Ic = Ic * pow(10,3);
	//Ib = Ib * pow(10,3);

	gain_pnp = Ic/Ib;
	if(Ib > 0 || Ib == 0.000001 || Ib > -0.000001){
		gain_pnp = -1;
	}

	if(Ib == 0){
		iCountPNP++;
	}

	if(isinf(gain_pnp)){
		gain_pnp = -1;
	}

	if(gain_pnp < 0){
		pnp_neg++;
	}

	print("VBB",Vbb);
	print("VRB",Vrb);
	print("IB",Ib);
	print("VCC",Vcc);
	print("VRC",Vrc);
	print("Ic",Ic);
	print("Gain",gain_pnp);
}
/******************************************************************
/*
/*	Transistor result type
/*
/******************************************************************/
void tr_result(int row){
	if(row < 6){
		if(gain_npn > maxGain_npn){
			maxGain_npn = gain_npn;
			bestConfig_npn = row;
		}
		print("MaxGain NPN",maxGain_npn);
		print("Best NPN",bestConfig_npn);
	}else{
		if(gain_pnp > maxGain_pnp){
			maxGain_pnp = gain_pnp;
			bestConfig_pnp = row;
		}
		print("MaxGain PNP",maxGain_pnp);
		print("Best PNP",bestConfig_pnp);
	}	
	Serial.println();
}
/******************************************************************
/*
/*	Transistor answer
/*
/******************************************************************/
void tr_answer(){
	//print("Gain NPN",gain_npn);
	//print("Gain PNP",gain_pnp);
	print("Count NPN NEG",npn_neg);
	print("Count PNP NEG",pnp_neg);
	Serial.println();
	String tr = "";
	if(iCountNPN == 6 || iCountPNP == 6){
		tr = "XXX:XXX:X";
	}else{
		if(npn_neg < pnp_neg){
			Serial.print("NPN Type ");
			Serial.print("\n");
			Serial.print(tr_pin[bestConfig_npn]);
			Serial.println();
			Serial.print("123");
			//Serial.println("NPN Best:"+bestConfig_npn);
			tr = "NPN:" + tr_pin[bestConfig_npn]+":"+bestConfig_npn;
			isNPN = true;
			isPNP = false;
		}else if(npn_neg > pnp_neg){
			Serial.print("PNP Type");
			Serial.print("\n");
			Serial.print(tr_pin[bestConfig_pnp-6]);		
			Serial.println();
			Serial.print("123");
			//Serial.println("PNP Best:"+bestConfig_pnp);
			tr = "PNP:" + tr_pin[bestConfig_pnp-6]+":"+(bestConfig_pnp-6);
			isNPN = false;
			isPNP = true;
		}
		else{
			tr = "XXX:XXX:X";
		}
	}
	//isPinFound = true;
	//if(isReady){
		//char ch[tr.length()+1];
		//tr.toCharArray(ch,sizeof(ch),0);
		//connection->writeString(ch);
		//delay(50);
		//ADB::poll();
		//delay(60);	
		write_droid(tr);
	//}
}
/******************************************************************
/*
/*	Transistor off
/*
/******************************************************************/
//relay off
void relay_off(){
	for(int i=0;i<sizeof(rl_num);i++){
		digitalWrite(rl_num[i],LOW);
		delay(10);
	}
	Serial.println("Relay off...");
}
//relay on
void relay_on(String row){
	for(int i=0;i<row.length();i++){
		if(row[i] == '1'){
			digitalWrite(rl_num[i],HIGH);
		}else{
			digitalWrite(rl_num[i],LOW);
		}
		delay(delay_npn);
	}
}
/******************************************************************
/*
/*	Testing mode
/*
/******************************************************************/

//void test_rl_num(){
//	//
//	mcp4922.set_Volt_ChanelB(1);
//	mcp4922.set_Volt_ChanelA(5);
//	//
//	digitalWrite(RL5,HIGH);
//	digitalWrite(RL4,HIGH);
//	//
//	digitalWrite(RL7,HIGH);
//	digitalWrite(RL11,HIGH);
//	//Read Analog
//	adc0_num = adc.analogReadXXbit(A0,12,25);	//VBB
//	adc2_num = adc.analogReadXXbit(A2,12,25);	//VCC
//	//Analog VRB , VRC Read
//	adc1_num = adc.analogReadXXbit(A1,12,25);	//VRB
//	adc3_num = adc.analogReadXXbit(A3,12,25);	//VRC
//	//Print
//	print("VBB",adc0_num);
//	print("VRB",adc1_num);
//	//print("IB",Ib);
//
//	print("VCC",adc3_num);
//	print("VRC",adc2_num);
//	//print("IC",Ic);
//
//	Serial.println();
//}

//void check_pin_pnp(){
//	mcp4922.set_Volt_ChanelB(1);
//	mcp4922.set_Volt_ChanelA(5);
//
//	if(Serial.available() > 0){
//
//		sRev = Serial.readStringUntil('\n');
//
//		if(sRev.equalsIgnoreCase("1")){
//			Serial.println("End : Pass key '0' ");
//
//			//String rl_row = rl_npn_pintable[0];			//NPN
//			String rl_row = rl_pnp_pintable[0];		//PNP
//
//			for(int i=0;i<rl_row.length();i++){
//				if(rl_row[i] == '1'){
//					digitalWrite(rl_num[i],HIGH);
//				}else{
//					digitalWrite(rl_num[i],LOW);
//				}
//				delay(50);
//			}
//		}else{
//			for(int i=0;i<16;i++){
//				digitalWrite(rl_num[i],LOW);
//				delay(10);
//			}
//			Serial.println("Start : Pass key '1' ");
//		}
//	}else{
//		//Set com+
//		digitalWrite(RD1,HIGH);	//	Set com+
//		digitalWrite(RD2,LOW);	//	Set gnd a / gnd vcc
//		delay(10);
//		//Read Analog
//		adc0_num = adc.analogReadXXbit(A0,12,25);	//VCC	-> AN1
//		adc1_num = adc.analogReadXXbit(A1,12,25);	//VRC	-> AN2
//		Vcc = (adc0_num / 4095) * 15;;
//		Vrc = (adc1_num / 4095) * 15;
//		Ic = (Vcc-Vrc)/Rc;
//
//		digitalWrite(RD1,HIGH);	//	Set com+
//		digitalWrite(RD2,HIGH);	//	Set gnd a / gnd vcc
//		delay(10);
//		adc2_num = adc.analogReadXXbit(A2,12,25);	//VBB	-> AN3
//		adc3_num = adc.analogReadXXbit(A3,12,25);	//VRB	-> AN4
//		Vbb =  (adc2_num / 4095) * 15;;
//		Vrb = (adc3_num / 4095) * 15;
//		Ib = (Vbb-Vrb)/Rb;
//
//		gain = Ic/Ib;
//		//Print
//		print("VBB",Vbb);
//		print("VRB",Vrb);
//		//print("IB",Ib);
//
//		print("VCC",Vcc);
//		print("VRC",Vrc);
//
//		print("IB",Ib);
//		print("Ic",Ic);
//		print("Gain",gain);
//
//		Serial.println();
//		delay(1000);
//	}
//}

//void check_pin(){
//
//	mcp4922.set_Volt_ChanelB(1);
//	mcp4922.set_Volt_ChanelA(5);
//
//	if(Serial.available() > 0){
//
//		sRev = Serial.readStringUntil('\n');
//
//		if(sRev.equalsIgnoreCase("1")){
//			Serial.println("End : Pass key '0' ");
//
//			String rl_row = rl_npn_pintable[0];			//NPN
//			//String rl_row = rl_pnp_pintable[0];		//PNP
//
//			for(int i=0;i<rl_row.length();i++){
//				if(rl_row[i] == '1'){
//					digitalWrite(rl_num[i],HIGH);
//				}else{
//					digitalWrite(rl_num[i],LOW);
//				}
//				delay(50);
//			}
//		}else{
//			for(int i=0;i<16;i++){
//				digitalWrite(rl_num[i],LOW);
//				delay(10);
//			}
//			Serial.println("Start : Pass key '1' ");
//		}
//	}else{
//		//Read Analog
//		adc0_num = adc.analogReadXXbit(A0,12,25);	//VCC	-> AN1
//		adc1_num = adc.analogReadXXbit(A1,12,25);	//VRC	-> AN2
//
//		adc2_num = adc.analogReadXXbit(A2,12,25);	//VBB	-> AN3
//		adc3_num = adc.analogReadXXbit(A3,12,25);	//VRB	-> AN4
//
//		Vcc = (adc0_num / 4095) * 15;
//		Vrc = (adc1_num / 4095) * 15;
//		Ic = (Vcc-Vrc)/Rc;
//
//		Vbb = (adc2_num / 4095) * 15;
//		Vrb = (adc3_num / 4095) * 15;
//		Ib = (Vbb-Vrb)/Rb;
//
//		gain = Ic/Ib;
//		//Print
//		print("VBB",Vbb);
//		print("VRB",Vrb);
//		//print("IB",Ib);
//
//		print("VCC",Vcc);
//		print("VRC",Vrc);
//
//		print("IB",Ib);
//		print("Ic",Ic);
//		print("Gain",gain);
//
//		Serial.println();
//	}
//}

//void test_pin(){
//	mcp4922.set_Volt_ChanelB(1);
//	mcp4922.set_Volt_ChanelA(5);
//	
//	Serial.println("End : Pass key '0' ");
//	String rl_row = rl_npn_pintable[0];			//NPN
//	//String rl_row = rl_pnp_pintable[0];		//PNP
//
//	for(int i=0;i<rl_row.length();i++){
//		if(rl_row[i] == '1'){
//			digitalWrite(rl_num[i],HIGH);
//		}else{
//			digitalWrite(rl_num[i],LOW);
//		}
//		delay(500);
//	}
//
//}



//float calibrate_analog(float data, float filterVal, float smoothedVal){
//	smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
//	return smoothedVal;
//}

//void calibrate(){
//
//	for(float i=0;i<16.0;i+=1){
//
//		relay_on(rl_pintable[5]);
//
//		//digitalWrite(RL16,HIGH);
//		//digitalWrite(RL5,HIGH);
//		//digitalWrite(RL4,HIGH);
//
//		mcp4922.set_Volt_ChanelA(i);
//		//mcp4922.set_Volt_ChanelA(2);
//		//mcp4922.set_ChanelB(value);
//		//mcp4922.set_ChanelA(value);
//
//		//Step3		->		Read analog
//		adc0_num = adc.analogReadXXbit(A0,12,30);	//VCC	-> AN1
//		adc1_num = adc.analogReadXXbit(A1,12,30);	//VRC	-> AN2
//		
//		adc2_num = adc.analogReadXXbit(A2,12,30);	//VBB	-> AN3
//		adc3_num = adc.analogReadXXbit(A3,12,30);	//VRB	-> AN4
//		
//		Vcc = (adc0_num / 4095) * 15;
//		Vrc = (adc1_num / 4095) * 15;
//		Ic = (Vcc-Vrc)/Rc;
//
//		Vbb = (adc2_num / 4095) * 15;
//		Vrb = (adc3_num / 4095) * 15;
//		Ib = (Vbb-Vrb)/Rb;
//
//		//print("AN0",adc0_num);
//		//print("AN1",adc1_num);
//		//print("AN3",adc3_num);
//		//print("AN2",adc2_num);
//
//		print("",Vbb);
//		//print("VRB",Vrb);
//		//print("IB",Ib);
//
//		print("",Vcc);
//		//print("VRC",Vrc);
//		//print("IC",Ic);
//		print("",i);
//
//		Serial.println();
//	}
//	relay_off();
//	//digitalWrite(RL16,LOW);
//	//digitalWrite(RL5,LOW);
//	//digitalWrite(RL4,LOW);
//}





