/**
 * FreeIMU library serial communication protocol
 * TODO: ROT 
 * Vessels Attitude ($xxXDR or $xxNTHPR) would be a great bonus!"
 *
*/

#include <ADXL345.h>
#include <bma180.h>
#include <HMC58X3.h>
#include <ITG3200.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>
#include <FlexiTimer2.h>
#include <AverageList.h>
#include <PString.h>

//#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>

/**mag declination*/
#define DECL  "DEC"

volatile boolean execute = false;
volatile int interval = 0;
float q[4];
float qm[7];
//float m[3];
int raw_values[11];
float ypr[3]; // yaw pitch roll
float yprm[4]; // yaw, pitch, roll, mag heading
char str[256];
float val[9];
float declination=0.0;
String inputSerial = ""; // a string to hold incoming data

typedef volatile float rval; //change float to the datatype you want to use
const byte MAX_NUMBER_OF_READINGS = 5;
rval mghStorage[MAX_NUMBER_OF_READINGS] = {0.0};
AverageList<rval> mghList = AverageList<rval>( mghStorage, MAX_NUMBER_OF_READINGS );

// rate of turn average
rval rotStorage[MAX_NUMBER_OF_READINGS] = {0.0};
AverageList<rval> rotList = AverageList<rval>( rotStorage, MAX_NUMBER_OF_READINGS);


// Set the FreeIMU object
FreeIMU my3IMU = FreeIMU();
//The command from the PC
char cmd;

/*
 * Timer interrupt driven method to do time sensitive calculations
 * The calc flag causes the main loop to execute other less sensitive calls.
 */
void calculate() {
	//we create 100ms pings here
	execute = true;
	//we record the ping count out to 2 secs
	interval++;
	interval = interval % 20;
}

void setup() {
	inputSerial.reserve(40);
  Serial.begin(38400);
  Wire.begin();
  
  my3IMU.init(true);
  
  //setup timers
  FlexiTimer2::set(100, calculate); // 100ms period
  FlexiTimer2::start();
  // LED
  pinMode(13, OUTPUT);
  mghList.reset();
}

/*
 SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
	while (Serial.available()) {
		// get the new byte:
		char inChar = (char) Serial.read();
		// add it to the inputString:
		inputSerial += inChar;
		if (inChar == '\n') {
			//inputSerialComplete = true;
			char carray[inputSerial.length() + 1]; //determine size of the array
			inputSerial.toCharArray(carray, sizeof(carray));
			process(carray, ',');
			inputSerial = "";
			//inputSerialComplete = false;
		}

	}
}
void loop() {
 
  if (execute) {
    //do these every 100ms
    //quarternary, updates AHRS
    my3IMU.getQ(q);
    if (interval % 2 == 0) {
      // do every 200ms
          //mag 
          my3IMU.getValues(val);
       
          //convert to YPR
          my3IMU.getYawPitchRollRad(yprm);
          //convert to magnetic heading
          calcMagHeading();
          updateROT();
          mghList.addValue(yprm[3]);
    }
    if (interval % 5 == 0) {
	//do every 500ms
         float h = degrees(mghList.getTotalAverage());
          if(h<0.0){
            h=(360.0+h);
          }
         //ArduIMU output format
        printIMU(h);
          //now do the NMEA version
         
          printNmeaMag(h);
           printRateOfTurn();
        }
        
        execute = false;
  }
   
    
  
}

void process(char * s, char parser) {
	//if (DEBUG) Serial.print("Process str:");
	//if (DEBUG) Serial.println(s);
	char *cmd = strtok(s, ",");
	while (cmd != NULL && strlen(cmd) > 3) {
		//starts with # its a command
		//if (DEBUG) Serial.print("Process incoming..l=");
		//if (DEBUG) Serial.print(strlen(cmd));
		//if (DEBUG) Serial.print(", ");
		//if (DEBUG) Serial.println(cmd);

		char key[5];
		int l = strlen(cmd);

		if (cmd[0] == '#') {
			//
			strncpy(key, cmd, 4);
			key[4] = '\0';
			char valArray[l - 4];
			memcpy(valArray, &cmd[5], l - 5);
			valArray[l - 5] = '\0';
			//if (DEBUG) Serial.print(key);
			//if (DEBUG) Serial.print(" = ");
			//if (DEBUG) Serial.println(valArray);

		} else {
			strncpy(key, cmd, 3);
			key[3] = '\0';
			char valArray[l - 3];
			memcpy(valArray, &cmd[4], l - 4);
			valArray[l - 4] = '\0';
			//if (DEBUG) Serial.print(key);
			//if (DEBUG) Serial.print(" = ");
			//if (DEBUG) Serial.println(valArray);
			
			if (strcmp(key, DECL) == 0) {
				declination= atof(valArray);
			}
		}
		//next token
		cmd = strtok(NULL, ",");
	}
	//if (DEBUG) Serial.println("Process str exit");
}

void printIMU(float h){
  //!!!VER:1.9,RLL:-0.52,PCH:0.06,YAW:80.24,IMUH:253,MGX:44,MGY:-254,MGZ:-257,MGH:80.11,LAT:-412937350,LON:1732472000,ALT:14,COG:116,SOG:0,FIX:1,SAT:5,TOW:22504700***
         
          Serial.print("!!!VER:1.9,UID:IMU,");
          Serial.print("MGH:");
          
          Serial.print(h);
          Serial.print(",YAW:");
          Serial.print(degrees(yprm[0]));
          Serial.print(",PCH:");
          Serial.print(degrees(yprm[1]));
          Serial.print(",RLL:");
          Serial.print(degrees(yprm[2]));
          Serial.println(","); 
  
}
/*


        1   2 3
        |   | |
 $--ROT,x.x,A*hh<CR><LF>

Field Number:

    Rate Of Turn, degrees per minute, "-" means bow turns to port

    Status, A means data is valid

    Checksum
   */
   
void printRateOfTurn(){
  
   char rotSentence [25];

	PString str(rotSentence, sizeof(rotSentence));
	str.print("$TIROT,");
        float r =rotList.getTotalAverage();
        char rBuf[5];
        dtostrf(r, 3, 1, rBuf);
	str.print(rBuf); //prints 1 decimal, but round to degree, should be good enough
	str.print(",A*");
	
	//calculate the checksum

	int cs = 0; //clear any old checksum
	for (unsigned int n = 1; n < strlen(rotSentence) - 1; n++) {
		cs ^= rotSentence[n]; //calculates the checksum
	}
	str.print(cs, HEX); // Assemble the final message and send it out the serial port
	Serial.println(rotSentence);
        //val[] 3,4,5
         //Serial.print(",gyrox:");
         // Serial.print(degrees(val[3]));
          //Serial.print(",gyroy:");
          //Serial.print(degrees(val[4]));
          //Serial.print(",gyroz:");
          //Serial.println(degrees(val[5])*0.06097);
}
   
/*
 Print out the NMEA string for mag heading
 $HC = mag compass
            === HDM - Heading - Magnetic ===

            Vessel heading in degrees with respect to magnetic north produced by
            any device or system producing magnetic heading.
            
            ------------------------------------------------------------------------------
                    1   2 3
                    |   | |
             $--HDM,x.x,M*hh<CR><LF>
            ------------------------------------------------------------------------------
            
            Field Number: 
            
            1. Heading Degrees, magnetic
            2. M = magnetic
            3. Checksum
            
*/
void printNmeaMag(float h){
    //Assemble a sentence of the various parts so that we can calculate the proper checksum
    char magSentence [20];

	PString str(magSentence, sizeof(magSentence));
	str.print("$HCHDM,");

	str.print((int)h); //prints 1 decimal, but round to degree, should be good enough
	str.print(".0,M*");
	
	//calculate the checksum

	int cs = 0; //clear any old checksum
	for (unsigned int n = 1; n < strlen(magSentence) - 1; n++) {
		cs ^= magSentence[n]; //calculates the checksum
	}
	str.print(cs, HEX); // Assemble the final message and send it out the serial port
	Serial.println(magSentence);
        //do HDT if we have a declination
        if(declination!=0.0){
            //print HDT
            char hdtSentence [20];
                PString str(hdtSentence, sizeof(hdtSentence));
        	str.print("$HCHDT,");
                float d = h-declination;
                char dBuf[5];
                dtostrf(d, 3, 1, dBuf);
        	str.print(dBuf); 
        	str.print(",T*");
        	
        	//calculate the checksum
        
        	int cs = 0; //clear any old checksum
        	for (unsigned int n = 1; n < strlen(hdtSentence) - 1; n++) {
        		cs ^= hdtSentence[n]; //calculates the checksum
        	}
        	str.print(cs, HEX); // Assemble the final message and send it out the serial port
        	Serial.println(hdtSentence);  
                
                //and do HDG (Heading, Deviation and Variation)
                /*
                ------------------------------------------------------------------------------
                        1   2   3 4   5 6
                        |   |   | |   | |
                 $--HDG,x.x,x.x,a,x.x,a*hh<CR><LF>
                ------------------------------------------------------------------------------
                
                Field Number: 
                
                1. Magnetic Sensor heading in degrees
                2. Magnetic Deviation, degrees
                3. Magnetic Deviation direction, E = Easterly, W = Westerly, direction of magnetic north from true north.
                4. Magnetic Variation degrees
                5. Magnetic Variation direction, E = Easterly, W = Westerly
                6. Checksum
                
                */
                char hdgSentence [30];
                float x = (abs(declination));
                char xBuf[5];
                dtostrf(x, 3, 1, xBuf);
                //x=((int)(x*10))*0.1;
                PString hdgStr(hdgSentence, sizeof(hdgSentence));
        	hdgStr.print("$HCHDG,");
                hdgStr.print(h);//mag heading 
        	hdgStr.print(",");
                hdgStr.print(xBuf);//declination
                if(declination<0.0){
                  hdgStr.print(",E,");
                }else{
                  hdgStr.print(",W,");
                }
                hdgStr.print(xBuf);//declination
                if(declination<0.0){
                  hdgStr.print(",E*");
                }else{
                  hdgStr.print(",W*");
                }
        	
        	//calculate the checksum
        
        	cs = 0; //clear any old checksum
        	for (unsigned int n = 1; n < strlen(hdgSentence) - 1; n++) {
        		cs ^= hdgSentence[n]; //calculates the checksum
        	}
        	hdgStr.print(cs, HEX); // Assemble the final message and send it out the serial port
        	Serial.println(hdgSentence);  
                
        }

  
}

void updateROT(){
  //executed every 0.2 secs
     //value is deg/s
     float rot=(val[5]*0.06097);
     rotList.addValue(-degrees(rot));
}

void calcMagHeading(){
  float Head_X;
  float Head_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(-yprm[2]);
  sin_roll = sin(-yprm[2]);
  cos_pitch = cos(yprm[1]);
  sin_pitch = sin(yprm[1]);
  
  //Example calc
  //Xh = bx * cos(theta) + by * sin(phi) * sin(theta) + bz * cos(phi) * sin(theta)
  //Yh = by * cos(phi) - bz * sin(phi)
  //return wrap((atan2(-Yh, Xh) + variation))
    
  // Tilt compensated Magnetic field X component:
  Head_X = val[6]*cos_pitch+val[7]*sin_roll*sin_pitch+val[8]*cos_roll*sin_pitch;
  // Tilt compensated Magnetic field Y component:
  Head_Y = val[7]*cos_roll-val[8]*sin_roll;
  // Magnetic Heading
  yprm[3] = atan2(-Head_Y,-Head_X);
  
}

char serial_busy_wait() {
  while(!Serial.available()) {
    ; // do nothing until ready
  }
  return Serial.read();
}

const int EEPROM_MIN_ADDR = 0;
const int EEPROM_MAX_ADDR = 511;

void eeprom_serial_dump_column() {
  // counter
  int i;

  // byte read from eeprom
  byte b;

  // buffer used by sprintf
  char buf[10];

  for (i = EEPROM_MIN_ADDR; i <= EEPROM_MAX_ADDR; i++) {
    b = EEPROM.read(i);
    sprintf(buf, "%03X: %02X", i, b);
    Serial.println(buf);
  }
}
