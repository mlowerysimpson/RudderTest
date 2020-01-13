#include "../RemoteControlTest/Rudder.h"
#include "../RemoteControlTest/AToD.h"
#include "../RemoteControlTest/ShipLog.h"
#include <pthread.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <memory>

//example program that lets you test out the air rudder(s) connected to a DS3218MG Digital Servo
#define ATOD_ADDRESS 0x48 //I2C address of AtoD
#define MAX_ANGLES 100//maximum # of angles to input on the command line

using namespace std;
pthread_mutex_t g_i2cMutex;//mutex for making sure remote commands do not interfere with one another

/**
 * @brief Get the angle values that the user entered on the command line using the -a flag
 * 
 * @param fAngles the angles that this function parses are returned in the fAngles floating point array
 * @param nNumAnglesFound the number of angles that could be parsed 
 * @param argc the number of command line arguments that the user specified on the command line when calling this program
 * @param argv an array of character pointers that corresponds to the program arguments
 * @return true if the user successfully specified one or more rudder angles in the program arguments following the -a flag.
 * @return false if an error occurred.
 */
bool GetAngles(float fAngles[], int &nNumAnglesFound, int argc, char * argv[]) {
	bool bFoundAngleFlag=false;
	float fMinAngle = MIN_RUDDER_ANGLE;
	float fMaxAngle = MAX_RUDDER_ANGLE;
	float fAngle=0;
	int i=0;
	nNumAnglesFound=0;
	for (i=0;i<(argc-1);i++) {
		if (strlen(argv[i])<2) continue;
		if (strncmp(argv[i],"-a",2)==0) {
			bFoundAngleFlag=true;
			break;
		}
		else if (strncmp(argv[i],"-A",2)==0) {
			bFoundAngleFlag=true;
			break;
		}
	}
	if (bFoundAngleFlag) {
		for (int j=i+1;j<argc;j++) {
			if (sscanf(argv[j],"%f",&fAngle)<1) {
				//unable to parse angle
				return (nNumAnglesFound>0);
			}
			if (fAngle<MIN_RUDDER_ANGLE) {
				printf("Invalid angle, must be >= %.1f.\n",fMinAngle);
				return false;
			}
			else if (fAngle>MAX_RUDDER_ANGLE) {
				printf("Invalid angle, must be < %.1f.\n",fMaxAngle);
				return false;
			}
			fAngles[nNumAnglesFound]=fAngle;
			nNumAnglesFound++;
			if (nNumAnglesFound>=MAX_ANGLES) {
				break;
			}
		}
		return (nNumAnglesFound>0);
	}
	return false;//the angle flag was not specified	
}

//isShowVoltageFlagPresent: return true if the flag for showing voltage was specified in the program arguments
//argc: the number of program arguments
//argv: an array of character pointers that corresponds to the program arguments
//returns true if the flag for showing voltage (-v) is present, otherwise returns false
bool isShowVoltageFlagPresent(int argc, char * argv[]) {
	for (int i=0;i<argc;i++) {
		if (strlen(argv[i])<2) continue;
		if (strncmp(argv[i],"-v",2)==0) {
			return true;
		}
		else if (strncmp(argv[i],"-V",2)==0) {
			return true;
		}
	}
	return false;
}

/**
 * @brief checks to see if the flag for adding a delay (in seconds) between rudder movements was specified in the program arguments.
 * 
 * @param argc the number of program arguments.
 * @param argv an array of character pointers that corresponds to the program arguments.
 * @return the index of the program argument that corresponds to the delay in seconds, if the -d flag was specified, or -1 if no delay flag was specified
  */
int isDelayFlagPresent(int argc, char *argv[]) {
	for (int i=0;i<argc;i++) {
		if (strlen(argv[i])<2) continue;
		if (strncmp(argv[i],"-d",2)==0) {
			if (i<(argc-1)) {
				return i+1;
			}
		}
		else if (strncmp(argv[i],"-D",2)==0) {
			if (i<(argc-1)) {
				return i+1;
			}
		}	
	}
	return false;
}

void ShowUsage(float fMinAngle, float fMaxAngle) {//explain how to use this program
	printf("RudderTest allows you to test the air rudder(s) at one or more angles.\n");
	printf("Usage: RudderTest <-a angle1 angle2 angle3... > [-V] [-d time_seconds]\n");
	printf("-a sets the angle(s) of the rudder to use for the test. Each angle specified can vary from %.1f to %.1f degrees.\n",fMinAngle,fMaxAngle); 
	printf("-V flag (optional) outputs voltage information while the program is running.\n");
	printf("-d sets an optional delay in seconds between rudder movements.\n");
}

int main(int argc, char * argv[])
{
  ShipLog shipLog;
  g_i2cMutex = PTHREAD_MUTEX_INITIALIZER;
  float fMinAngle = MIN_RUDDER_ANGLE;
  float fMaxAngle = MAX_RUDDER_ANGLE;
  if (argc<3) {
	  ShowUsage(fMinAngle,fMaxAngle);
	  return -1;
  }
  float fAngles[MAX_ANGLES];
  int nNumAnglesSpecified=0;
  if (!GetAngles(fAngles, nNumAnglesSpecified, argc, argv)) {
	 printf("Error, a valid angle was not specified with the -a flag.\n");
	 ShowUsage(fMinAngle, fMaxAngle);
	 return -3;
  }
  bool bVoltageOutput = isShowVoltageFlagPresent(argc, argv);	
  int nDelayIndex = isDelayFlagPresent(argc, argv);
  float fDelayTimeSec = 0;
  if (nDelayIndex>0) {
	  sscanf(argv[nDelayIndex],"%f",&fDelayTimeSec);
  }
  AToD *pAToD = nullptr;
  double dBatteryVoltage =0.0;
  if (bVoltageOutput) {	
	char *i2c_filename = (char*)"/dev/i2c-1";
	pAToD = new AToD(i2c_filename,ATOD_ADDRESS,&shipLog, &g_i2cMutex);
  }
  
  Rudder::OneTimeSetup();//do I/O setup for propeller (once per power-cycle)
  Rudder *pRudder = new Rudder();
     
  const int LOOP_DELAY = 1000;//delay in ms to add after each angle
  for (int i=0;i<nNumAnglesSpecified;i++) {
	delay(fDelayTimeSec*1000);
  	printf("Setting angle to %.1f degrees.\n", fAngles[i]);  
  	pRudder->SetAngle(0,fAngles[i]);
    if (bVoltageOutput) {
	  if (pAToD->GetBatteryVoltage(dBatteryVoltage,nullptr)) {
		  //printf("Voltage = %.3f V\n",dBatteryVoltage);
	  }
	}
	delay(LOOP_DELAY);
  }
  delete pRudder;
  pRudder=nullptr;
  if (pAToD) {
	  delete pAToD;
	  pAToD=nullptr;
  }
  return 0 ;
}
