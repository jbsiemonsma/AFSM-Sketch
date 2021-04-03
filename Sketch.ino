
// V38 Standaard Arduino application by J.B. Siemonsma
//
// Arduino Finite State Machine
// Copyright(C) 2018 Jelle Siemonsma
// 
// This program is free software : you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
// GNU General Public License for more details.
//
// < https://www.gnu.org/licenses/>.
//
//
//   MM          MM   EEEEEEEEEE     GGGGGGGG        AAAAAA
//   MMMM      MMMM   EE           GGG      GG      AA    AA
//   MM  MM  MM  MM   EE          GG               AA      AA
//   MM    MM    MM   EEEEEE      GG      GGGGGG  AA        AA
//   MM          MM   EE          GG          GG  AAAAAAAAAAAA
//   MM          MM   EE           GGG      GGG   AA        AA
//   MM          MM   EEEEEEEEEE     GGGGGGGG     AA        AA
//
// Supporting:
// - hardware Arduino MEGA 2560
// - initialization of digital inputs (pullup used, so switch to gnd)
// - initialization of digital ouputs
// - initialization of analog inputs
// - initialization of analog outputs (PWN, puls width modulation)
// - initialization of servos
// - Visual control of a working/checked configuration (Flashing LED PIN 13)
// - Help functions in the serial monitor (use "h" for help)
// - cyclic reading of defined digital inputs, with current status, timestamp, change notification; e.g. usefull for phase transitions;
//   if the status of an input changes in cycle n, it is notified by field "changed", cycle n+1, the field "changed" is reset again.
// - cyclic reading of defined analog inputs
// - calculated cycle time, can be visualised in the help function
// - asynchronous timer function, waiting without using "delay" function. Made up by pointerstructure.
// - function for setting digital outputs
// - function for setting analog outputs
// - example of interrupt handling (see also , loop and the Isr function)
// - available functions to determine and check bits by there tagname
// - available functions to determine and check analogue values by there tagname
// - available function to send messages by I2C (e.g. to the SMS Alarm sketch or the LCD display sketch).
// - serial communication for HMI functionality, see also the documentation
// - RTC, Real time clock for setting the system time
// - One wire channel for reading e.g. temperature(s)
//
//
// All included libraries and constants, do not remove or change
//

#include "UserConfiguration.h"
#include <TimeLib.h>
#include <RTClib.h>
#include <Servo.h>
#include <Wire.h>
#include <MemoryFree.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
//#include <PID_v1.h>
//#include <AccelStepper.h>
//
#define TIME_MSG_LEN       11  // time sync to PC is HEADER followed by unix time_t as ten ascii digits
#define TIME_HEADER        'T' // Header tag for serial time sync message
#define TIME_REQUEST       7   // ASCII bell character requests a time sync message 

#define StartDigitalPins   22  //Start of the pin layout for the Digital pins
#define EndDigitalPins     53  //End of the pin layout for the digital pins
boolean LayoutDigital[EndDigitalPins - StartDigitalPins + 1]; //Check array digital pins

#define StartPWMPins        2  //Start of the pin layout for the PWM pins
#define EndPWMPins          13 //End of the pin layout for the PWM Pins
boolean LayoutPWM[EndPWMPins - StartPWMPins]; // Check array PWM pins

#define StartAnalogInPins   0  //Start of the pin layout for the analogue in pins
#define EndAnalogInPins    15  //End of the pin layout for the analogue in pins
boolean LayoutAnalogInPins[EndAnalogInPins - StartAnalogInPins]; // check array analogue in Pins

//
// Predefined timer id's, do not remove!
//
const byte TimOneHour    = 200;
const byte TimISR        = 201;
const byte TimUS         = 202;
const byte TimTT         = 203;
const byte TimDelayGMI   = 204;
const byte TimHMIWtd     = 205;
const byte TimBlinking   = 206;
const byte TimBlinkWtdOn = 207;
const byte TimBlinkWtdOff= 208;
const byte TimGpsPoll    = 209;
//
// Temperature sensors. Do not remove.
//
OneWire oneWire(OneWireChannel);
DallasTemperature sensors(&oneWire);
//
// Real time, type clock DS1307
RTC_DS1307 RealTimeClock;
//
// Stepper
//AccelStepper Spindle(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
//
// Interrupt Pin numbers (do not remove/change)
//int Pin2  = 0;
//int Pin3  = 1;
//int Pin21 = 2; SCL
//int Pin20 = 3; SCA
//int Pin19 = 4;
//int Pin18 = 5;
//
//
//reserved variables, do not remove/change
//
int                    Pin13    = 0;
int                    Blinking = 500;
byte                   incomingByte;
boolean                CConfigurationOK;
boolean                OnOff;
time_t                 tijd;
boolean                RTCAvailable;
unsigned long          CycleStart;
unsigned long          NoOfCycle;
boolean                CycleCalc;
boolean                CycleCalcHMI;
unsigned int           PWN_value;
String                 OldMessageI2C;
char                   WatchdogI2C = 5;
volatile unsigned long ISR_Count;
unsigned long          ISR_Count_Copy;
int                    Threads = 1;
int                    CurrentThread = 0;
int                    NoFSMStates = 0;
int                    MachineState = 0;
char                   Karakter = -1;
String                 KarakterString = "";
String                 GPSString = "";
String                 IntercardString = "";
int                    KarakterIndex = 0;
int                    Getal;
int                    NoHandTags = 0;
int                    NoMaskTags = 0;
String                 ValidationId;
String                 DummyCommands [] = {"DUMMY"};

String                 HMICommands [] = { "SDI", "SAI", "FDO", "FAO", "RTA", "PDI", "PAI", "CAN", "RDC", "GMI", 
//                                           0      1      2      3      4      5      6      7      8      9
                                          "MDI", "MAI", "RMM", "FSM", "FSO", "WTD", "PMT", "FMT", "MTC", "THM",
//                                        	10     11     12     13     14     15     16     17     18     19
                                          "CLT", "RAM", "TIM", "JM0", "RDB", "CDB", "CHM" };
//                                          20     21     22     23     24     25     26
int                    NoHMICommands = 26+1;

String                 GPSCommands[] = { "PUBX"};
//                                           0 
int                    NoGPSCommands = 0 + 1;

String                 IntercardCommands[] = { "CHAR01", "CHAR10", "INT001", "INT003" };
//                                                   0         1         2         3
int                    NoIntercardCommands = 3 + 1;
String                 IntercardChar01;
String                 IntercardChar10;
String                 IntercardInt001;
String                 IntercardInt010;

char                   LF = 10;
char                   CR = 13;
boolean                DelayGMI = false;
boolean                HMIWatchDog;
unsigned long          HMIWatchDogTime;


struct FSMStateType
{
  String         FSMStateName;
  int            FSMStateNo;
  String         ActualState;
  boolean        Enter;
  boolean        Exit;
  int            ThreadNo;
  unsigned long  StartTime;
  unsigned long  TotalTime;
  struct         FSMStateType* Next;
};
FSMStateType FSMState;
FSMStateType* FSMStateKetting;
FSMStateType* FSMStateSchakel;

struct TimerType
{
  int           TimerId;
  unsigned long Initial;
  unsigned long Wait;
  boolean       State;
  boolean       OneShot;
  struct        TimerType* Next;
  struct        TimerType* Previous;
};
TimerType TimerSchakel;
TimerType* TimerKetting;
TimerType* HuidigeSchakel;
TimerType* NieuweSchakel;
TimerType* VorigeSchakel;
int AantalSchakels;

struct MessageType
{
  String        Message;
  unsigned long SendTime;
  struct        MessageType* Next;
  struct        MessageType* Previous;
};
MessageType MessageSchakel;
MessageType* MessageKetting;
MessageType* HuidigeMessage;
MessageType* NieuweMessage;
MessageType* VorigeMessage;
MessageType* VolgendeMessage;

typedef struct DigInPinType
{
  int           Pin;
  boolean       Status;
  unsigned long TimeChange;
  boolean       Changed;
  String        Tagname;
  boolean       Report;
  boolean       Poll;
  unsigned long PollFreq;
  unsigned long PollTime;
  boolean       Mask;
  boolean       MaskValue;
  boolean       DBLogging;
};
DigInPinType InPin[NoInputPins + 1];

typedef struct DigOutPinType
{
  int           Pin;
  boolean       Status;
  boolean       PreStatus;
  unsigned long TimeChange;
  boolean       FixedStatus;
  boolean       Changed;
  String        Tagname;
  boolean       Report;
  boolean       Poll;
  unsigned long PollFreq;
  unsigned long PollTime;
  boolean       Auto;
  boolean       DBLogging;
} ;
DigOutPinType OutPin[NoOutputPins + 1];

typedef struct TempSensorType
{
  int           Channel;
  int           DeviceNr;
  float         Value;
  float         PrevValue;
  unsigned long TimeChange;
  boolean       Changed;
  String        Tagname;
  boolean       Poll;
  unsigned long PollFreq;
  unsigned long PollTime;
  boolean       Mask;
  boolean       DBLogging;
} ;
TempSensorType TempSensor[NoTemp + 1];

typedef struct AnaInPinType
{
  int           Pin;
  unsigned int  Value;
  unsigned int  PrevValue;
  unsigned long TimeChange;
  boolean       Changed;
  String        Tagname;
  boolean       Poll;
  unsigned long PollFreq;
  unsigned long PollTime;
  boolean       Mask;
  boolean       DBlogging;
} ;
AnaInPinType AnaInPin[NoAnaInPins + 1];

typedef struct AnaOutPinType
{
  int Pin;
  unsigned int  Value;
  unsigned int  PrevValue;
  unsigned long TimeChange;
  unsigned int  FixedValue;
  boolean       Changed;
  String        Tagname;
  boolean       Poll;
  unsigned long PollFreq;
  unsigned long PollTime;
  boolean       Auto;
  boolean       DBLogging;
} ;
AnaOutPinType AnaOutPin[NoAnaOutPins + 1];

typedef struct MyServoType
{
  int           Pin;
  int           Angle;
  Servo         MyServo;
  unsigned long TimeChange;
  int           FixedAngle;
  boolean       Changed;
  String        Tagname;
  boolean       Poll;
  unsigned long PollFreq;
  unsigned long PollTime;
  boolean       Auto;
  boolean       DBlogging;
} ;
MyServoType ServoPin[NoServos + 1];

typedef struct MyUltraType
{
  int           Trigger;
  int           Echo;
  int           MaxDistance;
  int           Distance;
  String        Tagname;
  boolean       Poll;
  unsigned long PollFreq;
  unsigned long PollTime;
  boolean       Mask;
  boolean       DBLogging;
} ;
MyUltraType Ultrasonic[NoUltrasonic + 1];

typedef struct MarkerType
{
  boolean       Status;
  boolean       PreStatus;
  boolean       FixedStatus;
  int           Value;
  int           FixedValue;
  int           PrevValue;
  String        TextString;
  String        PreTextString;
  unsigned long TimeChange;
  boolean       Changed;
  boolean       ChangedAnalogue;
  boolean       ChangedText;
  String        Tagname;
  boolean       Report;
  boolean       Poll;
  boolean       Auto;
  unsigned long PollFreq;
  unsigned long PollTime;
  unsigned long PollTimeAnalogue;
  unsigned long PollTimeText;
  boolean       DBLogging;
} ;
MarkerType Marker[NoMarkers + 1];

struct GPSType
{
  //NEO-6M Type with use of $PUBX commands
  String        UTC;           //2
  String        Latitude;      //3
  String        NSIndicator;   //4
  String        Longitude;     //5
  String        EWIndicator;   //6
  String        NavStat;       //8
  String        Hacc;          //9
  String        SOG;           //11
  String        COG;           //12
  String        GU;            //18 
};
GPSType GPSdata;


//*******************************************************************************
//
//  Name: HMISendString 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Send a message to HMI (via hardware communication channel 1)
//
//
//*******************************************************************************
void HMISendString ( String MessageToHMI )
{

  char LF = 10;
  char CR = 13;
  char Buffer[1000];
  int LenMessage;

  LenMessage = MessageToHMI.length();
  MessageToHMI.toCharArray(Buffer, 500);

  //Serial.println ( "@HMISendString (length:" + String(LenMessage) + "): >" + MessageToHMI + "<" );
    
  for (int index = 0; index < LenMessage; index++) Serial1.write(Buffer[index]);
  Serial1.write(CR);
  Serial1.write(LF);

  //Serial1.println(MessageToHMI);

}

//*******************************************************************************
//
//  Name: HMISendIntercard 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Send a message to another 2560Mega card (serial 3)
//
//
//*******************************************************************************
void IntercardSendString(String MessageToMEGA)
{

	char LF = 10;
	char CR = 13;
	char Buffer[500];
	int LenMessage;

	LenMessage = MessageToMEGA.length();
	MessageToMEGA.toCharArray(Buffer, 500);

	//Serial.println ( "@IntercardSendString (length:" + String(LenMessage) + "): >" + MessageToMEGA + "<" );

	for (int index = 0; index < LenMessage; index++) Serial3.write(Buffer[index]);
	Serial3.write(CR);
	Serial3.write(LF);

	Serial3.println(MessageToMEGA);

}


//*******************************************************************************
//
//  Name: digitalClockDisplay
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Print the current time on the serial monitor.
//
//
//*******************************************************************************
void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

//*******************************************************************************
//  Name: LeadingZero
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Return a string of 4 characters, with leading zero's. 
//  Input an unsigned integer
//
//
//*******************************************************************************
String LeadingZero ( unsigned int Value )
{

  String Leading;
  String StrValue;

  StrValue = String ( Value );
  if (Value < 10 ) Leading = "0";
  if (Value < 100 ) Leading = Leading + "0";
  if (Value < 1000 ) Leading = Leading + "0";
  StrValue = Leading + StrValue;
  return StrValue;

}

//*******************************************************************************
//
//  Name: PrintDigits
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Print semicolon and a leading zero on serial monitor, input is an integer
//
//
//*******************************************************************************
void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10) Serial.print('0');
  Serial.print(digits);
}

//*******************************************************************************
//
//  Name: processSyncMessage 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  When there is a unix time stamp from the serial monitor available,
//  set the time on the board. When a RTC available, set also the time on the
//  real time clock.
//
//
//*******************************************************************************
void processSyncMessage() {
  // if time sync available from serial port, update time and return true
	while (Serial.available() >= TIME_MSG_LEN) { // time message consists of header & 10 ASCII digits
		char c = Serial.read();
		//Serial.print(c);
		if (c == TIME_HEADER) {
			time_t pctime = 0;
			for (int i = 0; i < TIME_MSG_LEN - 1; i++) {
				c = Serial.read();
				if (c >= '0' && c <= '9') {
					pctime = (10 * pctime) + (c - '0'); // convert digits to a number
				}
			}
			setTime(pctime);
			if (RTCAvailable)
			{
				RealTimeClock.adjust(pctime);
				//RTC.set(pctime);
				//Serial.println ( F("%INF-RTC-CTC, RTC is also updated"));
			}
		}
	}
}

//*******************************************************************************
//
//  Name: InitFSMStates 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Set the pointer structure for the FSM states. See also the documentation
//  for information about FMS states. Set the default states for the FSM's on
//  HIB (hibernate).
//
//
//*******************************************************************************
void InitFSMStates ()
{

  int StateNo = 0;
  FSMStateType* FSMStateLaatste;

  Serial.println ( F("Setup FSM States"));
  NoFSMStates = sizeof(PossibleFSMStates) / 6;

  while (StateNo < NoFSMStates)
  {
    // Build the state list..
    if ( FSMStateKetting == 0 )
    {
      // Chain doesn't exists yet
      FSMStateKetting = new FSMStateType;
      FSMStateKetting->FSMStateName = PossibleFSMStates [StateNo];
      FSMStateKetting->FSMStateNo = StateNo + 1;
      FSMStateKetting->ActualState = "HIB";
      FSMStateKetting->ThreadNo = 0;
      FSMStateKetting->StartTime = 0;
      FSMStateKetting->TotalTime = 0;
      FSMStateKetting->Next = 0;
      FSMStateLaatste = FSMStateKetting;
    }
    else
    {
      FSMStateSchakel = new FSMStateType;
      FSMStateSchakel->FSMStateName = PossibleFSMStates [StateNo];
      FSMStateSchakel->FSMStateNo = StateNo + 1;
      FSMStateSchakel->ActualState = "HIB";
      FSMStateSchakel->ThreadNo = 0;
      FSMStateSchakel->StartTime = 0;
      FSMStateSchakel->TotalTime = 0;
      FSMStateSchakel->Next = 0;
      FSMStateLaatste->Next = FSMStateSchakel;
      FSMStateLaatste = FSMStateSchakel;
    }
    ++StateNo;
  }

  //Going trough the chain....
  FSMStateSchakel = FSMStateKetting;
  do
  {
    Serial.println ( "State " + FSMStateSchakel->FSMStateName + " (PID=" + String (FSMStateSchakel->FSMStateNo ) + "), current state = " + FSMStateSchakel->ActualState );
    FSMStateSchakel = FSMStateSchakel->Next;
  } while ( FSMStateSchakel != 0);

  Serial.println ( F("Setup FSM States ready"));
  Serial.println ( F("-------------------------------------"));
  Serial.println ( F(""));

} // End of InitFSMStates

//*******************************************************************************
//
//  Name: FSMStateOverview
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Present an overview of the FSM states on the serial monitor.
//
//
//*******************************************************************************
void FSMStateOverview()
{

  String RunTime;

  Serial.println();
  Serial.print(F("FSM state overview at "));
  digitalClockDisplay();
  Serial.println(F("----------------------------------------- "));
  FSMStateSchakel = FSMStateKetting;
  do
  {
    if (FSMStateSchakel->StartTime == 0 )
    {
      RunTime = String ( FSMStateSchakel->TotalTime / 1000 );
    }
    else
    {
      RunTime = String ( ( FSMStateSchakel->TotalTime + millis() - FSMStateSchakel->StartTime ) / 1000 ) + "+";
    };
    Serial.println ( "State " + FSMStateSchakel->FSMStateName + " (PID=" + String (FSMStateSchakel->FSMStateNo ) + "), current state = " + FSMStateSchakel->ActualState + " in thread no. " + String (FSMStateSchakel->ThreadNo) + " Runtime: " + RunTime );
    FSMStateSchakel = FSMStateSchakel->Next;
  } while ( FSMStateSchakel != 0);
  Serial.println(" ");

} // End of FSMStateOverview;

//*******************************************************************************
//
//  Name: TransitionToState 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  - Find the current state in the active thread and put it on hibernate (HIB).
//  - Examine all existing threads and find the highest thread.
//  - Check the existens of the new statename.
//  - Put the new statename on state pending (PEN). 
//  - If the name of the state is END do nothing.
//
//  Explanation of handling FSM states (most be documented somewhere...)
//
//  - Initial all states have the status HIB (hibernate)
//  - By calling TransitionToState, the state's status is changed to PEN (pending)
//    so the state is a candidate for executing the code in the main loop.
//  - At the end of the PLC cycle, all PEN (pending) states are put in RUN (running) state.
//    Also all states still on COM (computing) are put on RUN (running) again.
//  - In the new PLC cylce, all RUN (running) states are executed once and the status is
//    put on COM (computing). When TransitionToState is called in the active state,
//    the active state is put in HIB (hibernate).  
//  - It is possible to have more FSM's executed in parallel. Just call 
//    TransitionToState more than once from the active state. They are then devided
//    over more parallel Threads.
//  - It is possible to Kill a thread when calling the END state in TransistionToState.
//    Be aware that killing the only living thread will kill the FSM.  
//
//*******************************************************************************
void TransitionToState ( String StateName )
{
  FSMStateType* Chain;
  boolean FSMStateNameFound = false;
  int CurrentThread;
  int HighThreadNo;
  unsigned long EndTime;

  // find the current state and put it on HIB
  Chain = FSMStateKetting;
  do
  {
    if (Chain->FSMStateNo == MachineState)
    {
      // Current state is found... put it on HIB
      Chain->ActualState = "HIB";
	  Chain->Exit = true;
      Chain->ThreadNo = 0;
      if (Chain->StartTime != 0)
      {
        Chain->TotalTime = Chain->TotalTime + millis() - Chain->StartTime;
        Chain->StartTime = 0;
      }
    if (UseHMISerial) HMISendString("@FSM," + String(Chain->FSMStateName) + ",HIB");
    }
    Chain = Chain->Next;
  } while ( Chain != 0);

  // What can happen now?
  // Look for States on RUN and find highest ThreadNo
  Chain = FSMStateKetting;
  HighThreadNo = 0;
  do
  {
    //State with status on RUN?
    if ((Chain->ActualState == "RUN" ) || (Chain->ActualState == "PEN" ))
    {
      Threads = Threads + 1;
      if (Chain->ThreadNo > HighThreadNo ) HighThreadNo = Chain->ThreadNo;
    };
    Chain = Chain->Next;
  } while ( Chain != 0);
  HighThreadNo = HighThreadNo + 1;

  // To be sure, check if StateName is known...
  Chain = FSMStateKetting;
  do
  {
    //State with status on PEN and ThreadNo is set, except for END (=do Nothing)
    if (Chain->FSMStateName == StateName ) FSMStateNameFound = true;
    Chain = Chain->Next;
  } while ( Chain != 0);
  if (!FSMStateNameFound)
  {
	  Blinking = 1000;
	  Serial.println("State: " + StateName + " not found");
  }

  // ThreadNo is determined and now we can set the new state on PEN
  Chain = FSMStateKetting;
  do
  {
    //State with status on PEN and ThreadNo is set, except for END (=do Nothing)
    if ((Chain->FSMStateName == StateName ) && ( StateName != "END") && (Chain->ActualState == "HIB"))
    {
      Chain->ThreadNo = HighThreadNo;
      Chain->ActualState = "PEN";
	  //extra check; when exit event is set, this must be done in this transition; so it's a call to this transistion to remain running. 
	  //do nothing with the enter or exit event!
	  if (!Chain->Exit) Chain->Enter = true;
      Chain->StartTime = millis();
      if (UseHMISerial) HMISendString ( "@FSM," + String(StateName) + ",RUN" );
    };
    Chain = Chain->Next;
  } while ( Chain != 0);

} // End of TransitionToState

//*******************************************************************************
//
//  Name: CurrentState
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Return the PID number of the running (RUN) state and change the actual state 
//  to computing (COM)
//
//
//*******************************************************************************
int CurrentState()
{
  FSMStateType* Chain;
  int Action;
  boolean Stop;

  Action = 0;
  Chain = FSMStateKetting;
  do
  {
    if ((Chain->ActualState == "RUN"))
    {
      Chain->ActualState = "COM";
      Action = Chain->FSMStateNo;
      break;
    };
    Chain = Chain->Next;
  } while (( Chain != 0)) ;

  return Action;

} // End of CurrentState

//*******************************************************************************
//
//  Name: ComToRunState 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Change a computing (COM) or pending (PEN) state to running state(RUN).
//
//
//*******************************************************************************
void ComToRunState()
{
  FSMStateType* Chain;
  int Action;

  Action = 0;
  Chain = FSMStateKetting;
  do
  {
    if ((Chain->ActualState == "COM") || (Chain->ActualState == "PEN"))
    {
      Chain->ActualState = "RUN";
    };
    Chain = Chain->Next;
  } while ( Chain != 0) ;

} // End of ComToRunState


//*******************************************************************************
//
//  Name: FiniteState
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Return the PID number of the CurrentState
//
//
//*******************************************************************************
int FiniteState ( String CurrentState )
{
  FSMStateType* Chain;
  int Action;

  Action = 0;
  Chain = FSMStateKetting;
  do
  {
    if (Chain->FSMStateName == CurrentState)
    {
      Action = Chain->FSMStateNo;
    };
    Chain = Chain->Next;
  } while ( Chain != 0) ;

  return Action;

}

//*******************************************************************************
//
//  Name: EnterState
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Return true when entering the state the first time and reset the enter event
//
//
//*******************************************************************************
boolean EnterState()
{
	FSMStateType* Chain;
	boolean Action;

	Action = false;
	Chain = FSMStateKetting;
	do
	{
		if (Chain->FSMStateNo == MachineState)
		{
			Action = Chain->Enter;
			Chain->Enter = false;
		}
		Chain = Chain->Next;
	} while (Chain != 0);

	return Action;

} //EnterState


  //*******************************************************************************
  //
  //  Name: ExitState
  //
  //  Modification date: 
  //  Changed by: 
  //
  //  Function:
  //  Return true when leaving the state and reset the exit event
  //
  //
  //*******************************************************************************
boolean ExitState()
{
	FSMStateType* Chain;
	boolean Action;

	Action = false;
	Chain = FSMStateKetting;
	do
	{
		if (Chain->FSMStateNo == MachineState)
		{
			Action = Chain->Exit;
			Chain->Exit = false;
		}
		Chain = Chain->Next;
	} while (Chain != 0);

	return Action;

} //ExitState


//*******************************************************************************
//
//  Name: InitInputs 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Initialize all digital input tags. Default INPUT_PULLUP. So not connected
//  the pins are high.
//
//
//*******************************************************************************
void InitInputs ()
{
  Serial.println ( F("Setup inputs"));
  Serial.println ( String("No. of inputs ") + String(NoInputPins));
  for (int InputIndex = 1; InputIndex <= (NoInputPins); InputIndex++)
  {
	InPin[InputIndex].Pin = (InputPins[InputIndex-1].substring(InputPins[InputIndex-1].length()-2, InputPins[InputIndex-1].length())).toInt();
    pinMode(InPin[InputIndex].Pin, INPUT_PULLUP );
    InPin[InputIndex].Status = digitalRead (InPin[InputIndex].Pin);
    InPin[InputIndex].Changed = 0;
    InPin[InputIndex].Poll = false;
    InPin[InputIndex].Mask = false;
    InPin[InputIndex].TimeChange = millis();
    InPin[InputIndex].Tagname = InputTags[InputIndex - 1];
	InPin[InputIndex].DBLogging = false;
	Serial.println ( "Init " + String (InPin[InputIndex].Tagname) + " #" + String ( InPin[InputIndex].Pin ) );
  }
  Serial.println ( F("Setup inputs ready"));
  Serial.println ( F("-------------------------------------"));
  Serial.println ();

} // End of InitInputs

//*******************************************************************************
//
//  Name: InitMarkers
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Initialize all markers tags.
//
//
//*******************************************************************************
void InitMarkers ()
{
  Serial.println ( F("Setup markers"));
  Serial.println ( String("No. of Markers ") + String(NoMarkers));
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
    Marker[Index].Status = 0;
	Marker[Index].PreStatus = 0;
	Marker[Index].FixedStatus = 0;
	Marker[Index].TextString = "";
    Marker[Index].Changed = 0;
	Marker[Index].ChangedAnalogue = 0;
	Marker[Index].ChangedText = 0;
	Marker[Index].Auto = true;
    Marker[Index].Poll = false;
    Marker[Index].Report = false;
    Marker[Index].TimeChange = millis();
    Marker[Index].Tagname = MarkerTags[Index - 1];
	Marker[Index].DBLogging = false;
    Serial.println ( "Init " + String (Marker[Index].Tagname) );
  }
  Serial.println ( F("Setup markers ready"));
  Serial.println ( F("-------------------------------------"));
  Serial.println ( F(""));

} // End of InitMarkers


//*******************************************************************************
//
//  Name: InitUltrasonics
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Initialize all ultrasonic devices. Each ultrasonic devices uses 2 digital
//  pins. The first is the trigger pin, the next the echo pin.
//
//
//*******************************************************************************
void InitUltrasonics ()
{
  Serial.println ( F("Setup ultrasonics"));
  Serial.println ( String("No. of ultrasonics ") + String(NoUltrasonic));
  for (int InputIndex = 1; InputIndex <= (NoUltrasonic); InputIndex++)
  {
    Ultrasonic[InputIndex].Tagname = UltraTags[InputIndex - 1];
    Ultrasonic[InputIndex].MaxDistance = 500;
    Ultrasonic[InputIndex].Distance = 0;
    Ultrasonic[InputIndex].Poll = false;
    Ultrasonic[InputIndex].Mask = false;
	Ultrasonic[InputIndex].Trigger = (UltraPins[2*(InputIndex-1)].substring(UltraPins[2*(InputIndex-1)].length() - 2, UltraPins[2*(InputIndex-1)].length())).toInt();
	Ultrasonic[InputIndex].Echo = (UltraPins[2*InputIndex - 1].substring(UltraPins[2 *InputIndex - 1].length() - 2, UltraPins[2 * InputIndex - 1].length())).toInt();
	Ultrasonic[InputIndex].DBLogging = false;
	pinMode(Ultrasonic[InputIndex].Trigger, OUTPUT);
	pinMode(Ultrasonic[InputIndex].Echo, INPUT);
    Serial.println ( "Init " + String (Ultrasonic[InputIndex].Tagname) + " trigger #" + String ( Ultrasonic[InputIndex].Trigger ) + " echo #" + String ( Ultrasonic[InputIndex].Echo));
  }
  Serial.println ( F("Setup ultrasonic sensors ready"));
  Serial.println ( F("-------------------------------------"));
  Serial.println ();


} // End of InitUltrasonocs


//*******************************************************************************
//
//  Name: InitOutputs 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Initialize all output tags.
//
//
//*******************************************************************************
void InitOutputs ()
{
  Serial.println ( F("Setup outputs"));
  Serial.println ( String("No. of outputs ") + String(NoOutputPins));
  for (int OutputIndex = 1; OutputIndex <= (NoOutputPins); OutputIndex++)
  {
	OutPin[OutputIndex].Pin = (OutputPins[OutputIndex - 1].substring(OutputPins[OutputIndex - 1].length() - 2, OutputPins[OutputIndex - 1].length())).toInt();
    pinMode(OutPin[OutputIndex].Pin, OUTPUT);
    OutPin[OutputIndex].Status = 0;
    OutPin[OutputIndex].PreStatus = 0;
    OutPin[OutputIndex].Changed = 0;
    OutPin[OutputIndex].TimeChange = millis();
    OutPin[OutputIndex].FixedStatus = false;
    OutPin[OutputIndex].Auto = true;
    OutPin[OutputIndex].Poll = false;
    OutPin[OutputIndex].Tagname = OutputTags[OutputIndex - 1];
	if (OutPin[OutputIndex].Tagname.substring(0, 1) == "_")	digitalWrite(OutPin[OutputIndex].Pin, HIGH);
	OutPin[OutputIndex].DBLogging = false;
    Serial.println ( "Init " + String (OutPin[OutputIndex].Tagname) + " #" + String ( OutPin[OutputIndex].Pin ) );
  }
  Serial.println ( F("Setup outputs ready"));
  Serial.println ( F("-------------------------------------"));
  Serial.println ();

} // End of InitOuputs

//*******************************************************************************
//
//  Name: InitTemp 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Initialize all temperature devices. They are connected to one PWN pin
//  Remark: The sensors take time to compute the temperature and hold-up
//          the board!
//
//
//*******************************************************************************
void InitTemp ()
{
  Serial.println ( F("Setup Temp inputs"));
  Serial.println ( String("No. of temperature inputs ") + String(NoTemp));
  
  for (int InputIndex = 1; InputIndex <= (NoTemp); InputIndex++)
  {
    TempSensor[InputIndex].DeviceNr = InputIndex ;
	TempSensor[InputIndex].Value = 0;
    TempSensor[InputIndex].PrevValue = TempSensor[InputIndex].Value;
    TempSensor[InputIndex].TimeChange = millis();
    TempSensor[InputIndex].Poll = false;
    TempSensor[InputIndex].Mask = false;
    TempSensor[InputIndex].Tagname = TempTags[InputIndex - 1];
	TempSensor[InputIndex].DBLogging = false;
    Serial.println ( "Init " + String (TempSensor[InputIndex].Tagname) + " Device no." + String ( TempSensor[InputIndex].DeviceNr ) + " (One-wire channel pin# " + String(OneWireChannel) + ")" );
  }
  Serial.println ( F("Setup Temp inputs ready"));
  Serial.println ( F("-------------------------------------"));
  Serial.println ( F(""));

} // End of InitTemp

//*******************************************************************************
//
//  Name: InitAnaIn
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Initialize analogue input tags. The analogue reference is set on Default.
//
//
//*******************************************************************************
void InitAnaIn ()
{
  Serial.println ( F("Setup analog inputs"));
  Serial.println ( String("No. of analog inputs ") + String(NoAnaInPins));
  analogReference ( DEFAULT );
  for (int InputIndex = 1; InputIndex <= (NoAnaInPins); InputIndex++)
  {
    AnaInPin[InputIndex].Pin = (AnaInPins[InputIndex - 1].substring(AnaInPins[InputIndex - 1].length() - 2, AnaInPins[InputIndex - 1].length())).toInt();
    pinMode(AnaInPin[InputIndex].Pin, INPUT);
    AnaInPin[InputIndex].Value = analogRead (AnaInPin[InputIndex].Pin);
    AnaInPin[InputIndex].PrevValue = analogRead (AnaInPin[InputIndex].Pin);
    AnaInPin[InputIndex].TimeChange = millis();
    AnaInPin[InputIndex].Poll = false;
    AnaInPin[InputIndex].Mask = false;
    AnaInPin[InputIndex].Tagname = AnaInTags[InputIndex - 1];
	AnaInPin[InputIndex].DBlogging = false;
    Serial.println ( "Init " + String (AnaInPin[InputIndex].Tagname) + " #" + String ( AnaInPin[InputIndex].Pin ) );
  }
  Serial.println ( F("Setup Analog inputs ready"));
  Serial.println ( F("-------------------------------------"));
  Serial.println ( F(""));

} // End of InitAnaIn

//*******************************************************************************
//
//  Name: InitAnaOut 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Initialize the analogue output tags. Remark: Pulse Width Modulation (PWM)
//
//
//*******************************************************************************
void InitAnaOut ()
{
  Serial.println ( F("Setup analog outputs"));
  Serial.println ( String("No. of analog outputs ") + String(NoAnaOutPins));
  for (int OutputIndex = 1; OutputIndex <= (NoAnaOutPins); OutputIndex++)
  {
    AnaOutPin[OutputIndex].Pin = (AnaOutPins[OutputIndex - 1].substring(AnaOutPins[OutputIndex - 1].length() - 2, AnaOutPins[OutputIndex - 1].length())).toInt();
	pinMode(AnaOutPin[OutputIndex].Pin, OUTPUT);
    AnaOutPin[OutputIndex].PrevValue = 0;
    AnaOutPin[OutputIndex].Value = 0;
    AnaOutPin[OutputIndex].TimeChange = millis();
    AnaOutPin[OutputIndex].Auto = true;
    AnaOutPin[OutputIndex].Poll = false;
    AnaOutPin[OutputIndex].FixedValue = true;
    AnaOutPin[OutputIndex].Tagname = AnaOutTags[OutputIndex - 1];
	AnaOutPin[OutputIndex].DBLogging = false;
    Serial.println ( "Init " + String (AnaOutPin[OutputIndex].Tagname) + " #" + String ( AnaOutPin[OutputIndex].Pin ) );
  }
  Serial.println ( F("Setup analog outputs ready"));
  Serial.println ( F("-------------------------------------"));
  Serial.println ();

} // End of InitAnaOut

//*******************************************************************************
//
//  Name: InitServos
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Initialize the servo tags (PWM's). Default angle is 90 degrees.
//
//
//*******************************************************************************
void InitServos ()
{
  Serial.println ( F("Setup servos"));
  Serial.println ( String("No. of servos ") + String(NoServos));
  for (int OutputIndex = 1; OutputIndex <= (NoServos); OutputIndex++)
  {
    ServoPin[OutputIndex].Pin = (ServoPins[OutputIndex - 1].substring(ServoPins[OutputIndex - 1].length() - 2, ServoPins[OutputIndex - 1].length())).toInt();
	ServoPin[OutputIndex].MyServo.attach ( ServoPin[OutputIndex].Pin );
    ServoPin[OutputIndex].TimeChange = millis();
    ServoPin[OutputIndex].Angle = 90;
    ServoPin[OutputIndex].FixedAngle = 90;
    ServoPin[OutputIndex].Auto = true;
    ServoPin[OutputIndex].Poll = false;
    ServoPin[OutputIndex].Tagname = ServoTags[OutputIndex - 1];
	ServoPin[OutputIndex].DBlogging = false;
    Serial.println ( "Init " + String (ServoPin[OutputIndex].Tagname) + " #" + String ( ServoPin[OutputIndex].Pin ) );
  }
  Serial.println ( F("Setup servo's ready"));
  Serial.println ( F("-------------------------------------"));
  Serial.println ( "" );

} // End of Servos


//*******************************************************************************
//
//  Name: InputOverview
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Present the input tags on the serial monitor.
//
//
//*******************************************************************************
void InputOverview()
{


  Serial.println("");
  Serial.print(F("Input overview at "));
  digitalClockDisplay();
  Serial.println("---------------------------------- ");
  //Serial.println( String("Total ") + String (NoInputPins) + String(" (#1 pin on ") + String(FirstInput) + String(")"));
  //Serial.println("");

  //Serial.println (F("    0  1  2  3  4  5  6  7  8  9  "));
  //Serial.println (F(" +------------------------------"));
  //Serial.print (String(FirstInput / 10) + String("|"));
  //for (int InputIndex = 1; InputIndex <= (FirstInput % 10); InputIndex++)
  //{
  //  Serial.print(F("   "));
  //};
  //for (int InputIndex = 1; InputIndex <= (NoInputPins); InputIndex++)
  //{
  //  Serial.print( String("  ") + String(InPin[InputIndex].Status));
  //  if ( (InputIndex + (FirstInput % 10)) % 10 == 0 )
  //  {
  //    Serial.println ();
  //    Serial.print( String ((InputIndex + FirstInput) / 10 ));
  //    Serial.print( F("|"));
  //  }
  //}
  Serial.println(F(" "));
  Serial.println(F(" "));
  for (int InputIndex = 1; InputIndex <= (NoInputPins); InputIndex++)
  {
	  Serial.println("Pin: #" + String(InPin[InputIndex].Pin) + " " + String(InputTags[InputIndex - 1]) +
		  String(" Status:") + String(InPin[InputIndex].Status) +
		  String(" Mask:") + String(InPin[InputIndex].Mask) +
		  String(" Poll:") + String(InPin[InputIndex].Poll) +
		  String(" Report:") + String(InPin[InputIndex].Report) +
		  String(" DB:") + String(InPin[InputIndex].DBLogging));
  }
  Serial.println(" ");
}
//** End of InputOverview 

//*******************************************************************************
//
//  Name: MarkerOverview
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Present an overview of the markers on the serial monitor.
//
//
//*******************************************************************************
void MarkerOverview()
{
  Serial.println();
  Serial.print(F("Marker overview at "));
  digitalClockDisplay();
  Serial.println(F("---------------------------------- "));
  Serial.println( String("Total ") + String (NoMarkers));
  Serial.println("");
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  Serial.println("Marker: " + String(Marker[Index].Tagname) +
		  String(" Status:") + String(Marker[Index].Status) +
		  String(" Value:") + String(Marker[Index].Value) +
		  String(" Text:") + String(Marker[Index].TextString) +
		  String(" Poll:") + String(Marker[Index].Poll) +
		  String(" Report:") + String(Marker[Index].Report) +
		  String(" Auto:") + String(Marker[Index].Auto) +
		  String(" DB:") + String(Marker[Index].DBLogging));
  }
  Serial.println(" ");
}
//** End of MarkerOverview 


//*******************************************************************************
//
//  Name: OutputOverview
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Present an overview of the output tags on the serial monitor.
//
//
//*******************************************************************************
void OutputOverview()
{
  Serial.println("");
  Serial.print(F("Output overview at "));
  digitalClockDisplay();

  //Serial.println(F("----------------------------------- "));
  //Serial.println( String("Total ") + String (NoOutputPins) + String(" (#1 pin on ") + String(FirstOutput) + String(")"));
  //Serial.println("");

  //Serial.println (F("    0  1  2  3  4  5  6  7  8  9  "));
  //Serial.println (F(" +------------------------------"));
  //Serial.print (String(FirstOutput / 10) + String("|"));
  //for (int OutputIndex = 1; OutputIndex <= (FirstOutput % 10); OutputIndex++)
  //{
  //  Serial.print("   ");
  //}
  //for (int OutputIndex = 1; OutputIndex <= (NoOutputPins); OutputIndex++)
  //{
  //  Serial.print( String("  ") + String(OutPin[OutputIndex].Status));
  //  if ( (OutputIndex + (FirstOutput % 10)) % 10 == 0 )
  //  {
  //    Serial.println ();
  //    Serial.print( String ((OutputIndex + FirstOutput) / 10 ));
  //    Serial.print(F("|"));
  //  }
  //}
  //Serial.println(F(" "));
  Serial.println(F(" "));
  for (int OutputIndex = 1; OutputIndex <= (NoOutputPins); OutputIndex++)
  {
    Serial.println ("Pin: #" + String(OutPin[OutputIndex].Pin) + " " + String ( OutputTags [OutputIndex - 1 ] ) +
                    " Status:" + String( OutPin[OutputIndex].Status) +
                    " Auto:" + String( OutPin[OutputIndex].Auto) +
                    " Poll:" + String( OutPin[OutputIndex].Poll) +
                    " Report:" + String( OutPin[OutputIndex].Report) +
                    " DB:" + String(OutPin[OutputIndex].DBLogging));
  }
  Serial.println(F(" "));

}
//** End of OutputOverview 

//*******************************************************************************
//
//  Name: AnaInOverview 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Present an overview of all analogue input tags
//
//
//*******************************************************************************
void AnaInOverview()
{
  float Volts;

  Serial.println();
  Serial.print(F("Analog input overview at "));
  digitalClockDisplay();

  //Serial.println(F("----------------------------------------- "));
  //Serial.println( String("Total ") + String (NoAnaInPins) + String(" (#1 pin on ") + String(FirstAnaIn) + String(")"));
  //Serial.println();

  for (int AnaInIndex = 1; AnaInIndex <= NoAnaInPins; AnaInIndex++)
  {
    Volts = AnaInPin[AnaInIndex].Value * 5.00;
    Volts = Volts / 1023;
    Serial.println ("Pin: #" + String(AnaInPin[AnaInIndex].Pin) + " " + String ( AnaInTags [AnaInIndex - 1 ] ) +
                    " Value:(" + String ( AnaInPin[AnaInIndex].Value) + String ( ") " ) + String(Volts) + String( " Volt" ) +
                    " Mask:" + String ( AnaInPin[AnaInIndex].Mask) +
                    " Poll:" + String ( AnaInPin[AnaInIndex].Poll) +
		            " DB:" + String(AnaInPin[AnaInIndex].DBlogging));
  }
  Serial.println(F(" "));

}
//** End of AnaInOverview 

//*******************************************************************************
//
//  Name: AnaOutOverview 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Present an overview of all analogue output tags (PWM).
//
//
//*******************************************************************************
void AnaOutOverview()
{
  float Volts;

  Serial.println(F(""));
  Serial.print(F("Analog output overview at "));
  digitalClockDisplay();

  //Serial.println(F("----------------------------------------- "));
  //Serial.println( String("Total ") + String (NoAnaOutPins) + String(" (#1 pin on ") + String(FirstAnaOut) + String(")"));
  //Serial.println("");

  for (int AnaIndex = 1; AnaIndex <= NoAnaOutPins; AnaIndex++)
  {
    Volts = AnaOutPin[AnaIndex].Value;
    Volts = Volts * 0.01961;
    Serial.println ("Pin: #" + String(AnaOutPin[AnaIndex].Pin) + " " + String ( AnaOutTags [AnaIndex - 1] ) + " Value:(" + String ( AnaOutPin[AnaIndex].Value) + ") " + String(Volts) + " Volt" +
                    " Auto: " + String( AnaOutPin[AnaIndex].Auto) +
                    " Poll:" + String( AnaOutPin[AnaIndex].Poll) +
		            " DB:" + String(AnaOutPin[AnaIndex].DBLogging));
  }
  Serial.println(F(" "));

}
//** End of AnaOutOverview 

//*******************************************************************************
//
//  Name: TemperatureOverview 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Present an overview of all connected temp sensors
//
//
//*******************************************************************************
void TemperatureOverview()
{

  Serial.println("");
  Serial.print(F("Temperature overview at "));
  digitalClockDisplay();
  Serial.println(F("----------------------------------------- "));
  Serial.println( String("Total ") + String (NoTemp) + String(" (Onewire channel on ") + String(OneWireChannel) + String(")"));
  Serial.println("");
  for (int AnaIndex = 1; AnaIndex <= NoTemp; AnaIndex++)
  {
    Serial.println ("Temp: #" + String(TempSensor[AnaIndex].DeviceNr) + " " + String ( TempSensor [AnaIndex].Tagname ) + " Value:" + String ( TempSensor[AnaIndex].Value) + " Mask: " + String( TempSensor[AnaIndex].Mask) +
                    " Poll:" + String( TempSensor[AnaIndex].Poll) +
		            " DB:" + String(TempSensor[AnaIndex].DBLogging));
  }
  Serial.println(F(" "));

}
//** End of TemperatureOverview 

//*******************************************************************************
//
//  Name: ServoOverview
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Present an overview of all servo tags.
//
//
//*******************************************************************************
void ServoOverview()
{
  Serial.println();
  Serial.print(F("Servo's overview at "));
  digitalClockDisplay();

  //Serial.println(F("----------------------------------------- "));
  //Serial.println( String("Total ") + String (NoServos) + String(" (#1 pin on ") + String(FirstServo) + String(")"));
  //Serial.println("");

  for (int AnaIndex = 1; AnaIndex <= NoServos; AnaIndex++)
  {
    Serial.println ("Pin: #" + String(ServoPin[AnaIndex].Pin) + " " + String ( ServoTags [AnaIndex - 1] ) + " Angle:" + String ( ServoPin[AnaIndex].Angle) +
                    " Auto:" + String( ServoPin[AnaIndex].Auto) +
                    " Poll:" + String( ServoPin[AnaIndex].Poll) +
		            " DB:" + String(ServoPin[AnaIndex].DBlogging));
  }
  Serial.println(F(" "));

}
//** End of ServoOverview 

//*******************************************************************************
//
//  Name: UltraSonicOverview
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Present an overview of all Ultrasonic devices.
//
//
//*******************************************************************************
void UltraSonicOverview()
{
  Serial.println("");
  Serial.print(F("Ultrasonic overview at "));
  digitalClockDisplay();

  //Serial.println(F("----------------------------------------- "));
  //Serial.println( String("Total ") + String (NoUltrasonic) + String(" (#1 pin on ") + String(FirstUltra) + String(")"));
  //Serial.println("");

  for (int AnaIndex = 1; AnaIndex <= NoUltrasonic; AnaIndex++)
  {
    Serial.println ("UltraSonic: " + String ( UltraTags [AnaIndex - 1 ] ) + String("  Distance (cm): ") + String( Ultrasonic[AnaIndex].Distance) +
                    " Poll:" + String( Ultrasonic[AnaIndex].Poll) +
                    " Mask:" + String( Ultrasonic[AnaIndex].Mask) +
		            " DB:" + String(Ultrasonic[AnaIndex].DBLogging));
  }
  Serial.println(F(" "));

}
//** End of UltraSonicOverview 

//*******************************************************************************
//
//  Name: Activate
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Set a request for setting the selected digital output tagname high
//
//
//*******************************************************************************
boolean Activate (String Tag )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;
  for (int Index = 1; Index <= (NoOutputPins); Index++)
  {
    if (OutPin[Index].Tagname == Tag)
    {
      OutPin[Index].Changed = true;
      Action = true;
      TagFound = true;
    }
  }

  if (!TagFound)
    {
      //* Maybe a Marker?
      for (int Index = 1; Index <= (NoMarkers); Index++)
        {
          if (Marker[Index].Tagname == Tag)
            {
			//Serial.println("Marker found in Activate: " + Tag);
            Marker[Index].Changed = true;
            Action = true;
            TagFound = true;
            }
        }
    }

  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of Activate 

//*******************************************************************************
//
//  Name: Deactivate 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Deactivate the selected digital marker
//
//
//*******************************************************************************
boolean Deactivate (String Tag )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
    if (Marker[Index].Tagname == Tag)
    {
      Marker[Index].Status = false;
      Action = true;
      TagFound = true;
    }
  }

  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of Deactivate (marker) 

//*******************************************************************************
//
//  Name: MarkerValue
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Get Marker text string
//
//
//*******************************************************************************
String MarkerValue(String Tag)
{
	boolean Action = false;
	boolean TagFound = false;
	String TextString = "";

	for (int Index = 1; Index <= (NoMarkers); Index++)
	{
		if (Marker[Index].Tagname == Tag)
		{
			TextString = Marker[Index].TextString;
			Action = true;
			TagFound = true;
		}
	}

	if (!TagFound) Blinking = 50;

	return TextString;
}
//** End of MarkerValue 


//*******************************************************************************
//
//  Name: UpdateMarker 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Update marker text string
//
//
//*******************************************************************************
boolean UpdateMarker(String Tag, String TextString)
{
	boolean Action = false;
	boolean TagFound = false;

	for (int Index = 1; Index <= (NoMarkers); Index++)
	{
		if (Marker[Index].Tagname == Tag)
		{
			if (Marker[Index].Auto)
			{
				if (Marker[Index].TextString != TextString)
				{
					HMISendString("@RDC," + TextString + "," + Marker[Index].Tagname + "," + Marker[Index].Auto + "," + Marker[Index].DBLogging);
				}
				Marker[Index].TextString = TextString;
			}
			Action = true;
			TagFound = true;
		}
	}

	if (!TagFound) Blinking = 50;

	return Action;
}
//** End of UpdateMarker 


//*******************************************************************************
//
//  Name: SetOutputs 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  When there request for activating digital tags, the corresponding pins will be set 
//  by this function. Also present is the polling part, when the tag has to
//  be polled the information is send to HMI.
//
//
//*******************************************************************************
void SetOutputs ()
{

  long int CurrentTime;

  for (int Index = 1; Index <= (NoOutputPins); Index++)
  {
    if (!OutPin[Index].Auto) OutPin[Index].Changed = OutPin[Index].FixedStatus;
    if (OutPin[Index].Status != OutPin[Index].PreStatus) if ((OutPin[Index].Report) && (UseHMISerial)) HMISendString ( "@RDC," + String(OutPin[Index].Status) + "," + OutPin[Index].Tagname + "," + OutPin[Index].Auto + "," + OutPin[Index].DBLogging);
    OutPin[Index].PreStatus = OutPin[Index].Status;
    if (OutPin[Index].Changed)
    {
      OutPin[Index].Status = true;
	  if (OutPin[Index].Tagname.substring(0,1) != "_" )
		  digitalWrite(OutPin[Index].Pin, HIGH );
	  else
		  digitalWrite(OutPin[Index].Pin, LOW);
    }
    else
    {
		OutPin[Index].Status = false;
		if (OutPin[Index].Tagname.substring(0, 1) != "_")
			digitalWrite(OutPin[Index].Pin, LOW);
		else
			digitalWrite(OutPin[Index].Pin, HIGH);
    }
    OutPin[Index].Changed = false;
    // Polling part
    if ((OutPin[Index].Poll) && (UseHMISerial))
    {
      CurrentTime = millis();
      if (OutPin[Index].PollTime < CurrentTime)
      {
        HMISendString ( "@PDI" + String(OutPin[Index].Status) + OutPin[Index].Tagname + "," + OutPin[Index].Auto + "," + OutPin[Index].DBLogging);
        OutPin[Index].PollTime = OutPin[Index].PollTime + (OutPin[Index].PollFreq );
      }
    }
  }

  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if (!Marker[Index].Auto) Marker[Index].Changed = Marker[Index].FixedStatus;
	  if (Marker[Index].Status != Marker[Index].PreStatus) if ((Marker[Index].Report) && (UseHMISerial)) HMISendString("@RDC," + String(Marker[Index].Status) + "," + Marker[Index].Tagname + "," + Marker[Index].Auto + "," + Marker[Index].DBLogging);
	  Marker[Index].PreStatus = Marker[Index].Status;
	  if (Marker[Index].Changed)
	  {
		  Marker[Index].Status = true;
	  }
	  else
	  {
		  Marker[Index].Status = false;
	  }
	  Marker[Index].Changed = false;
	  // Polling part
	  if ((Marker[Index].Poll) && (UseHMISerial))
	  {
		  CurrentTime = millis();
		  if (Marker[Index].PollTime < CurrentTime)
		  {
			  HMISendString("@PDI" + String(Marker[Index].Status) + Marker[Index].Tagname + Marker[Index].Auto + "," + Marker[Index].DBLogging);
			  Marker[Index].PollTime = Marker[Index].PollTime + (Marker[Index].PollFreq);
		  }
	  }
  }



}
//** End of SetOutputs 


//*******************************************************************************
//
//  Name: UpdateAna 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Set a request for a new procesvalue (0-255) of an analogue output tag (PWN).
//
//
//*******************************************************************************
boolean UpdateAna (String Tag, int Waarde )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;

  for (int Index = 1; Index <= (NoAnaOutPins); Index++)
  {
    if ((AnaOutPin[Index].Tagname == Tag) && (Waarde <= 255))
    {
      if ( Waarde != AnaOutPin[Index].Value )
      {
        AnaOutPin[Index].PrevValue = AnaOutPin[Index].Value;
        AnaOutPin[Index].Changed = true;
        AnaOutPin[Index].Value = Waarde;
      }
      Action = true;
      TagFound = true;
    }
  }

  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if ((Marker[Index].Tagname == Tag) && (Waarde <= 255))
	  {
		  if (Waarde != Marker[Index].Value)
		  {
			  Marker[Index].PrevValue = Marker[Index].Value;
			  Marker[Index].ChangedAnalogue = true;
			  Marker[Index].Value = Waarde;
		  }
		  Action = true;
		  TagFound = true;
	  }
  }

  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of UpdateAna 

//*******************************************************************************
//
//  Name: 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  When there are request for changing values for analogue output tags, they
//  will be set by this function. Also present is the polling part for sending
//  messages to HMI. 
//
//
//*******************************************************************************
void SetAnaOutputs ()
{

  long int CurrentTime;

  for (int Index = 1; Index <= (NoAnaOutPins); Index++)
  {
    if ( AnaOutPin[Index].Changed == true )
    {
      if ( !AnaOutPin[Index].Auto ) AnaOutPin[Index].Value = AnaOutPin[Index].FixedValue;
      analogWrite ( AnaOutPin[Index].Pin, AnaOutPin[Index].Value );
      AnaOutPin[Index].Changed = false;
    }
    if ((AnaOutPin[Index].Poll) && (UseHMISerial))
    {
      CurrentTime = millis();
      if (AnaOutPin[Index].PollTime < CurrentTime)
      {
        HMISendString ( "@PAI," + LeadingZero(AnaOutPin[Index].Value) + "," + AnaOutPin[Index].Tagname + "," + AnaOutPin[Index].Auto + "," + AnaOutPin[Index].DBLogging);
        AnaOutPin[Index].PollTime = AnaOutPin[Index].PollTime + (AnaOutPin[Index].PollFreq );
      }
    }
  }

  
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if (Marker[Index].ChangedAnalogue == true)
	  {
		  if (!Marker[Index].Auto) Marker[Index].Value = Marker[Index].FixedValue;
		  Marker[Index].ChangedAnalogue = false;
	  }
	  if ((Marker[Index].Poll) && (UseHMISerial))
	  {
		  CurrentTime = millis();
		  if (Marker[Index].PollTimeAnalogue < CurrentTime)
		  {
			  HMISendString("@PAI," + LeadingZero(Marker[Index].Value) + "," + Marker[Index].Tagname + "," + Marker[Index].Auto + "," + Marker[Index].DBLogging);
			  Marker[Index].PollTimeAnalogue = Marker[Index].PollTimeAnalogue + (Marker[Index].PollFreq);
		  }
	  }
  }
  


}
//** End of SetOutputs 


//*******************************************************************************
//
//  Name: UpdateServo 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  A request is made for setting an angle for the selected servo (0-180).
//
//
//*******************************************************************************
boolean UpdateServo (String Tag, int Angle )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;
  for (int Index = 1; Index <= (NoServos); Index++)
  {
    if ((ServoPin[Index].Tagname == Tag) && (Angle <= 180))
    {
      if ( Angle != ServoPin[Index].Angle )
      {
        ServoPin[Index].Changed = true;
        ServoPin[Index].Angle = Angle;
      }
      Action = true;
      TagFound = true;
    }
  }

  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of UpdateServo 

//*******************************************************************************
//
//  Name: SetServos 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  All requests for setting a new angle for de servos are set by this function.
//  Polling request by HMI are also processed.
//
//
//*******************************************************************************
void SetServos ()
{

  long int CurrentTime;

  for (int Index = 1; Index <= (NoServos); Index++)
  {
    if ( ServoPin[Index].Changed == true )
    {
      if ( !ServoPin[Index].Auto ) ServoPin[Index].Angle = ServoPin[Index].FixedAngle;
      ServoPin[Index].MyServo.write ( ServoPin[Index].Angle );
      AnaOutPin[Index].Changed = false;
    }
    if ((ServoPin[Index].Poll) && (UseHMISerial))
    {
      CurrentTime = millis();
      if (ServoPin[Index].PollTime < CurrentTime)
      {
        HMISendString ( "@PAI," + LeadingZero(ServoPin[Index].Angle) + "," + ServoPin[Index].Tagname + "," + ServoPin[Index].Auto + "," + ServoPin[Index].DBlogging);
        ServoPin[Index].PollTime = ServoPin[Index].PollTime + (ServoPin[Index].PollFreq );
      }
    }
  }
}
//** End of SetServos 



//*******************************************************************************
//
//  Name: Timer 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  There are 3 different situations that can occur:
//   1. The timer identified by TimerNo is not known in the timer chain. 
//      A new entry is made in the chain and the timer is set. The function
//      returns false.
//   2. The timer identified by TimerNo is known but the the time is not expired.
//      he function returns false.
//   3. The timer identified by TimerNo is known and the time is now expired.
//      The timerNo is removed from the timer chain and the function returns true. 
//
//
//*******************************************************************************
boolean Timer (unsigned long Time, int TimerNo )
{
  boolean Action;

  Action = false;
  HuidigeSchakel = TimerKetting;
  VorigeSchakel = TimerKetting;
  
  while (( HuidigeSchakel->TimerId != 0 ) && ( HuidigeSchakel->TimerId != TimerNo ))
  {
    VorigeSchakel = HuidigeSchakel;
    HuidigeSchakel = HuidigeSchakel->Next;
  };

  // found an existing timer
  if ( HuidigeSchakel->TimerId == TimerNo )
  {
    if (( HuidigeSchakel->Wait - millis()) > HuidigeSchakel->Wait )
    {
      Action = true;
	  HuidigeSchakel->State = true;
    }
  }
  // Timer not found, create a new one...
  else
  {
    NieuweSchakel = new TimerType;
    NieuweSchakel->TimerId = TimerNo;
    NieuweSchakel->Initial = Time;
    NieuweSchakel->Wait = millis() + Time;
    NieuweSchakel->State = false;
	NieuweSchakel->OneShot = true;
    HuidigeSchakel = TimerKetting;
    NieuweSchakel->Next = HuidigeSchakel;
    TimerKetting = NieuweSchakel;
  }
  return Action ;
}
//** End of Timer 



//*******************************************************************************
//
//  Name: DelayTimer 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  There are 3 different situations that can occur:
//   1. The timer identified by TimerNo is not known in the timer chain. 
//      A new entry is made in the chain and the timer is set. The function
//      returns false.
//   2. The timer identified by TimerNo is known but the the time is not expired.
//      he function returns false.
//   3. The timer identified by TimerNo is known and the time is or was expired.
//      The timerNo is not removed from the timer chain and the function returns true. 
//
//   Remark: The timer id is not deleted after expiration! 
//           Returns always true after expiration. You have to call
//           CancelTimer for deleting this timer.
//
//
//*******************************************************************************
boolean DelayTimer(unsigned long Time, int TimerNo)
{
	boolean Action;

	Action = false;
	HuidigeSchakel = TimerKetting;
	VorigeSchakel = TimerKetting;
	// Search for the timer
	while ((HuidigeSchakel->TimerId != 0) && (HuidigeSchakel->TimerId != TimerNo))
	{
		VorigeSchakel = HuidigeSchakel;
		HuidigeSchakel = HuidigeSchakel->Next;
	};

	// found an existing timer
	if (HuidigeSchakel->TimerId == TimerNo)
	{
		if (((HuidigeSchakel->Wait - millis()) > HuidigeSchakel->Wait) || (HuidigeSchakel->State))
		{
			Action = true;
			HuidigeSchakel->State = true;
		}
	}
	// Timer not found, create a new one...
	else
	{
		NieuweSchakel = new TimerType;
		NieuweSchakel->TimerId = TimerNo;
		NieuweSchakel->Initial = Time;
		NieuweSchakel->Wait = millis() + Time;
		NieuweSchakel->State = false;
		NieuweSchakel->OneShot = false;
		HuidigeSchakel = TimerKetting;
		NieuweSchakel->Next = HuidigeSchakel;
		TimerKetting = NieuweSchakel;
	}
	return Action;
}
//** End of DelayTimer 

//*******************************************************************************
//
//  Name: RestartDelayTimer 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The timer identified by TimerNo is known and the time is reset.
//  he function returns false.
//
//*******************************************************************************
boolean RestartDelayTimer(int TimerNo)
{
	boolean Action;

	Action = false;
	HuidigeSchakel = TimerKetting;
	VorigeSchakel = TimerKetting;
	// Search for the timer
	while ((HuidigeSchakel->TimerId != 0) && (HuidigeSchakel->TimerId != TimerNo))
	{
		VorigeSchakel = HuidigeSchakel;
		HuidigeSchakel = HuidigeSchakel->Next;
	};

	// found an existing timer
	if (HuidigeSchakel->TimerId == TimerNo)
	{
		HuidigeSchakel->Wait = millis() + HuidigeSchakel->Initial;
		HuidigeSchakel->State = false;
	}
	return Action;
}
//** End of RestartDelayTimer 



//*******************************************************************************
//
//  Name: CancelTimer 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The timer identified by TimerNo is removed from the timer chain. When an
//  existing timer is removed the function return true. If the timer is not
//  found in the chain, the fucntion returns false.
//
//
//*******************************************************************************
boolean CancelTimer (int TimerNo )
{
  boolean Action;
  Action = false;
  HuidigeSchakel = TimerKetting;
  VorigeSchakel = TimerKetting;
  // Search for the timer
  while (( HuidigeSchakel->TimerId != 0 ) && ( HuidigeSchakel->TimerId != TimerNo ))
  {
    VorigeSchakel = HuidigeSchakel;
    HuidigeSchakel = HuidigeSchakel->Next;
  }

  // found an existing timer
  if ( HuidigeSchakel->TimerId == TimerNo )
  {
    Action = true;
    if ( HuidigeSchakel == TimerKetting ) // oh, het is de eerste al...
    {
      TimerKetting = HuidigeSchakel->Next;
      free ( HuidigeSchakel );
    }
    else
    {
      VorigeSchakel->Next = HuidigeSchakel->Next;
      free ( HuidigeSchakel );
    }
  }
  return Action ;
}
//** End of CancelTimer 


//*******************************************************************************
//
//  Name: RemoveExpTimers() 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Remove all expired, one shot timers from the timer list.
//
//*******************************************************************************
void RemoveExpTimers()
{
	HuidigeSchakel = TimerKetting;
	VorigeSchakel = TimerKetting;


	while (HuidigeSchakel->TimerId != 0)
	{
		//if (((HuidigeSchakel->Wait - millis() >= HuidigeSchakel->Wait)) && (HuidigeSchakel->OneShot))
		if (HuidigeSchakel->State && HuidigeSchakel->OneShot)
			{
			if (HuidigeSchakel == TimerKetting)
			{
				TimerKetting = HuidigeSchakel->Next;
				free(HuidigeSchakel);
				HuidigeSchakel = TimerKetting;
				VorigeSchakel = HuidigeSchakel;
			}
			else
			{
				VorigeSchakel->Next = HuidigeSchakel->Next;
				free(HuidigeSchakel);
				HuidigeSchakel = VorigeSchakel->Next;
			}
		}
		else //skip and goto next in the list
		{
			if (HuidigeSchakel == VorigeSchakel) HuidigeSchakel = HuidigeSchakel->Next;
			else
			{
				VorigeSchakel = HuidigeSchakel;
				HuidigeSchakel = HuidigeSchakel->Next;
			}
		}
	}


}

//** End of RemoveExpTimers 




//*******************************************************************************
//
//  Name: ReadTimers 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function reads all entries in the timer chain and presents it on
//  the serial monitor.
//
//
//*******************************************************************************
boolean ReadTimers ()
{
  boolean Action;
  Action = false;
  HuidigeSchakel = TimerKetting;
  Serial.println ( );
  Serial.println ( F("Overview active timers: "));
  Serial.println ( F("----------------------  "));
  while ( HuidigeSchakel->TimerId != 0 )
  {
    Serial.println ("Timer no: " + String(HuidigeSchakel->TimerId ) + ", set on " + String (HuidigeSchakel->Initial) + " mS" );
    if (( HuidigeSchakel->Wait - millis()) > HuidigeSchakel->Wait )
    {
      Serial.println ( F("Timer just expired"));
    }
    else
    {
      Serial.println ("still " + String ( (HuidigeSchakel->Wait - millis()) / 1000 ) + " Sec to go (" + String (HuidigeSchakel->Wait - millis()) + "mS)" );
    }
    Serial.println ();
    HuidigeSchakel = HuidigeSchakel->Next;
  }

  return Action ;

}
//** End of ReadTimers 



//*******************************************************************************
//
//  Name: ReadInputs 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function reads all defined digital inputs. If the
//  status is different from the old, the "changed" flag is set. When HMI asked 
//  for a message when the status is changed, it will send the message to HMI.
//  When the tag is masked by HMI, this value is overriding the field value.
//  When HMI asked to poll the tag, it is also handled by this function.
//  
//
//
//*******************************************************************************
void ReadInputs ()
{

  unsigned long CurrentTime;

  for (int InputIndex = 1; InputIndex <= (NoInputPins); InputIndex++)
  {
    if (InPin[InputIndex].Changed == true )
    {
      if ((InPin[InputIndex].Report) && (UseHMISerial)) HMISendString ( "@RDC," + String(InPin[InputIndex].Status) + "," + InPin[InputIndex].Tagname + "," + InPin[InputIndex].Mask + "," + InPin[InputIndex].DBLogging);
      InPin[InputIndex].Changed = false;
    };
    if (!InPin[InputIndex].Mask)
    {
      if (InPin[InputIndex].Status != digitalRead (InPin[InputIndex].Pin))
      {
        InPin[InputIndex].TimeChange = millis();
        InPin[InputIndex].Changed = true;
        InPin[InputIndex].Status = digitalRead (InPin[InputIndex].Pin);
      } ;
    };
    if (InPin[InputIndex].Mask)
    {
      if (InPin[InputIndex].Status != InPin[InputIndex].MaskValue)
      {
        InPin[InputIndex].TimeChange = millis();
        InPin[InputIndex].Changed = true;
        InPin[InputIndex].Status = InPin[InputIndex].MaskValue;
      } ;
    };
    // Polling part
    if ((InPin[InputIndex].Poll) && (UseHMISerial))
    {
      CurrentTime = millis();
      if (InPin[InputIndex].PollTime < CurrentTime)
      {
		HMISendString("@PDI," + String(InPin[InputIndex].Status) + "," + InPin[InputIndex].Tagname + "," + InPin[InputIndex].Mask + "," + InPin[InputIndex].DBLogging);
        InPin[InputIndex].PollTime = InPin[InputIndex].PollTime + (InPin[InputIndex].PollFreq );
      }
    }
  } // end of for loop reading inputs


} // End of ReadInputs



//*******************************************************************************
//
//  Name: UltraSonicDevice 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The trigger pin number, the echo pin number and the maximum distance (cm) are
//  the input for the function. A new instance of an ultrasonicdevice is made,
//  some time is reserved for initialization and than a "ping"  is made. The
//  result is returned in cm.
//
//
//*******************************************************************************
void UltraSonicDevice ( int PingPin, int EchoPin, int MaxDistance, int &Distance )
{
  int duration;
  Distance = 0;
  duration = 0;
  digitalWrite(PingPin, HIGH);
  delayMicroseconds(1000);
  digitalWrite(PingPin, LOW);
  duration = pulseIn(EchoPin, HIGH);
  Distance = (duration / 2) / 29.1 + 1;
  if (Distance >= MaxDistance || Distance <= 0) Distance = 0;
} // end of UltraSonicDevcie


//*******************************************************************************
//
//  Name: ReadUltrasonic 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function reads all defined ultrasonic devices that are not masked. 
//  When HMI asked for polling the tag, a message is send by this function. 
//
//
//*******************************************************************************
void ReadUltrasonic ()
{

  unsigned long CurrentTime;

  for (int InputIndex = 1; InputIndex <= (NoUltrasonic); InputIndex++)
  {
    if (!Ultrasonic[InputIndex].Mask) UltraSonicDevice ( Ultrasonic[InputIndex].Trigger, Ultrasonic[InputIndex].Echo, Ultrasonic[InputIndex].MaxDistance, Ultrasonic[InputIndex].Distance );
    // Polling part
    if ((Ultrasonic[InputIndex].Poll) && (UseHMISerial))
    {
      CurrentTime = millis();
      if (Ultrasonic[InputIndex].PollTime < CurrentTime) 
      {
        //if (Ultrasonic[InputIndex].Distance != 0) HMISendString ( "@PAI," + LeadingZero(Ultrasonic[InputIndex].Distance) + "," + Ultrasonic[InputIndex].Tagname + "," + Ultrasonic[InputIndex].Mask + "," + Ultrasonic[InputIndex].DBLogging);
		HMISendString("@PAI," + LeadingZero(Ultrasonic[InputIndex].Distance) + "," + Ultrasonic[InputIndex].Tagname + "," + Ultrasonic[InputIndex].Mask + "," + Ultrasonic[InputIndex].DBLogging);
		Ultrasonic[InputIndex].PollTime = Ultrasonic[InputIndex].PollTime + (Ultrasonic[InputIndex].PollFreq );
      }
    }
  } // end of for loop reading Ultrasonics
} // End of ReadUltrasonics


//*******************************************************************************
//
//  Name: ReadAnaIn 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Read all defined analogue input tags. When the tag is not masked, then 
//  the field value is read by the funcion. Also the polling part for the tags is
//  handled in the function. 
//
//
//*******************************************************************************
void ReadAnaIn ()
{

  long int CurrentTime;

  for (int InputIndex = 1; InputIndex <= (NoAnaInPins); InputIndex++)
  {
    if (!AnaInPin[InputIndex].Mask)
    {
      AnaInPin[InputIndex].PrevValue = AnaInPin[InputIndex].Value;
      AnaInPin[InputIndex].Value = analogRead (AnaInPin[InputIndex].Pin);
    }
    // Polling part
    if ((AnaInPin[InputIndex].Poll) && (UseHMISerial))
    {
      CurrentTime = millis();
      if (AnaInPin[InputIndex].PollTime < CurrentTime)
      {
        HMISendString ( "@PAI," + LeadingZero(AnaInPin[InputIndex].Value) + ","+ AnaInPin[InputIndex].Tagname + "," + AnaInPin[InputIndex].Mask + "," + AnaInPin[InputIndex].DBlogging);
        AnaInPin[InputIndex].PollTime = AnaInPin[InputIndex].PollTime + (AnaInPin[InputIndex].PollFreq );
      }
    }
  } // end of for loop reading analog inputs
} // End of ReadAnaIn


//*******************************************************************************
//
//  Name: ReadTemp 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  When not masked, the function reads the temperature. The temperature sensors
//  are of type BS18D20. They are read by means of one (1) PWN pin. Also polling 
//  the values are handled by the function.
//
//
//*******************************************************************************
void ReadTemp ()
{

  long int CurrentTime;

  //sensors.setWaitForConversion(true);
  sensors.requestTemperatures();

  for (int InputIndex = 1; InputIndex <= (NoTemp); InputIndex++)
  {
    if (!TempSensor[InputIndex].Mask)
    {
      TempSensor[InputIndex].PrevValue = TempSensor[InputIndex].Value;
      TempSensor[InputIndex].Value = (float)sensors.getTempCByIndex(InputIndex-1);
    }
    // Polling part
    if ((TempSensor[InputIndex].Poll) && (UseHMISerial))
    {
      CurrentTime = millis();
      if (TempSensor[InputIndex].PollTime < CurrentTime)
      {
        HMISendString ( "@PAI," + String(TempSensor[InputIndex].Value) + ","+ TempSensor[InputIndex].Tagname + "," + TempSensor[InputIndex].Mask + "," + TempSensor[InputIndex].DBLogging);
        TempSensor[InputIndex].PollTime = TempSensor[InputIndex].PollTime + (TempSensor[InputIndex].PollFreq );
      }
    }
  } // end of for loop reading analog inputs
} // End of ReadTemp


//*******************************************************************************
//
//  Name: CalcCycleCount  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  This function is called x times. Then the elapsed time is calculated
//  by x and than presented on the serial monitor.
//
//*******************************************************************************
void CalcCycleCount ()
{

  float Cycle;

  if (!CycleCalcHMI)
  {
	  if (NoOfCycle == 1) Serial.print(F("measuring, please wait "));
	  if (NoOfCycle == 200) Serial.print(F(" 5.."));
	  if (NoOfCycle == 400) Serial.print(F(" 4.."));
	  if (NoOfCycle == 600) Serial.print(F(" 3.."));
	  if (NoOfCycle == 800) Serial.print(F(" 2.."));
	  if (NoOfCycle == 1000) Serial.print(F(" 1.."));
  }

  NoOfCycle = NoOfCycle + 1;

  if ((NoOfCycle == 1200) && (!CycleCalcHMI))
  {
      Cycle = (millis() - CycleStart) / 1200.0;
      Serial.println ( " Cycle time = " + String (Cycle) + " mS" );
      CycleCalc = false;
      Serial.print(F("HelpMonitor > "));
  }

  if ((NoOfCycle == 1200) && (CycleCalcHMI))
  {
	  Cycle = (millis() - CycleStart) / 1200.0;
	  CycleCalc = false;
	  CycleCalcHMI = false;
	  HMIQueueMessage("@CLT," + String(Cycle),0);

  }
	  
}
//** end of CalcCycleCount

//*******************************************************************************
//
//  Name: SerMonitorCommand  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  When there is serial data available on the communication channel of the
//  serial monitor this is handled here. When typing "h" the help will be displayed.
//  When the monitor is not familiar with the command an error message is 
//  displayed, with the suggestion to type a "h"(elp).
//
//*******************************************************************************
void SerMonitorCommand ()

{
  if (Serial.available() > 0)
  {

    if (Serial.available() == 1)
    {
      // read the incoming byte:
      incomingByte = Serial.read();
      Serial.print(char(incomingByte));

      switch (incomingByte)
      {
        case 49:
          Serial.println("");
          InputOverview();
          break;
        case 50:
          Serial.println("");
          OutputOverview();
          break;
        case 51:
          Serial.println("");
          CycleCalc = true;
          NoOfCycle = 0;
          CycleStart = millis();
          break;
        case 52:
          Serial.println("");
          FSMStateOverview();
          break;
        case 53:
          Serial.println("");
          AnaInOverview();
          break;
        case 54:
          Serial.println("");
          AnaOutOverview();
          break;
        case 55:
          Serial.println("");
          ReadTimers();
          break;
        case 56:
          Serial.println("");
          ServoOverview();
          break;
        case 57:
          Serial.println("");
          UltraSonicOverview();
          break;
        case 65:
          Serial.println("");
          MarkerOverview();
          break;
        case 66:
          Serial.println("");
          Serial.println("Free RAM (bytes): " + String(getFreeMemory()));
          break;
        case 67:
          Serial.println("");
          TemperatureOverview();
          break;
		case 68:
		  HMIStatus();
		  break;
        case 104:
          Serial.println(" ");
          Serial.println(F("HELP"));
          Serial.println(F("1: Input overview"));
          Serial.println(F("2: Output overview"));
          Serial.println(F("3: Calc. cycle time" ));
          Serial.println(F("4: FSM State overview" ));
          Serial.println(F("5: Analoog input overview" ));
          Serial.println(F("6: Analoog output overview" ));
          Serial.println(F("7: Timer overview"));
          Serial.println(F("8: Servo overview"));
          Serial.println(F("9: UltraSonic overvieuw"));
          Serial.println(F("A: Marker overvieuw"));
          Serial.println(F("B: Free memory, check for leaks"));
          Serial.println(F("C: Temperature sensor overview"));
		  Serial.println(F("D: Status HMI"));
          Serial.println(F("T<unix-time-stamp>: for setting the time"));
          Serial.println(F("h: help"));
          Serial.println(" ");
          break;
        default:
          Serial.println(F("%INF-UNKCMD-SMC, Unknown command, type h for help"));
      };
    }
    else
    {
      if (Serial.available() == TIME_MSG_LEN )
      {
        processSyncMessage ();
        digitalClockDisplay ();
      }
      else
      {
        while ( Serial.available () != 0)
        {
          Serial.read();
        }
        Serial.println(F("%INF-UNKCMD-SMC, Unknown command, type h for help"));
      }
    }
  };

  Serial.print(F("HelpMonitor > "));

} // End of SerMonitorCommand

//*******************************************************************************
//
//  Name: HMIStatus
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Present the status of the HMI connection.
//
//*******************************************************************************
void HMIStatus()
{
	Serial.println();
	Serial.print(F("HMI connection status at "));
	digitalClockDisplay();
	Serial.println(F("----------------------------------------- "));
	Serial.print(F("HMI Connection status: "));
	if (UseHMISerial==false)
		{
			Serial.println(F("Disabled"));
	    }
	else
    	{
		if (HMIWatchDog) Serial.println(F("OK")); else Serial.println(F("Not connected"));
	    }
	Serial.println();

}

//*******************************************************************************
//
//  Name: DigRising  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The functione returns true when the tested tag is low in the prior and
//  high in the current cycle.
//
//*******************************************************************************
boolean DigRising ( String Tag )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;
  for (int InputIndex = 1; InputIndex <= (NoInputPins); InputIndex++)
  {
    if (InPin[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if ((InPin[InputIndex].Changed == true) && (InPin[InputIndex].Status == true) )
      {
        Action = true;
      }
    }
  }

  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of DigRising 

//*******************************************************************************
//
//  Name:  DigFalling
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function returns true when the tested tag is high in the prior and
//  high in the current cycle.
//
//*******************************************************************************
boolean DigFalling ( String Tag )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;
  for (int InputIndex = 1; InputIndex <= (NoInputPins); InputIndex++)
  {
    if (InPin[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if ((InPin[InputIndex].Changed == true) && (InPin[InputIndex].Status == false) )
      {
        Action = true;
      }
    }
  }

  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of DigFalling 

//*******************************************************************************
//
//  Name: DigLow  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function returns true when the tested digital tag is low.
//
//*******************************************************************************
//** Function DigLow
boolean DigLow ( String Tag )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;
  for (int InputIndex = 1; InputIndex <= (NoInputPins); InputIndex++)
  {
    if (InPin[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if (InPin[InputIndex].Status == false)
      {
        Action = true;
      }
    }
  }

  for (int OutputIndex = 1; OutputIndex <= (NoOutputPins); OutputIndex++)
  {
    if (OutPin[OutputIndex].Tagname == Tag)
    {
      TagFound = true;
      if (OutPin[OutputIndex].Status == false)
      {
        Action = true;
      }
    }
  }


  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of DigLow 

//*******************************************************************************
//
//  Name: DigHigh  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function returns true when the tested digital tag is high.
//
//*******************************************************************************
boolean DigHigh ( String Tag )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;
  for (int InputIndex = 1; InputIndex <= (NoInputPins); InputIndex++)
  {
    if (InPin[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if (InPin[InputIndex].Status == true)
      {
        Action = true;
      }
    }
  }

  for (int OutputIndex = 1; OutputIndex <= (NoOutputPins); OutputIndex++)
  {
    if (OutPin[OutputIndex].Tagname == Tag)
    {
      TagFound = true;
      if (OutPin[OutputIndex].Status == true)
      {
        Action = true;
      }
    }
  }


  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of DigHigh 

//*******************************************************************************
//
//  Name: Distance  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function returns the distance measured from the ultrasonic tag.
//
//*******************************************************************************
int Distance ( String Tag )
{
  int PV;
  boolean TagFound = false;
  PV = 0;
  for (int InputIndex = 1; InputIndex <= (NoUltrasonic); InputIndex++)
  {
    if (Ultrasonic[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      PV = Ultrasonic[InputIndex].Distance;
    }
  }

  if (!TagFound) Blinking = 50;

  return PV;
}
//** End of Distance 


//*******************************************************************************
//
//  Name: AnaValue  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The functione returns the value of the analogue tag.
//
//*******************************************************************************
int AnaValue ( String Tag )
{
  int PV;
  boolean TagFound = false;
  PV = 0;
  for (int InputIndex = 1; InputIndex <= (NoAnaInPins); InputIndex++)
  {
    if (AnaInPin[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      PV = AnaInPin[InputIndex].Value;
    }
  }

  if (!TagFound) Blinking = 50;

  return PV;
}
//** End of AnaValue 

//*******************************************************************************
//
//  Name: TempValue  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The functions returns the measured temperatuur of the temperature Tag
//
//*******************************************************************************
int TempValue ( String Tag )
{
  int PV;
  boolean TagFound = false;
  PV = 0;
  for (int InputIndex = 1; InputIndex <= (NoTemp); InputIndex++)
  {
    if (TempSensor[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      PV = TempSensor[InputIndex].Value;
    }
  }

  if (!TagFound) Blinking = 50;

  return PV;
}
//** End of TempValue 


//*******************************************************************************
//
//  Name: AnaBetween
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function returs true when the value of the analogue tag is between or
//  equal then the Low and High value (input parameters)
//
//*******************************************************************************
boolean AnaBetween ( String Tag, int LowVal, int HighVal )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;

  for (int InputIndex = 1; InputIndex <= (NoAnaInPins); InputIndex++)
  {
    if (AnaInPin[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if ((AnaInPin[InputIndex].Value >= LowVal) && (AnaInPin[InputIndex].Value <= HighVal))
      {
        Action = true;
      }
    }
  }

  for (int InputIndex = 1; InputIndex <= (NoTemp); InputIndex++)
  {
    if (TempSensor[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if ((TempSensor[InputIndex].Value >= LowVal) && (TempSensor[InputIndex].Value <= HighVal))
      {
        Action = true;
      }
    }
  }


  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of AnaBetween 

//*******************************************************************************
//
//  Name: AnaLower 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function returns true when the analogue tag value is lower then
//  the low value given as parameter.
//
//*******************************************************************************
//** Function AnaLower
boolean AnaLower ( String Tag, int LowVal )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;

  for (int InputIndex = 1; InputIndex <= (NoAnaInPins); InputIndex++)
  {
    if (AnaInPin[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if (AnaInPin[InputIndex].Value < LowVal)
      {
        Action = true;
      }
    }
  }

  for (int InputIndex = 1; InputIndex <= (NoTemp); InputIndex++)
  {
    if (TempSensor[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if ((TempSensor[InputIndex].Value < LowVal))
      {
        Action = true;
      }
    }
  }


  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of AnaLower 

//*******************************************************************************
//
//  Name: AnaHigher  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function returns true when the value of the analogue tag is higher then
//  the high value given as parameter.
//
//*******************************************************************************
boolean AnaHigher ( String Tag, int HighVal )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;

  for (int InputIndex = 1; InputIndex <= (NoAnaInPins); InputIndex++)
  {
    if (AnaInPin[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if (AnaInPin[InputIndex].Value > HighVal)
      {
        Action = true;
      }
    }
  }

  for (int InputIndex = 1; InputIndex <= (NoTemp); InputIndex++)
  {
    if (TempSensor[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if ((TempSensor[InputIndex].Value > HighVal))
      {
        Action = true;
      }
    }
  }


  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of AnaHigher 

//*******************************************************************************
//
//  Name: AnaGoUnder  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The funtion returns true when the value of the analogue tag is in the prior
//  cycle higher then the given limit in the current cylce.
//
//*******************************************************************************
//** Function AnaGoUnder
boolean AnaGoUnder ( String Tag, int LowVal )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;

  for (int InputIndex = 1; InputIndex <= (NoAnaInPins); InputIndex++)
  {
    if (AnaInPin[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if ((AnaInPin[InputIndex].PrevValue >= LowVal) && (AnaInPin[InputIndex].Value < LowVal))
      {
        Action = true;
      }
    }
  }

  for (int InputIndex = 1; InputIndex <= (NoTemp); InputIndex++)
  {
    if (TempSensor[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if ((TempSensor[InputIndex].PrevValue >= LowVal) && (TempSensor[InputIndex].Value < LowVal))
      {
        Action = true;
      }
    }
  }

  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of AnaGoUnder 

//*******************************************************************************
//
//  Name: AnaGoOver  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The functione return true when the value of the analogue tag is lower in
//  the prior cycle then the given limit in the current cycle.
//
//
//*******************************************************************************
//** Function AnaGoOver
boolean AnaGoOver ( String Tag, int HighVal )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;

  for (int InputIndex = 1; InputIndex <= (NoAnaInPins); InputIndex++)
  {
    if (AnaInPin[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if ((AnaInPin[InputIndex].PrevValue <= HighVal) && (AnaInPin[InputIndex].Value > HighVal))
      {
        Action = true;
      }
    }
  }

  for (int InputIndex = 1; InputIndex <= (NoTemp); InputIndex++)
  {
    if (TempSensor[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if ((TempSensor[InputIndex].PrevValue <= HighVal) && (TempSensor[InputIndex].Value > HighVal))
      {
        Action = true;
      }
    }
  }

  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of AnaGoUnder 

//*******************************************************************************
//
//  Name: DistanceBetween  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function returns true when the distance tag value is between or equal 
//  then the given low and high limits (parameters).
//
//
//*******************************************************************************
boolean DistanceBetween ( String Tag, int LowVal, int HighVal )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;
  for (int InputIndex = 1; InputIndex <= (NoUltrasonic); InputIndex++)
  {
    if (Ultrasonic[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if ((Ultrasonic[InputIndex].Distance >= LowVal ) && ( Ultrasonic[InputIndex].Distance <= HighVal ))
      {
        Action = true;
      }
    }
  }

  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of DistanceBetween 

//*******************************************************************************
//
//  Name: DistanceShorter  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function returns true when the value of the distance tag is shorter
//  then the given limit (parameter)
//
//*******************************************************************************
boolean DistanceShorter ( String Tag, int LowVal )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;
  for (int InputIndex = 1; InputIndex <= (NoUltrasonic); InputIndex++)
  {
    if (Ultrasonic[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if (Ultrasonic[InputIndex].Distance < LowVal )
      {
        Action = true;
      }
    }
  }

  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of DistanceShorter 

//*******************************************************************************
//
//  Name: DistanceLonger  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function returns true when the value of the distance tag is longer
//  then the given limit (parameter). 
//
//*******************************************************************************
boolean DistanceLonger ( String Tag, int HighVal )
{
  boolean Action;
  boolean TagFound = false;
  Action = false;
  for (int InputIndex = 1; InputIndex <= (NoUltrasonic); InputIndex++)
  {
    if (Ultrasonic[InputIndex].Tagname == Tag)
    {
      TagFound = true;
      if (Ultrasonic[InputIndex].Distance > HighVal )
      {
        Action = true;
      }
    }
  }

  if (!TagFound) Blinking = 50;

  return Action ;
}
//** End of DistanceLonger 


//*******************************************************************************
//
//  Name: MessageI2C 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Send a message over I2C by default port 9. A message is only send when it
//  is different from the prior message. The "wire" library is used.
//
//*******************************************************************************
void MessageI2C(String Message)
{

  int LenMessage;
  char STX = 3;
  char ETX = 4;
  String SFA;

  //** No identical messages over the line...!
  if (Message != OldMessageI2C)
  {
    OldMessageI2C = Message;

    LenMessage = Message.length();
    //Serial.println(String(Message) + " lengte:"+ String(LenMessage));

    Wire.beginTransmission(9); // transmit to device #9
    Wire.write( STX );
    Wire.endTransmission(); // stop transmitting

    for (int CharIndex = 0; CharIndex < LenMessage; CharIndex++ )
    {
      SFA = Message.substring ( CharIndex, CharIndex + 1 );
      Wire.beginTransmission(9); // transmit to device #9
      Wire.write( char(SFA[0]) );
      Wire.endTransmission(); // stop transmitting
    };

    Wire.beginTransmission(9); // transmit to device #9
    Wire.write( ETX );
    Wire.endTransmission(); // stop transmitting
  }
}
//** End of MessageI2C



//*******************************************************************************
//
//  Name: HandlingGPSSerial  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Process the message from GPS. 
//
//*******************************************************************************
void HandlingGPSSerial()
{

	int CommandNo;
	char inChar;
	int IndexOf[20];

	while (Serial2.available() > 0)
	{
		inChar = Serial2.read();
		if (inChar == LF)
		{
			// Check the incoming string and maybe it's an command, commands start with @... to begin
			if (GPSString.substring(0, 1) == "$")
			{
				// could be a command!! Looking in the existing commands...
				CommandNo = -1;
				for (int Index = 0; Index < NoGPSCommands; Index++)
				{
					if (GPSString.substring(1, 5) == GPSCommands[Index]) CommandNo = Index;
				}
				if (CommandNo > -1) switch (CommandNo)
				{
				case 0: 
					IndexOf[1] = GPSString.indexOf(',');
					for (int I = 2; I <= 20; I++)
					{
						IndexOf[I] = GPSString.indexOf(',', IndexOf[I-1] + 1);
					};
					//Serial.println("PUBX:");
					//Serial.println("UTC:" + GPSString.substring(IndexOf[2] + 1, IndexOf[2 + 1]));
					//Serial.println("SOG(km):" + GPSString.substring(IndexOf[11] + 1, IndexOf[11+1]));
					//Serial.println("No. of Sat:" + GPSString.substring(IndexOf[18] + 1, IndexOf[18 + 1]));
					//Serial.println("Hacc:" + GPSString.substring(IndexOf[9] + 1, IndexOf[9 + 1]));

					GPSdata.UTC = GPSString.substring(IndexOf[2] + 1, IndexOf[2 + 1]);
					GPSdata.Latitude = GPSString.substring(IndexOf[3] + 1, IndexOf[3 + 1]);
					GPSdata.NSIndicator = GPSString.substring(IndexOf[4] + 1, IndexOf[4 + 1]);
					GPSdata.Longitude = GPSString.substring(IndexOf[5] + 1, IndexOf[5 + 1]);
					GPSdata.EWIndicator = GPSString.substring(IndexOf[6] + 1, IndexOf[6 + 1]);
					GPSdata.NavStat = GPSString.substring(IndexOf[8] + 1, IndexOf[8 + 1]);
					GPSdata.Hacc = GPSString.substring(IndexOf[9] + 1, IndexOf[9 + 1]);
					GPSdata.SOG = GPSString.substring(IndexOf[11] + 1, IndexOf[11 + 1]);
					GPSdata.COG = GPSString.substring(IndexOf[12] + 1, IndexOf[12 + 1]);
					GPSdata.GU = GPSString.substring(IndexOf[18] + 1, IndexOf[18 + 1]);

					break;
				case 1: 
					//Serial.println("GPVTG");
					break;
				}
			}
			//Serial.println(">>>>> " + GPSString);
			GPSString = "";
		}
		else
		{
			// add it to the inputString:
			if (inChar != CR) GPSString += inChar;
		}
	}


} // End of Handling GPS Serial


  //*******************************************************************************
  //
  //  Name: HandlingIntercard  
  //
  //  Modification date: 
  //  Changed by: 
  //
  //  Function:
  //  Process the message from another 2560Mega. 
  //
  //*******************************************************************************
void HandlingIntercardSerial()
{

	int CommandNo;
	char inChar;
	int IndexOf[20];
	boolean MessageReceived = false;

	// All intercard variables are set to blank at the end of every cycle. When there is data availbale on serial3,
	// just one message is processed and one if the global Intercard variables is set. 
	// For one cycle this variable is available for the FSM loop. 

	while ((Serial3.available() > 0) && (!MessageReceived))
	{
		inChar = Serial3.read();
		if (inChar == LF)
		{
			// Check the incoming string and maybe it's an command, commands start with @... to begin
			if (IntercardString.substring(0, 1) == "@")
			{
				// could be a command!! Looking in the existing commands...
				CommandNo = -1;
				for (int Index = 0; Index < NoIntercardCommands; Index++)
				{
					if (IntercardString.substring(1, 7) == IntercardCommands[Index]) CommandNo = Index;
				}
				if (CommandNo > -1) MessageReceived = true;
				if (CommandNo > -1) switch (CommandNo)
				{
				case 0:
					// CHAR01
					IntercardChar01 = IntercardString.substring(7);
					//Serial.println("Received CHAR01: >>" + IntercardChar01 + "<<" );
					break;
				case 1:
					// CHAR10
					IntercardChar10 = IntercardString.substring(7);
					break;
				case 2:
					// INT001
					IntercardInt001 = IntercardString.substring(7);
					break;
				case 3:
					// INT003
					IntercardInt010 = IntercardString.substring(7);
					break;
				}
			}
			IntercardString = "";
		}
		else
		{
			// add it to the inputString:
			if (inChar != CR) IntercardString += inChar;
		}
	}

} // End of Handling Intercard Serial


//*******************************************************************************
//
//  Name: HandlingHMISerial  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Process the message from HMI. Standard by communication channel 1.
//  Message strings starts with the "@" karakter and are terminated by
//  the <LF> char. The next caracters after "@" aer standard 3 caracters long
//  and contain the message id. The rest of the string is submitted to
//  the function which handle the message. 
//
//
//*******************************************************************************
void HandlingHMISerial()
{

  int CommandNo;
  char inChar;
  char ETX = 03;

  while (Serial1.available() > 0)
  {
    inChar = Serial1.read();

	if ((inChar == LF) || (inChar == ETX ))
    {

      // Check the incoming string and maybe it's an command, commands start with @... to begin
      if (KarakterString.substring(0, 1) == "@")
      {
        // could be a command!! Looking in the existing commands...
        CommandNo = -1;
        for (int Index = 0; Index < NoHMICommands; Index++)
        {
          if (KarakterString.substring(1, 4) == HMICommands[Index]) CommandNo = Index;
        }
        ValidationId = KarakterString.substring(KarakterString.length()-4,KarakterString.length());
        KarakterString = KarakterString.substring(0,KarakterString.length()-5);

        //Serial.println ( "Message>" + KarakterString + "<, with valdation id:" + ValidationId + ", command index:" + String(CommandNo) );

        // ... still -1?
        if (CommandNo > -1) switch (CommandNo)
          {
            case 0: //@SDI
              SDI(KarakterString.substring(4));
              break;
            case 1: //@SAI
              SAI(KarakterString.substring(4));
              break;
            case 2: //@FDO
              FDO(KarakterString.substring(4));
              break;
            case 3: //@FAO
              FAO(KarakterString.substring(4));
              break;
            case 4: //@RTA
              RTA(KarakterString.substring(4));
              break;
            case 5: //@PDI
              PDI(KarakterString.substring(4));
              break;
            case 6: //@PAI
              PAI(KarakterString.substring(4));
              break;
            case 7: //@CAN
              CAN(KarakterString.substring(4));
              break;
            case 8: //@RDC
              RDC(KarakterString.substring(4));
              break;
            case 9: //@GMI
              GMI ();
              break;
            case 10: //@MDI
              MDI(KarakterString.substring(4));
              break;
            case 11: //@MAI
              MAI(KarakterString.substring(4));
              break;
            case 12: //@RMM
              RMM(KarakterString.substring(4));
              break;
            case 13: //@FSM
              FSM ();
              break;
            case 14: //@FSO
              FSO(KarakterString.substring(4));
              break;
			case 15: //@WTD
			  WatchDog();
			  break;
			case 16: //@PMT
			  PMT(KarakterString.substring(4));
			  break;
			case 17: //@FMT
     	      FMT(KarakterString.substring(4));
			  break;
			case 18: //@MTC
			  MTC(KarakterString.substring(4));
			  break;
			case 19: //@THM
			  THM(KarakterString.substring(4));
		      break;
			case 20: //@CLT
			  CLT();
			  break;
			case 21: //@RAM
			  RAM();
			  break;
			case 22: //@TIM
			  TIM();
			  break;
			case 23: //@JM0
			  ResetBoard();
			  break;
			case 24: //@RDB
		   	  RDB(KarakterString.substring(4));
			  break;
			case 25: //@CDB
			  CDB(KarakterString.substring(4));
			  break;
			case 26: //@CHM
				CHM();
				break;
		}
        else HMISendString ( F("%FAT-UNKCMD-BTSERIAL, Unknown command received" ));
      }
      else
      {
        HMISendString ( "%FAT-UNKCMD-BTSERIAL, String not processed: " + KarakterString);
      }
      KarakterString = "";
    }
    else
    {
      // add it to the inputString:
      if (inChar != CR) KarakterString += inChar;
    }
  }
}
//** End of Handling HMI Serial


//*******************************************************************************
//
//  Name: HandlingHMISerial2  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Process the message from HMI. Standard by communication channel 1.
//  Message strings starts with the "@" karakter and are terminated by
//  the <LF> char. The next caracters after "@" aer standard 3 caracters long
//  and contain the message id. The rest of the string is submitted to
//  the function which handle the message. 
//
//
//*******************************************************************************
void HandlingHMISerial2()
{

	int CommandNo;
	char inChar;
	char ETX = 03;

	while (Serial2.available() > 0)
	{
		inChar = Serial2.read();

		if ((inChar == LF) || (inChar == ETX))
		{

			// Check the incoming string and maybe it's an command, commands start with @... to begin
			if (KarakterString.substring(0, 1) == "@")
			{
				// could be a command!! Looking in the existing commands...
				CommandNo = -1;
				for (int Index = 0; Index < NoHMICommands; Index++)
				{
					if (KarakterString.substring(1, 4) == HMICommands[Index]) CommandNo = Index;
				}
				ValidationId = KarakterString.substring(KarakterString.length() - 4, KarakterString.length());
				KarakterString = KarakterString.substring(0, KarakterString.length() - 5);

				//Serial.println ( "Message>" + KarakterString + "<, with valdation id:" + ValidationId + ", command index:" + String(CommandNo) );

				// ... still -1?
				if (CommandNo > -1) switch (CommandNo)
				{
				case 0: //@SDI
					SDI(KarakterString.substring(4));
					break;
				case 1: //@SAI
					SAI(KarakterString.substring(4));
					break;
				case 2: //@FDO
					FDO(KarakterString.substring(4));
					break;
				case 3: //@FAO
					FAO(KarakterString.substring(4));
					break;
				case 4: //@RTA
					RTA(KarakterString.substring(4));
					break;
				case 5: //@PDI
					PDI(KarakterString.substring(4));
					break;
				case 6: //@PAI
					PAI(KarakterString.substring(4));
					break;
				case 7: //@CAN
					CAN(KarakterString.substring(4));
					break;
				case 8: //@RDC
					RDC(KarakterString.substring(4));
					break;
				case 9: //@GMI
					GMI();
					break;
				case 10: //@MDI
					MDI(KarakterString.substring(4));
					break;
				case 11: //@MAI
					MAI(KarakterString.substring(4));
					break;
				case 12: //@RMM
					RMM(KarakterString.substring(4));
					break;
				case 13: //@FSM
					FSM();
					break;
				case 14: //@FSO
					FSO(KarakterString.substring(4));
					break;
				case 15: //@WTD
					WatchDog();
					break;
				case 16: //@PMT
					PMT(KarakterString.substring(4));
					break;
				case 17: //@FMT
					FMT(KarakterString.substring(4));
					break;
				case 18: //@MTC
					MTC(KarakterString.substring(4));
					break;
				case 19: //@THM
					THM(KarakterString.substring(4));
					break;
				case 20: //@CLT
					CLT();
					break;
				case 21: //@RAM
					RAM();
					break;
				case 22: //@TIM
					TIM();
					break;
				case 23: //@JM0
					ResetBoard();
					break;
				case 24: //@RDB
					RDB(KarakterString.substring(4));
					break;
				case 25: //@CDB
					CDB(KarakterString.substring(4));
					break;
				case 26: //@CHM
					CHM();
					break;
				}
				else HMISendString(F("%FAT-UNKCMD-BTSERIAL, Unknown command received"));
			}
			else
			{
				HMISendString("%FAT-UNKCMD-BTSERIAL, String not processed: " + KarakterString);
			}
			KarakterString = "";
		}
		else
		{
			// add it to the inputString:
			if (inChar != CR) KarakterString += inChar;
		}
	}
}
//** End of Handling HMI Serial2




//*******************************************************************************
//
//  Name: WTD  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  When there is a connection with a HMI client, this client will send
//  every 5 sec a message to the board. When received and prccessed by this
//  function every 10 seconds; the boolean HMIWatchDog will stay true.
//
//*******************************************************************************
void WatchDog()
{
	unsigned long CurrentTime;

	CurrentTime = millis();
	if (CurrentTime < (HMIWatchDogTime + 10000))
	{
		HMIWatchDog = true;
		CancelTimer(TimHMIWtd);
		Timer(10000, TimHMIWtd);
	}
	HMIWatchDogTime = CurrentTime;
	HMISendString("@VAL," + ValidationId);
}
//** End of WTD

//*******************************************************************************
//
//  Name: SDI  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Send the digital value of the given tag to HMI.
//
//*******************************************************************************
void SDI ( String Tag )
{
  int Index;
  boolean TagFound = false;

  // Search all digital inputs
  for (int Index = 1; Index <= (NoInputPins); Index++)
  {
    if (InPin[Index].Tagname == Tag)
    {
      HMIQueueMessage ( "@SDI," + String(InPin[Index].Tagname) + "=" + String(InPin[Index].Status), 0 );
      TagFound = true;
    }
  }
  // Search all digital outputs
  for (int Index = 1; Index <= (NoOutputPins); Index++)
  {
    if (OutPin[Index].Tagname == Tag)
    {
      HMIQueueMessage ( "@SDI," + String(OutPin[Index].Tagname) + "=" + String(OutPin[Index].Status), 0);
      TagFound = true;
    }
  }

  // Search all digital markers
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if (Marker[Index].Tagname == Tag)
	  {
		  HMIQueueMessage("@SDI," + String(Marker[Index].Tagname) + "=" + String(Marker[Index].Status), 0);
		  TagFound = true;
	  }
  }


  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-SDI, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );
}
//** End of SDI

//*******************************************************************************
//
//  Name: SAI  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Send the value of the analogue tag to HMI.
//
//*******************************************************************************
void SAI ( String Tag )
{
  int Index;
  boolean TagFound = false;

  // Search all analogue inputs
  for (int Index = 1; Index <= (NoAnaInPins); Index++)
  {
    if (AnaInPin[Index].Tagname == Tag)
    {
      HMIQueueMessage( "@SAI," + String(AnaInPin[Index].Tagname) + "=" + String(AnaInPin[Index].Value),0 );
      TagFound = true;
    }
  }
  // Search all analogue outputs
  for (int Index = 1; Index <= (NoAnaOutPins); Index++)
  {
    if (AnaOutPin[Index].Tagname == Tag)
    {
      HMIQueueMessage ( "@SAI," + String(AnaOutPin[Index].Tagname) + "=" + String(AnaOutPin[Index].Value),0);
      TagFound = true;
    }
  }
  // Search all analogue markers
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if (Marker[Index].Tagname == Tag)
	  {
		  HMIQueueMessage("@SAI," + String(Marker[Index].Tagname) + "=" + String(Marker[Index].Value), 0);
		  TagFound = true;
	  }
  }
  
  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-SAI, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

}
//** End of SAI

//*******************************************************************************
//
//  Name: RDC  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  When this message is received, a change of the given digital tag will
//  force a message to HMI with the value of the digital.
//
//
//*******************************************************************************
void RDC ( String Tag )
{
  int Index;
  boolean TagFound = false;

  // Search all digital inputs
  for (int Index = 1; Index <= (NoInputPins); Index++)
  {
    if (InPin[Index].Tagname == Tag)
    {
      InPin[Index].Report = true;
      HMIQueueMessage ( "@RDC," + String(InPin[Index].Status) + "," + InPin[Index].Tagname + "," + InPin[Index].Mask + "," + InPin[Index].DBLogging, 0);
      TagFound = true;
    }
  }
  // Search all digital outputs
  for (int Index = 1; Index <= (NoOutputPins); Index++)
  {
    if (OutPin[Index].Tagname == Tag)
    {
      OutPin[Index].Report = true;
      HMIQueueMessage ( "@RDC," + String(OutPin[Index].Status) + "," + OutPin[Index].Tagname + "," + OutPin[Index].Auto + "," + OutPin[Index].DBLogging, 0 );
      TagFound = true;
    }
  }
  // Search all markers
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if (Marker[Index].Tagname == Tag)
	  {
		  Marker[Index].Report = true;
		  HMIQueueMessage("@RDC," + String(Marker[Index].Status) + "," + Marker[Index].Tagname + "," + Marker[Index].Auto + "," + Marker[Index].DBLogging, 0);
		  TagFound = true;
	  }
  }
  
  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-RDC, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

}
//** End of RDC


//*******************************************************************************
//
//  Name: RDB
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  When this message is received, the RDC messages, will have the DBLogging bit
//  set to high.
//
//
//*******************************************************************************
void RDB(String Tag)
{
	int Index;
	boolean TagFound = false;

	// Search all digital inputs
	for (int Index = 1; Index <= (NoInputPins); Index++)
	{
		if (InPin[Index].Tagname == Tag)
		{
			InPin[Index].DBLogging = true;
			HMIQueueMessage("@RDC," + String(InPin[Index].Status) + "," + InPin[Index].Tagname + "," + InPin[Index].Mask + "," + InPin[Index].DBLogging, 0);
			TagFound = true;
		}
	}
	// Search all digital outputs
	for (int Index = 1; Index <= (NoOutputPins); Index++)
	{
		if (OutPin[Index].Tagname == Tag)
		{
			OutPin[Index].DBLogging = true;
			HMIQueueMessage("@RDC," + String(OutPin[Index].Status) + "," + OutPin[Index].Tagname + "," + OutPin[Index].Auto + "," + OutPin[Index].DBLogging, 0);
			TagFound = true;
		}
	}
	// Search all markers
	for (int Index = 1; Index <= (NoMarkers); Index++)
	{
		if (Marker[Index].Tagname == Tag)
		{
			Marker[Index].DBLogging = true;
			TagFound = true;
		}
	}
	// Search all Analogue IN
	for (int Index = 1; Index <= (NoAnaInPins); Index++)
	{
		if (AnaInPin[Index].Tagname == Tag)
		{
			AnaInPin[Index].DBlogging = true;
			TagFound = true;
		}
	}
	// Search all Analogue Out
	for (int Index = 1; Index <= (NoAnaOutPins); Index++)
	{
		if (AnaOutPin[Index].Tagname == Tag)
		{
			AnaOutPin[Index].DBLogging = true;
			TagFound = true;
		}
	}
	// Search all Temperatures
	for (int Index = 1; Index <= (NoTemp); Index++)
	{
		if (TempSensor[Index].Tagname == Tag)
		{
			TempSensor[Index].DBLogging = true;
			TagFound = true;
		}
	}
	// Search all Servos
	for (int Index = 1; Index <= (NoServos); Index++)
	{
		if (ServoPin[Index].Tagname == Tag)
		{
			ServoPin[Index].DBlogging = true;
			TagFound = true;
		}
	}
	// Search all Ultrasonic
	for (int Index = 1; Index <= (NoUltrasonic); Index++)
	{
		if (Ultrasonic[Index].Tagname == Tag)
		{
			Ultrasonic[Index].DBLogging = true;
			TagFound = true;
		}
	}
	// Search all Markers
	for (int Index = 1; Index <= (NoMarkers); Index++)
	{
		if (Marker[Index].Tagname == Tag)
		{
			Marker[Index].DBLogging = true;
			TagFound = true;
		}
	}


	if (!TagFound) Serial.println(F("%ERR-UNKTAG-RDB, Unknown tagname received"));
	if (TagFound) HMISendString("@VAL," + ValidationId);

}
//** End of RDB



//*******************************************************************************
//
//  Name: MTC  
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  When this message is received, a change of the given text tag will
//  force a message to HMI with the value of the textstring.
//
//
//*******************************************************************************
void MTC(String Tag)
{
	int Index;
	boolean TagFound = false;

	// Search all markers
	for (int Index = 1; Index <= (NoMarkers); Index++)
	{
		if (Marker[Index].Tagname == Tag)
		{
			Marker[Index].Report = true;
			Serial.println("MTC Tagname received: " + Tag);
			HMIQueueMessage("@MTC," + String(Marker[Index].TextString) + "," + Marker[Index].Tagname + "," + Marker[Index].Auto, 0);
			TagFound = true;
		}
	}

	if (!TagFound) Serial.println(F("%ERR-UNKTAG-MTC, Unknown tagname received"));
	if (TagFound) HMISendString("@VAL," + ValidationId);

}
//** End of MTC


//*******************************************************************************
//
//  Name: FDO 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function will force the digital output tag to the value given in
//  the message from HMI. The digital output is no longer in automatic mode.
//  Changes of the digital by the FSM are ignored.
//
//
//*******************************************************************************
void FDO ( String Command )
{
  int Index;
  boolean TagFound = false;
  String Tag;
  String Bitje;

  // Search all digital outputs
  for (int Index = 1; Index <= (NoOutputPins); Index++)
  {
    if (OutPin[Index].Tagname == Command.substring(1))
    {
      TagFound = true;
      Bitje = Command.substring(0, 1);
      if ((Bitje == "1") || (Bitje == "0"))
      {
        if (Bitje == "1") OutPin[Index].FixedStatus = true;
        if (Bitje == "0") OutPin[Index].FixedStatus = false;
        OutPin[Index].Auto = false;
        HMIQueueMessage ( "@RDC," + String(OutPin[Index].Status) + "," + OutPin[Index].Tagname + "," + OutPin[Index].Auto + "," + OutPin[Index].DBLogging, 0 );
      }
      else
        HMISendString ( F("%FDO-UNKCMD-BTSERIAL, No boolean expression" ));
    }
  }
  // Search all markers
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if (Marker[Index].Tagname == Command.substring(1))
	  {
		  TagFound = true;
		  Bitje = Command.substring(0, 1);
		  if ((Bitje == "1") || (Bitje == "0"))
		  {
			  if (Bitje == "1") Marker[Index].FixedStatus = true;
			  if (Bitje == "0") Marker[Index].FixedStatus = false;
			  Marker[Index].Auto = false;
			  HMIQueueMessage("@RDC," + String(Marker[Index].Status) + "," + Marker[Index].Tagname + "," + Marker[Index].Auto + "," + Marker[Index].DBLogging, 0);
		  }
		  else
			  HMISendString(F("%FDO-UNKCMD-BTSERIAL, No boolean expression"));
	  }
  }

  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-FDO, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

}
//** End of FDO

//*******************************************************************************
//
//  Name: FAO 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function will force the value of the analogue output tag to the given
//  value by HMI. The analogue output tag is no longer in automatic mode.
//  Changes of the value by the FSM are ignored.
//
//*******************************************************************************
void FAO ( String Command )
{
  int Index;
  boolean TagFound = false;
  String Tag;
  String StrValue;
  int Value;

  // Search all analogue outputs
  for (int Index = 1; Index <= (NoAnaOutPins); Index++)
  {
    if (AnaOutPin[Index].Tagname == Command.substring(3))
    {
      TagFound = true;
      StrValue = Command.substring(0, 3);
      Value = 0;
      Value = StrValue.toInt();
      if ((Value >= 0) && (Value <= 255))
      {
        AnaOutPin[Index].Changed = true;
        AnaOutPin[Index].FixedValue = Value;
        AnaOutPin[Index].Auto = false;
      }
      else
        HMISendString ( F("%FAO-UNKCMD-BTSERIAL, No valid value") );
      TagFound = true;
    }
  }

  // Search all markers
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if (Marker[Index].Tagname == Command.substring(3))
	  {
		TagFound = true;
		StrValue = Command.substring(0, 3);
		Value = 0;
		Value = StrValue.toInt();
		Serial.println("FAO Tagname found:" + Marker[Index].Tagname + ", Value:" + String(Value));
		if ((Value >= 0) && (Value <= 255))
		{
		  Marker[Index].ChangedAnalogue = true;
		  Marker[Index].FixedValue = Value;
		  Marker[Index].Auto = false;
		}
		else
		  HMISendString(F("%FAO-UNKCMD-BTSERIAL, No valid value"));
		TagFound = true;
	  }
  }

  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-FAO, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

}
//** End of FAO

//*******************************************************************************
//
//  Name: FMT 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function will force the value of the marker tag to the given
//  value by HMI. 
//
//*******************************************************************************
void FMT(String Command)
{
	int Index;
	boolean TagFound = false;
	String Tag;
	String LenValueString;
	int Len;

	Serial.println("%INF-FMT-RECCMD, Received command: >>>" + Command + "<<<");

	LenValueString = Command.substring(0, 3);
	Len = LenValueString.toInt();

	// Search all markers
	for (int Index = 1; Index <= (NoMarkers); Index++)
	{
		if (Marker[Index].Tagname == Command.substring(Len+3))
		{
			TagFound = true;
         	Marker[Index].ChangedText = true;
			Marker[Index].Auto = false;
			Marker[Index].TextString = Command.substring(3,Len-1+4);
			TagFound = true;
			HMISendString("@RDC," + Marker[Index].TextString + "," + Marker[Index].Tagname + "," + Marker[Index].Auto + "," + Marker[Index].DBLogging);
		}
	}

	if (!TagFound) Serial.println(F("%ERR-UNKTAG-FMT, Unknown tagname received"));
	if (TagFound) HMISendString("@VAL," + ValidationId);

}
//** End of FMT


//*******************************************************************************
//
//  Name: FSO 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function will force the value of the servo tag to the given
//  value by HMI. The servo output tag is no longer in automatic mode.
//  Changes of the value by the FSM are ignored.
//
//*******************************************************************************//** FSO Force servo out
void FSO ( String Command )
{
  int Index;
  boolean TagFound = false;
  String Tag;
  String StrValue;
  int Value;

  // Search all analogue outputs
  for (int Index = 1; Index <= (NoServos); Index++)
  {
    if (ServoPin[Index].Tagname == Command.substring(3))
    {
      TagFound = true;
      StrValue = Command.substring(0, 3);
      Value = 0;
      Value = StrValue.toInt();
      if ((Value >= 0) && (Value <= 180))
      {
        ServoPin[Index].Changed = true;
        ServoPin[Index].FixedAngle = Value;
        ServoPin[Index].Auto = false;
      }
      else
        HMISendString ( F("%FAO-UNKCMD-BTSERIAL, No valid value") );
      TagFound = true;
    }
  }

  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-FSO, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

}
//** End of FSO


//*******************************************************************************
//
//  Name: MAI
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function forces the value of the analogue input tag to the given
//  value by HMI. This is called masking of the analogue input tag.
//  Changes of the value by the field are ignored.
//
//*******************************************************************************
void MAI ( String Command )
{
  int Index;
  boolean TagFound = false;
  String Tag;
  String StrValue;
  int Value;

  // Search all analogue inputs
  for (int Index = 1; Index <= (NoAnaInPins); Index++)
  {
    if (AnaInPin[Index].Tagname == Command.substring(4))
    {
      StrValue = Command.substring(0, 4);
      Value = 0;
      Value = StrValue.toInt();
      TagFound = true;
      if ((Value >= 0) && (Value <= 1023))
      {
        if (Value != AnaInPin[Index].Value) AnaInPin[Index].Changed = true;
        AnaInPin[Index].Value = Value;
        AnaInPin[Index].Mask = true;
      }
      else
        HMISendString ( F("%MAI-UNKCMD-BTSERIAL, No valid value") );
    }
  }

  // Search all ultrasonics
  for (int Index = 1; Index <= (NoUltrasonic); Index++)
  {
    if (Ultrasonic[Index].Tagname == Command.substring(4))
    {
      StrValue = Command.substring(0, 4);
      Value = 0;
      Value = StrValue.toInt();
      TagFound = true;
      if ((Value >= 0) && (Value <= Ultrasonic[Index].MaxDistance))
      {
        Ultrasonic[Index].Distance = Value;
        Ultrasonic[Index].Mask = true;
      }
      else
        HMISendString ( F("%MAI-UNKCMD-BTSERIAL, No valid value" ));
    }
  }

  // Search all temperature sensors
  for (int Index = 1; Index <= (NoTemp); Index++)
  {
    if (TempSensor[Index].Tagname == Command.substring(4))
    {
      StrValue = Command.substring(0, 4);
      Value = 0;
      Value = StrValue.toInt();
      TagFound = true;
      if ((Value >= -55) && (Value <= 125))
      {
        TempSensor[Index].Value = Value;
        TempSensor[Index].Mask = true;
      }
      else
        HMISendString ( F("%MAI-UNKCMD-BTSERIAL, No valid value" ));
    }
  }

  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-MAI, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

}
//** End of MAI


//*******************************************************************************
//
//  Name: MDI
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function forces the value of the digital input tag to the given
//  value by HMI. This is called masking of the digital input tag.
//  Changes of the value by the field are ignored.
//
//*******************************************************************************
void MDI ( String Command )
{
  int Index;
  boolean TagFound = false;
  String Tag;
  String Bitje;
  boolean MaskValue;

  // Search all digital inputs
  for (int Index = 1; Index <= (NoInputPins); Index++)
  {
    if (InPin[Index].Tagname == Command.substring(1))
    {
      TagFound = true;
      Bitje = Command.substring(0, 1);
      if ((Bitje == "1") || (Bitje == "0"))
      {
        if (Bitje == "1") MaskValue = true;
        if (Bitje == "0") MaskValue = false;
        InPin[Index].MaskValue = MaskValue;
        InPin[Index].Mask = true;
        HMIQueueMessage ( "@RDC," + String(InPin[Index].Status) + "," + String(InPin[Index].Tagname) + "," + String(InPin[Index].Status) + "," + InPin[Index].DBLogging, 0);
      }
      else
        HMISendString ( F("%MDI-UNKCMD-BTSERIAL, No boolean expression" ));
    }
  }

  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-MDI, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

}
//** End of MDI

//*******************************************************************************
//
//  Name: RMM
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function removes the mask from the input tag. Values from the field
//  will changes the input tags again. 
//
//*******************************************************************************
void RMM ( String Tag )
{
  int Index;
  boolean TagFound = false;

  // Search all digital inputs
  for (int Index = 1; Index <= (NoInputPins); Index++)
  {
    if (InPin[Index].Tagname == Tag)
    {
      InPin[Index].Mask = false;
      HMIQueueMessage ( "@RDC," + String(InPin[Index].Status) + "," + String(InPin[Index].Tagname) + ",0" + "," + InPin[Index].DBLogging, 0);
      TagFound = true;
    }
  }

  // Search all analogue inputs
  for (int Index = 1; Index <= (NoAnaInPins); Index++)
  {
    if (AnaInPin[Index].Tagname == Tag)
    {
      AnaInPin[Index].Mask = false;
      TagFound = true;
    }
  }

  // Search all ultrasonics
  for (int Index = 1; Index <= (NoUltrasonic); Index++)
  {
    if (Ultrasonic[Index].Tagname == Tag)
    {
      Ultrasonic[Index].Mask = false;
      TagFound = true;
    }
  }

  // Search all temperature sensors
  for (int Index = 1; Index <= (NoTemp); Index++)
  {
    if (TempSensor[Index].Tagname == Tag)
    {
      TempSensor[Index].Mask = false;
      TagFound = true;
    }
  }

  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-RMM, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

}
//** End of RMM


//*******************************************************************************
//
//  Name: RTA
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function forces the output tag to automatic mode.
//  The FMS has the control again.
//
//*******************************************************************************
void RTA ( String Tag )
{
  int Index;
  boolean TagFound = false;

  // Search all digital outputs
  for (int Index = 1; Index <= (NoOutputPins); Index++)
  {
    if (OutPin[Index].Tagname == Tag)
    {
      OutPin[Index].Auto = true;
      HMIQueueMessage ( "@RDC," + String(OutPin[Index].Status) + "," + OutPin[Index].Tagname + "," + OutPin[Index].Auto + "," + OutPin[Index].DBLogging, 0 );
      TagFound = true;
    }
  }

  // Search all ANALOGUE outputs
  for (int Index = 1; Index <= (NoAnaOutPins); Index++)
  {
    if (AnaOutPin[Index].Tagname == Tag)
    {
      AnaOutPin[Index].Auto = true;
      TagFound = true;
    }
  }

  // Search all servo's outputs
  for (int Index = 1; Index <= (NoServos); Index++)
  {
    if (ServoPin[Index].Tagname == Tag)
    {
      ServoPin[Index].Auto = true;
      TagFound = true;
    }
  }
  
  // Search all markers
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if (Marker[Index].Tagname == Tag)
	  {
		  Marker[Index].Auto = true;
		  TagFound = true;
		  //HMISendString("@RDC," + Marker[Index].TextString + "," + Marker[Index].Tagname + "," + Marker[Index].Auto  + "," + Marker[Index].DBLogging);
	  }
  }

  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-RTA, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

}
//** End of RTA

//*******************************************************************************
//
//  Name: PDI
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Poll the digital input tag. The message contains the tagname and the poll
//  frequency. Possible values are 0000..3600 sec. Actual sending of polling
//  is done in other functions.
//
//*******************************************************************************
void PDI ( String Command )
{
  int Index;
  boolean TagFound = false;
  String Tag;
  String StrPollFrequentie;
  unsigned long PollFrequentie;

  // Search all digital inputs
  for (int Index = 1; Index <= (NoInputPins); Index++)
  {
    if (InPin[Index].Tagname == Command.substring(4))
    {
      TagFound = true;
      StrPollFrequentie = Command.substring(0, 4);
      PollFrequentie = 0;
      PollFrequentie = StrPollFrequentie.toInt() * 1000;
      if ((PollFrequentie > 0) && (PollFrequentie <= 3600000))
      {
        InPin[Index].Poll = true;
        InPin[Index].PollFreq = PollFrequentie;
        InPin[Index].PollTime = millis();
      }
      else
        HMISendString ( F("%PDI-UNKCMD-BTSERIAL, poll freq not between 0..3600" ));
    }
  }

  // Search all digital outputs
  for (int Index = 1; Index <= (NoOutputPins); Index++)
  {
    if (OutPin[Index].Tagname == Command.substring(4))
    {
      TagFound = true;
      StrPollFrequentie = Command.substring(0, 4);
      PollFrequentie = 0;
      PollFrequentie = StrPollFrequentie.toInt() * 1000;
      if ((PollFrequentie > 0) && (PollFrequentie <= 3600000))
      {
        OutPin[Index].Poll = true;
        OutPin[Index].PollFreq = PollFrequentie;
        OutPin[Index].PollTime = millis();
      }
      else
        HMISendString ( F("%PDI-UNKCMD-BTSERIAL, poll freq not between 0..3600" ));
    }
  }

  // Search all digital markers
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if (Marker[Index].Tagname == Command.substring(4))
	  {
		  TagFound = true;
		  StrPollFrequentie = Command.substring(0, 4);
		  PollFrequentie = 0;
		  PollFrequentie = StrPollFrequentie.toInt() * 1000;
		  if ((PollFrequentie > 0) && (PollFrequentie <= 3600000))
		  {
			  Marker[Index].Poll = true;
			  Marker[Index].PollFreq = PollFrequentie;
			  Marker[Index].PollTime = millis();
		  }
		  else
			  HMISendString(F("%PDI-UNKCMD-BTSERIAL, poll freq not between 0..3600"));
	  }
  }

  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-PDI, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

}
//** End of PDI

//*******************************************************************************
//
//  Name: PMT
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Poll the marker text tag. The message contains the tagname and the poll
//  frequency. Possible values are 0000..3600 sec. Actual sending of polling
//  is done in other functions.
//
//*******************************************************************************
void PMT(String Command)
{
	int Index;
	boolean TagFound = false;
	String Tag;
	String StrPollFrequentie;
	unsigned long PollFrequentie;

	// Search all markers
	for (int Index = 1; Index <= (NoMarkers); Index++)
	{
		if (Marker[Index].Tagname == Command.substring(4))
		{
			TagFound = true;
			StrPollFrequentie = Command.substring(0, 4);
			PollFrequentie = 0;
			PollFrequentie = StrPollFrequentie.toInt() * 1000;
			if ((PollFrequentie > 0) && (PollFrequentie <= 3600000))
			{
				Marker[Index].Poll = true;
				Marker[Index].PollFreq = PollFrequentie;
				Marker[Index].PollTime = millis();
			}
			else
				HMISendString(F("%PDI-UNKCMD-BTSERIAL, poll freq not between 0..3600"));
		}
	}

	if (!TagFound) Serial.println(F("%ERR-UNKTAG-PDI, Unknown tagname received"));
	if (TagFound) HMISendString("@VAL," + ValidationId);

}
//** End of PMT

//*******************************************************************************
//
//  Name: CAN 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Turn the polling off for the given Tag.
//
//*******************************************************************************
void CAN ( String Tag )
{
  int Index;
  boolean TagFound = false;

  // Search all digital inputs
  for (int Index = 1; Index <= (NoInputPins); Index++)
  {
    if (InPin[Index].Tagname == Tag)
    {
      InPin[Index].Poll = false;
      InPin[Index].Report = false;
      TagFound = true;
    }
  }
  // Search all digital outputs
  for (int Index = 1; Index <= (NoOutputPins); Index++)
  {
    if (OutPin[Index].Tagname == Tag)
    {
      OutPin[Index].Poll = false;
      OutPin[Index].Report = false;
      TagFound = true;
    }
  }
  // Search all analogue inpute
  for (int Index = 1; Index <= (NoAnaInPins); Index++)
  {
    if (AnaInPin[Index].Tagname == Tag)
    {
      AnaInPin[Index].Poll = false;
      TagFound = true;
    }
  }
  // Search all analogue outpute
  for (int Index = 1; Index <= (NoAnaOutPins); Index++)
  {
    if (AnaOutPin[Index].Tagname == Tag)
    {
      AnaOutPin[Index].Poll = false;
      TagFound = true;
    }
  }
  // Search all ultrasonic
  for (int Index = 1; Index <= (NoUltrasonic); Index++)
  {
    if (Ultrasonic[Index].Tagname == Tag)
    {
      Ultrasonic[Index].Poll = false;
      TagFound = true;
    }
  }
  // Search all servos
  for (int Index = 1; Index <= (NoServos); Index++)
  {
    if (ServoPin[Index].Tagname == Tag)
    {
      ServoPin[Index].Poll = false;
      TagFound = true;
    }
  }
  // Search all temperatures
  for (int Index = 1; Index <= (NoTemp); Index++)
  {
    if (TempSensor[Index].Tagname == Tag)
    {
      TempSensor[Index].Poll = false;
      TagFound = true;
    }
  }
  // Search all markers
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if (Marker[Index].Tagname == Tag)
	  {
		  Marker[Index].Poll = false;
		  TagFound = true;
	  }
  }
  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-CAN, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

}
//** End of CAN


//*******************************************************************************
//
//  Name: CHM
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Turn the polling off for all Tags.
//
//*******************************************************************************
void CHM()
{
	int Index;
	boolean TagFound = false;

	// Search all digital inputs
	for (int Index = 1; Index <= (NoInputPins); Index++)
	{
			InPin[Index].Poll = false;
			InPin[Index].Report = false;
	}
	// Search all digital outputs
	for (int Index = 1; Index <= (NoOutputPins); Index++)
	{
			OutPin[Index].Poll = false;
			OutPin[Index].Report = false;
	}
	// Search all analogue inpute
	for (int Index = 1; Index <= (NoAnaInPins); Index++)
	{
			AnaInPin[Index].Poll = false;
	}
	// Search all analogue outpute
	for (int Index = 1; Index <= (NoAnaOutPins); Index++)
	{
			AnaOutPin[Index].Poll = false;
	}
	// Search all ultrasonic
	for (int Index = 1; Index <= (NoUltrasonic); Index++)
	{
			Ultrasonic[Index].Poll = false;
	}
	// Search all servos
	for (int Index = 1; Index <= (NoServos); Index++)
	{
			ServoPin[Index].Poll = false;
	}
	// Search all temperatures
	for (int Index = 1; Index <= (NoTemp); Index++)
	{
			TempSensor[Index].Poll = false;
	}
	// Search all markers
	for (int Index = 1; Index <= (NoMarkers); Index++)
	{
			Marker[Index].Poll = false;
	}

	if (!TagFound) Serial.println(F("%ERR-UNKTAG-CAN, Unknown tagname received"));
	if (TagFound) HMISendString("@VAL," + ValidationId);

}
//** End of CHM




//*******************************************************************************
//
//  Name: CDB
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Disable database logging boolean.
//
//*******************************************************************************
void CDB(String Tag)
{
	int Index;
	boolean TagFound = false;

	// Search all digital inputs
	for (int Index = 1; Index <= (NoInputPins); Index++)
	{
		if (InPin[Index].Tagname == Tag)
		{
			InPin[Index].DBLogging = false;
			HMIQueueMessage("@RDC," + String(InPin[Index].Status) + "," + InPin[Index].Tagname + "," + InPin[Index].Mask + "," + InPin[Index].DBLogging, 0);
			TagFound = true;
		}
	}
	// Search all digital outputs
	for (int Index = 1; Index <= (NoOutputPins); Index++)
	{
		if (OutPin[Index].Tagname == Tag)
		{
			OutPin[Index].DBLogging = false;
			HMIQueueMessage("@RDC," + String(OutPin[Index].Status) + "," + OutPin[Index].Tagname + "," + OutPin[Index].Auto + "," + OutPin[Index].DBLogging, 0);
			TagFound = true;
		}
	}
	// Search all Analogue IN
	for (int Index = 1; Index <= (NoAnaInPins); Index++)
	{
		if (AnaInPin[Index].Tagname == Tag)
		{
			AnaInPin[Index].DBlogging = false;
			HMIQueueMessage("@RDC," + String(AnaInPin[Index].Value) + "," + AnaInPin[Index].Tagname + "," + AnaInPin[Index].Mask + "," + AnaInPin[Index].DBlogging, 0);
			TagFound = true;
		}
	}
	// Search all Analogue Out
	for (int Index = 1; Index <= (NoAnaOutPins); Index++)
	{
		if (AnaOutPin[Index].Tagname == Tag)
		{
			AnaOutPin[Index].DBLogging = false;
			HMIQueueMessage("@RDC," + String(AnaOutPin[Index].Value) + "," + AnaOutPin[Index].Tagname + "," + AnaOutPin[Index].Auto + "," + AnaOutPin[Index].DBLogging, 0);
			TagFound = true;
		}
	}
	// Search all Temperatures
	for (int Index = 1; Index <= (NoTemp); Index++)
	{
		if (TempSensor[Index].Tagname == Tag)
		{
			TempSensor[Index].DBLogging = false;
			HMIQueueMessage("@RDC," + String(TempSensor[Index].Value) + "," + TempSensor[Index].Tagname + "," + TempSensor[Index].Mask + "," + TempSensor[Index].DBLogging, 0);
			TagFound = true;
		}
	}
	// Search all Servos
	for (int Index = 1; Index <= (NoServos); Index++)
	{
		if (ServoPin[Index].Tagname == Tag)
		{
			ServoPin[Index].DBlogging = false;
			HMIQueueMessage("@RDC," + String(ServoPin[Index].Angle) + "," + ServoPin[Index].Tagname + "," + ServoPin[Index].Auto + "," + ServoPin[Index].DBlogging, 0);
			TagFound = true;
		}
	}
	// Search all Ultrasonic
	for (int Index = 1; Index <= (NoUltrasonic); Index++)
	{
		if (Ultrasonic[Index].Tagname == Tag)
		{
			Ultrasonic[Index].DBLogging = false;
			HMIQueueMessage("@RDC," + String(Ultrasonic[Index].Distance) + "," + Ultrasonic[Index].Tagname + "," + Ultrasonic[Index].Mask + "," + Ultrasonic[Index].DBLogging, 0);
			TagFound = true;
		}
	}
	// Search all Markers
	for (int Index = 1; Index <= (NoMarkers); Index++)
	{
		if (Marker[Index].Tagname == Tag)
		{
			Marker[Index].DBLogging = false;
			TagFound = true;
		}
	}



	if (!TagFound) Serial.println(F("%ERR-UNKTAG-PDI, Unknown tagname received"));
	if (TagFound) HMISendString("@VAL," + ValidationId);
}
//** End of CDB



//*******************************************************************************
//
//  Name: PAI
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Turn on polling of analogue tags. The message contains the tagname and
//  poll frequency (0000..3600 sec).
//
//*******************************************************************************
void PAI ( String Command )
{
  int Index;
  boolean TagFound = false;
  String Tag;
  String StrPollFrequentie;
  long int PollFrequentie;

  // Search all analogue inputs
  for (int Index = 1; Index <= (NoAnaInPins); Index++)
  {
    if (AnaInPin[Index].Tagname == Command.substring(4))
    {
      TagFound = true;
      StrPollFrequentie = Command.substring(0, 4);
      PollFrequentie = 0;
      PollFrequentie = StrPollFrequentie.toInt() * 100;
      if ((PollFrequentie > 0) && (PollFrequentie <= 3600000))
      {
        AnaInPin[Index].Poll = true;
        AnaInPin[Index].PollFreq = PollFrequentie;
        AnaInPin[Index].PollTime = millis() - PollFrequentie;
      }
      else
        HMISendString ( F("%PDI-UNKCMD-BTSERIAL, poll freq not between 0..3600" ));
    }
  }

  // Search all analogue outputs
  for (int Index = 1; Index <= (NoAnaOutPins); Index++)
  {
    if (AnaOutPin[Index].Tagname == Command.substring(4))
    {
      TagFound = true;
      StrPollFrequentie = Command.substring(0, 4);
      PollFrequentie = 0;
      PollFrequentie = StrPollFrequentie.toInt() * 100;
      if ((PollFrequentie > 0) && (PollFrequentie <= 3600000))
      {
        AnaOutPin[Index].Poll = true;
        AnaOutPin[Index].PollFreq = PollFrequentie;
        AnaOutPin[Index].PollTime = millis();
    }
      else
        HMISendString ( F("%PAI-UNKCMD-BTSERIAL, poll freq not between 0..3600" ));
    }
  }

  // Search all Ultrasonics
  for (int Index = 1; Index <= (NoUltrasonic); Index++)
  {
    if (Ultrasonic[Index].Tagname == Command.substring(4))
    {
      TagFound = true;
      StrPollFrequentie = Command.substring(0, 4);
      PollFrequentie = 0;
      PollFrequentie = StrPollFrequentie.toInt() * 100;
      if ((PollFrequentie > 0) && (PollFrequentie <= 3600000))
      {
        Ultrasonic[Index].Poll = true;
        Ultrasonic[Index].PollFreq = PollFrequentie;
        Ultrasonic[Index].PollTime = millis();
      }
      else
        HMISendString ( F("%PAI-UNKCMD-BTSERIAL, poll freq not between 0..3600" ));
    }
  }

  // Search all servos
  for (int Index = 1; Index <= (NoServos); Index++)
  {
    if (ServoPin[Index].Tagname == Command.substring(4))
    {
      TagFound = true;
      StrPollFrequentie = Command.substring(0, 4);
      PollFrequentie = 0;
      PollFrequentie = StrPollFrequentie.toInt() * 100;
      if ((PollFrequentie > 0) && (PollFrequentie <= 3600000))
      {
        ServoPin[Index].Poll = true;
        ServoPin[Index].PollFreq = PollFrequentie;
        ServoPin[Index].PollTime = millis();
      }
      else
        HMISendString ( F("%PAI-UNKCMD-BTSERIAL, poll freq not between 0..3600" ));
    }
  }

  // Search all Temperatures
  for (int Index = 1; Index <= (NoTemp); Index++)
  {
    if (TempSensor[Index].Tagname == Command.substring(4))
    {
      TagFound = true;
      StrPollFrequentie = Command.substring(0, 4);
      PollFrequentie = 0;
      PollFrequentie = StrPollFrequentie.toInt() * 100;
      if ((PollFrequentie > 0) && (PollFrequentie <= 3600000))
      {
        TempSensor[Index].Poll = true;
        TempSensor[Index].PollFreq = PollFrequentie;
        TempSensor[Index].PollTime = millis();
      }
      else
        HMISendString ( F("%PAI-UNKCMD-BTSERIAL, poll freq not between 0..3600" ));
    }
  }

  // Search all Markers
  for (int Index = 1; Index <= (NoMarkers); Index++)
  {
	  if (Marker[Index].Tagname == Command.substring(4))
	  {
		  TagFound = true;
		  StrPollFrequentie = Command.substring(0, 4);
		  PollFrequentie = 0;
		  PollFrequentie = StrPollFrequentie.toInt() * 100;
		  if ((PollFrequentie > 0) && (PollFrequentie <= 3600000))
		  {
			  Marker[Index].Poll = true;
			  Marker[Index].PollFreq = PollFrequentie;
			  Marker[Index].PollTime = millis();
		  }
		  else
			  HMISendString(F("%PAI-UNKCMD-BTSERIAL, poll freq not between 0..3600"));
	  }
  }

  if (!TagFound) Serial.println ( F("%ERR-UNKTAG-PAI, Unknown tagname received") );
  if (TagFound) HMISendString ( "@VAL," + ValidationId );

  
}
//** End of PAI


//*******************************************************************************
//
//  Name: GMI
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  When this message is received from HMI, send all meta infotmation of all
//  defined tags to HMI. All information of the tags is send, number of different
//  I/O, connected pins, name, mode, value.
//  Message to HMI are send in several different messages depending on 
//  the I/O-type.
//
//*******************************************************************************
void GMI ()
{

  String MetaInfo;
  boolean Auto = true;
  boolean Mask = false;

  CancelTimer ( TimDelayGMI );
  DelayGMI = true;
  UseHMISerial = true; // Prevent the sending of HMI messages. 
  

  HMISendString ( "@VAL," + ValidationId );

  // All digital Inputs
  if (NoInputPins > 0 )
  {
    MetaInfo = "@GMIDI," + String(NoInputPins) + ",";
    for (int Index = 1; Index <= (NoInputPins); Index++)
    {
      if (InPin[Index].Mask) Mask = true;
      MetaInfo = MetaInfo + InPin[Index].Tagname + "," + InPin[Index].Pin + "," + InPin[Index].Report + "," + InPin[Index].Poll + "," + InPin[Index].Mask;
      if (Index < NoInputPins) MetaInfo = MetaInfo + ",";
    };
    HMIQueueMessage( String(MetaInfo), 0);
  }

  // All digital outputs
  if (NoOutputPins > 0 )
  {
    MetaInfo = "@GMIDO," + String(NoOutputPins) + ",";
    for (int Index = 1; Index <= (NoOutputPins); Index++)
    {
      if (!OutPin[Index].Auto) Auto = false;
      MetaInfo = MetaInfo + OutPin[Index].Tagname + "," + OutPin[Index].Pin + "," + OutPin[Index].Report + "," + OutPin[Index].Poll + "," + OutPin[Index].Auto;
      if (Index < NoOutputPins) MetaInfo = MetaInfo + "," ;
    };
    HMIQueueMessage( String(MetaInfo), 0);
  }

  // All Analogue Inputs
  if (NoAnaInPins > 0 )
  {
    MetaInfo = "@GMIAI," + String(NoAnaInPins) + ",";
    for (int Index = 1; Index <= (NoAnaInPins); Index++)
    {
      if (AnaInPin[Index].Mask) Mask = true;
      MetaInfo = MetaInfo + AnaInPin[Index].Tagname + "," + AnaInPin[Index].Pin + "," + AnaInPin[Index].Poll + "," + AnaInPin[Index].Mask;
      if (Index < NoAnaInPins) MetaInfo = MetaInfo + ",";
    };
    HMIQueueMessage( String(MetaInfo), 0);
  }

  // All analogue outputs
  if (NoAnaOutPins > 0 )
  {
    MetaInfo = "@GMIAO," + String(NoAnaOutPins) + ",";
    for (int Index = 1; Index <= (NoAnaOutPins); Index++)
    {
      if (!AnaOutPin[Index].Auto) Auto = false;
      MetaInfo = MetaInfo + AnaOutPin[Index].Tagname + "," + AnaOutPin[Index].Pin + "," + AnaOutPin[Index].Poll + "," + AnaOutPin[Index].Auto;
      if (Index < NoAnaOutPins) MetaInfo = MetaInfo + ",";
    };
    HMIQueueMessage( String(MetaInfo), 0);

  }

  // All Servo outputs
  if (NoServos > 0 )
  {
    MetaInfo = "@GMISO," + String(NoServos) + ",";
    for (int Index = 1; Index <= (NoServos); Index++)
    {
      if (!ServoPin[Index].Auto) Auto = false;
      MetaInfo = MetaInfo + ServoPin[Index].Tagname + "," + ServoPin[Index].Pin + "," + ServoPin[Index].Poll + "," + ServoPin[Index].Auto;
      if (Index < NoServos) MetaInfo = MetaInfo + ",";
    };
    HMIQueueMessage( String(MetaInfo), 0);
  }

  // All Ultrasonic outputs
  if (NoUltrasonic > 0 )
  {
    MetaInfo = "@GMIDM," + String(NoUltrasonic) + ",";
    for (int Index = 1; Index <= (NoUltrasonic); Index++)
    {
      if (Ultrasonic[Index].Mask) Mask = true;
      MetaInfo = MetaInfo + Ultrasonic[Index].Tagname + "," + Ultrasonic[Index].Trigger + "," + Ultrasonic[Index].Echo + "," + Ultrasonic[Index].Poll + "," + Ultrasonic[Index].Mask + "," + Ultrasonic[Index].MaxDistance;
      if (Index < NoUltrasonic) MetaInfo = MetaInfo + ",";
    };
    HMIQueueMessage( String(MetaInfo), 0);
  }

  // All Temperatures
  if (NoTemp > 0)
  {
    MetaInfo = "@GMITT," + String(NoTemp) + "," + String(OneWireChannel) + ",";
    for (int Index = 1; Index <= (NoTemp); Index++)
    {
      if (TempSensor[Index].Mask) Mask = true;
      MetaInfo = MetaInfo + TempSensor[Index].Tagname + "," + TempSensor[Index].DeviceNr + "," + TempSensor[Index].Poll + "," + TempSensor[Index].Mask;
      if (Index < NoTemp) MetaInfo = MetaInfo + ",";
    };
    HMIQueueMessage( String(MetaInfo), 0);
  }

  // All markers
  if (NoMarkers > 0)
  {
	  MetaInfo = "@GMIMK," + String(NoMarkers) + ",";
	  for (int Index = 1; Index <= (NoMarkers); Index++)
	  {
		  if (!Marker[Index].Auto) Auto = false;
		  MetaInfo = MetaInfo + Marker[Index].Tagname + "," + Marker[Index].Report + "," + Marker[Index].Poll + "," + Marker[Index].Auto;
		  if (Index < NoMarkers) MetaInfo = MetaInfo + ",";
	  };
	  HMIQueueMessage(String(MetaInfo), 0);
  }

  //if (!Auto) HMISendString ( F("@GMIBM,MANUAL" )); else HMISendString ( F("@GMIBM,AUTOMATIC") );
  //if (Mask) HMISendString ( F("@GMIMS,MASK" )); else HMISendString ( F("@GMIMS,AUTOMATIC") );

}
//** End of GMI


//*******************************************************************************
//
//  Name: FSM 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Send the active FSM states to HMI in seperate messages.
//  
//
//*******************************************************************************
void FSM ()
{

  FSMStateSchakel = FSMStateKetting;
  HMISendString ( "@VAL," + ValidationId );

  do
  {
  if (FSMStateSchakel->ActualState == "RUN") HMISendString("@FSM," + String(FSMStateSchakel->FSMStateName) + ",RUN");
  if (FSMStateSchakel->ActualState != "RUN") HMISendString("@FSM," + String(FSMStateSchakel->FSMStateName) + ",HIB");
  FSMStateSchakel = FSMStateSchakel->Next;
  } while ( FSMStateSchakel != 0);


} // End of FSM

//*******************************************************************************
//
//  Name: THM (time HMI master)
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Set the time receives from HMI. If a RTC is present, update the RTC also.
//  
//
//*******************************************************************************
void THM(String UnixTime)
{

	HMISendString("@VAL," + ValidationId);

    time_t pctime = 0;
	char c[10];

	UnixTime.toCharArray(c, 10);
    for (int i = 0; i < 10; i++) 
	    {
		pctime = (10 * pctime) + (c[i] - '0'); // convert digits to a number
		};
	setTime(pctime);
	if (RTCAvailable)
		{
		RealTimeClock.adjust(pctime);
		//Serial.println(F("%INF-RTC-UPDATE, RTC synchronous with HMI time"));
		}


} // End of THM

//*******************************************************************************
//
//  Name: CLT (Cycle Time )
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Measure the cycle time of the board
//  
//
//*******************************************************************************
void CLT()
{

	HMISendString("@VAL," + ValidationId);

	if (!CycleCalc)
	{
	  CycleCalcHMI = true; 
	  CycleCalc = true;
	  NoOfCycle = 0;
	  CycleStart = millis();
	}


} // End of CLT

//*******************************************************************************
//
//  Name: RAM (Free RAM memory)
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Free RAM
//  
//
//*******************************************************************************
void RAM()
{

	HMISendString("@VAL," + ValidationId);
	HMISendString("@RAM," + String(getFreeMemory()));

} // End of RAM

//*******************************************************************************
//
//  Name: TIM (Send Time of the board)
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Send the time of the board to HMI (just for verification)
//  
//
//*******************************************************************************
void TIM()
{
	String TimeOfTheDay;

	HMISendString("@VAL," + ValidationId);
	TimeOfTheDay = String(now());
	HMISendString("@TIM," + TimeOfTheDay);

} // End of TIM


//*******************************************************************************
//
//  Name: ResetBoard()
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Perform a soft reset on the board
//  
//
//*******************************************************************************
void ResetBoard()
{
	asm volatile (" jmp 0");

} // End of ResetBoard






//*******************************************************************************
//
//  Name: HMIQueueMessage
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Send a message to HMI (with or without a delay). 
//  When send with a delay, the message is queued. 
//
//*******************************************************************************
void HMIQueueMessage ( String Message, unsigned long DelayTime )
{
  //If there is no delay, then send the message directly
  
  if (DelayTime == 0) HMISendString (Message);
  else
    {
      
     if (MessageKetting == 0)
       {
        //There is no chain yet
        NieuweMessage = new MessageType;
        NieuweMessage->Message = Message;
        NieuweMessage->SendTime = millis() + DelayTime;
        NieuweMessage->Next = 0;
        NieuweMessage->Previous = 0;
        MessageKetting = NieuweMessage;
       }
     else
       {
       //There is a chain..! Connect the new message to the chain (at the end)
       NieuweMessage = new MessageType;
       NieuweMessage->Message = Message;
       NieuweMessage->SendTime = millis() + DelayTime;
       NieuweMessage->Next = 0;
       HuidigeMessage = MessageKetting;
       while (HuidigeMessage->Next != 0) HuidigeMessage = HuidigeMessage->Next;
       //At the end of the chain... connect the new message
       HuidigeMessage->Next = NieuweMessage;
       NieuweMessage->Previous = HuidigeMessage;
       };

    }
} 
// End of QueueMessage


//*******************************************************************************
//
//  Name: HMIFromQueue
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The function looks in the message queue for message with an expired timer.
//  When a message is found, it will be send to HMI.
//
//*******************************************************************************
void HMIFromQueue ()
{
  //Examine the queue and send message when the timer is expired

  boolean done;
  MessageType* Obsolete;
  
  HuidigeMessage = MessageKetting;
  VorigeMessage = MessageKetting;
     
  do
  {
    done = false;
    
    // Just one entry in the queue
    if ((HuidigeMessage->Next == 0) && (HuidigeMessage == MessageKetting))
      {
      if (HuidigeMessage->SendTime < millis())
        {
          Obsolete = HuidigeMessage; 
          HMISendString (HuidigeMessage->Message);
          MessageKetting = 0;
          HuidigeMessage = 0;
          free(Obsolete);
          done = true;
        } 
      }
      
    // First one in the queue chain
    if (HuidigeMessage != 0) if ((HuidigeMessage = MessageKetting) && (HuidigeMessage->Next != 0) && (!done))
      {
      if (HuidigeMessage->SendTime < millis())
        {
          Obsolete = HuidigeMessage;
          HMISendString (HuidigeMessage->Message);
          MessageKetting = HuidigeMessage->Next;
          VolgendeMessage = HuidigeMessage->Next;
          VolgendeMessage->Previous = 0;
          HuidigeMessage = MessageKetting;
          free(Obsolete);
          done = true;
        }  
      }
      
    // At the end of the queue chain
    if ((HuidigeMessage->Next == 0) && ( HuidigeMessage != MessageKetting) && (!done))
      {
      if (HuidigeMessage->SendTime < millis())
        {
          Obsolete = HuidigeMessage;
          HMISendString (HuidigeMessage->Message);
          VorigeMessage = HuidigeMessage->Previous;
          VorigeMessage->Next = 0;
          HuidigeMessage = 0;
          free(Obsolete);
          done = true;
        }  
      };
      
    // In the middle of the queue chain
    if (HuidigeMessage != 0) if ((HuidigeMessage->Next != 0) && (HuidigeMessage->Previous != 0) && (!done))
      {
      if (HuidigeMessage->SendTime < millis())
        {
          Obsolete = HuidigeMessage;
          HMISendString (HuidigeMessage->Message);
          VorigeMessage = HuidigeMessage->Previous;
          VolgendeMessage = HuidigeMessage->Next;
          VorigeMessage->Next = VolgendeMessage;
          VolgendeMessage->Previous = VorigeMessage;
          HuidigeMessage = VorigeMessage;          
          free(Obsolete);
        }  
      }  
      
    //One step further in the chain... if possible
    if (HuidigeMessage != 0) if (HuidigeMessage->Next != 0) HuidigeMessage = HuidigeMessage->Next;
    
  } 
  while ((HuidigeMessage != 0) && (!done));
    
}  
// End of SendMessage


//*******************************************************************************
//
//  Name: Isr
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Example of an interrupt routine. Each time it is called by means of an 
//  interrupt, a counter is incremented.
//
//*******************************************************************************
void Isr ()
{
  ISR_Count = ISR_Count + 1;
}
//** end Interrupt service routine



//*******************************************************************************
//
//  Name: #include "UserFiniteStateMachine.h"
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  The Include contains de User part of teh Finite State Machine
//
//*******************************************************************************
#include "UserFiniteStateMachine.h"
//** End of #include "UserFiniteStateMachïne.h"


//*******************************************************************************
//**  SETUP  ********************************************************************
//***********  SETUP  ***********************************************************
//********************  SETUP  **************************************************
//*****************************  SETUP  *****************************************
//**************************************  SETUP  ********************************
//*******************************************************************************
//
//  Name: setup 
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  Alle actions for initializing the FSM are processed.
//  - Initialize the serial monitor channel
//  - Initialize the communication channel to HMI (when used)
//  - Initialize the watchdog LED (pin13)
//  - Set up the timer pointer list.
//  - Check the configuration of the defined I/O tags
//  - Initialize the RTC
//  - Initialize all defined I/O
//  - Define the initial state of the FSM; START
//  - Send the GMI messages to HMI (when applied) 
//
//*******************************************************************************
void setup()  {


  int i;
  time_t pctime;
  DateTime Now;
  int IntYear;
  int IntMonth;
  int IntDay;
  int IntHour;
  int IntMinute;
  int IntSecond;

  Serial.begin(9600);     
  Serial.println(F("Arduino 2560 "));
  Serial.println(F("Program made by Siemonsma"));
  Serial.println("");
  Serial.println(F("Standard v40, 160516.001"));
  Serial.println("");
  Serial.println("");

  if (UseHMISerial) 
    {
      pinMode(19,INPUT_PULLUP);
	  Serial1.begin(9600, SERIAL_8N1);
  }

  // For the GPS
  if (UseGPS)
  {
	  pinMode(17, INPUT_PULLUP);
	  Serial2.begin(9600);
	  //Serial2.println(F("$PUBX,40,RMC,0,0,0,0*47")); //RMC OFF
	  //Serial2.println(F("$PUBX,40,VTG,0,0,0,0*5E")); //VTG OFF
	  //Serial2.println(F("$PUBX,40,GGA,0,0,0,0*5A")); //CGA OFF
	  //Serial2.println(F("$PUBX,40,GSA,0,0,0,0*4E")); //GSA OFF
	  //Serial2.println(F("$PUBX,40,GSV,0,0,0,0*59")); //GSV OFF
	  //Serial2.println(F("$PUBX,40,GLL,0,0,0,0*5C")); //GLL OFF
  }

  if (UseIntercard)
  {
	  pinMode(15, INPUT_PULLUP);
	  Serial3.begin(9600, SERIAL_8N1);
  }

  // Used as watchdog
  pinMode(13, OUTPUT);

  //Cycle controle
  NoOfCycle = 0;

  // initialize the timer chain
  TimerSchakel.TimerId = 0;
  TimerSchakel.Wait = 0;
  TimerSchakel.State = false;
  TimerSchakel.OneShot = false;
  TimerKetting = new TimerType;
  *TimerKetting = TimerSchakel;
  MessageKetting = 0;

  // do some initialization
  CConfigurationOK = true;
 
  if ( CConfigurationOK == true )
  {
    // Start I2C bus
    Wire.begin(); 


	//Spindle.setMaxSpeed(350.0);
	//Spindle.setAcceleration(500.0);
	
	RTCAvailable = true;
	if (!RealTimeClock.begin())    
       { 
       Serial.println(F("%INF-RTC-UTS, Unable to sync with the RTC"));
       RTCAvailable = false;
       }
    else
       {
	   Now = RealTimeClock.now(); 
	   IntYear = int(Now.year());
	   IntMonth = int(Now.month());
	   IntDay = int(Now.day());
	   IntHour = int(Now.hour());
	   IntMinute = int(Now.minute());
	   IntSecond = int(Now.second());
	   setTime(IntHour, IntMinute, IntSecond, IntDay, IntMonth, IntYear);
       Serial.print(F("%INF-RTC-STM, RTC time is : "));      
       digitalClockDisplay ();
       }
    Serial.println(F("%INF-CNFOK-SETUP, Configuration ok"));
    Serial.println();
    InitInputs();
    InitOutputs();
    InitAnaIn();
    InitAnaOut();
    InitServos();
    InitUltrasonics();
    InitTemp();
    InitMarkers();
    InitFSMStates();
	sensors.begin ();
    // Begin with state START
    TransitionToState("START");
    if (UseHMISerial) GMI(); 

  }
  
  //Interrupt handler, example
  //pinMode ( 20, INPUT );
  //digitalWrite ( 20, HIGH);
  //attachInterrupt ( Pin20, Isr, RISING );

  //*============= START USER SETUP ======= START USER SETUP =====================

  //Kp = 1;
  //Ki = 1;
  //Kd = 0.85;
  //Setpoint = 90;
  //Controller1.SetOutputLimits(0,180);
  //Controller1.SetTunings ( Kp, Ki, Kd );
  //Controller1.SetSampleTime ( 150 );
  //Controller1.SetMode(AUTOMATIC);

  //*============  END USER SETUP  =======  END USER SETUP  ======================

  Serial.print(F("HelpMonitor > "));

}

//*******************************************************************************
//**  LOOP   ********************************************************************
//***********  LOOP   ***********************************************************
//********************  LOOP   **************************************************
//*****************************  LOOP   *****************************************
//**************************************  LOOP   ********************************
//*******************************************************************************
//
//  Name: loop
//
//  Modification date: 
//  Changed by: 
//
//  Function:
//  - Check result of configuration test
//  - Read all input tags
//  - Start the FSM  
//
//*******************************************************************************

void loop()  
{

  boolean ActionResult;
  long positions[1];

  //*============================================================================
  //*  Standard actions on front of cycle. Don't change.

  if ( CConfigurationOK == false )
  {
    Serial.println("");
    Serial.println(F("%FAT-CNFFAILURE-SETUP, Failure, code not executed or stopt."));
    delay(2000);
    if ( Pin13 == 1 ) {
      Pin13 = 0;
      digitalWrite (13, Pin13);
    }
    else {
      Pin13 = 1;
      digitalWrite (13, Pin13);
    }
  }
  else
  { //begin else from the main loop

	  if (NoInputPins != 0) ReadInputs();
	  if (NoAnaInPins != 0) ReadAnaIn();
	  if (Timer(1000, TimUS)) if (NoUltrasonic != 0) ReadUltrasonic();
	  if (Timer(1000, TimTT)) if (NoTemp != 0) ReadTemp();
	  if (Timer(5000, TimGpsPoll)) if (UseGPS) Serial2.println("$PUBX,00*33");

	  //  when a HMI client is connected, it will send a message every 2 seconds, triggering timer
	  //  TimHMIWtd for 4 seconds again and setting HMIWatchDoog to true. So when there HMIWatchDog is true,
	  //  check the timer, if it is expired, turn HMIWatchDog to false;
	  //  In the program you can do what ever you want with the boolean HMIWatchDog. Up to you.

	  if (HMIWatchDog == true)
	  {
		  if (Timer(10000, TimHMIWtd)) HMIWatchDog = false;
		  if (Pin13 == 0)
		  {
			  if (Timer(260, TimBlinkWtdOn)) digitalWrite(13, 1);
			  if (Timer(280, TimBlinkWtdOff))
			  {
				  digitalWrite(13, 0);
				  CancelTimer(TimBlinkWtdOn);
			  }
		  }
	  };


	  // when flashing 500mS (output 13), the configuration is OK
	  if (Timer(Blinking, TimBlinking))
	  {
		  CancelTimer(TimBlinking);
		  if (Pin13 == 1)
		  {
			  Pin13 = 0;
			  digitalWrite(13, Pin13);
		  }
		  else
		  {
			  Pin13 = 1;
			  digitalWrite(13, Pin13);
		  }
	  };

	  if (Timer(60000, TimISR))
	  {
		  //To prevent ISR_count to change intermediate bij the ISR function, interrups will be blocked to make a copy of ISR_Count
		  noInterrupts();
		  ISR_Count_Copy = ISR_Count;
		  //... and enabled again
		  interrupts();
		  //Serial.println ( "ISR called for " + String (ISR_Count_Copy) + " times");
	  }

	  if (DelayGMI)
	  {
		  // If DelayHMI is true, the HMI messages are turned off.
		  // Wait for a certain time and turn the messages on again.
		  // This is done to allow HMI to process the GMI messages.
		  if (Timer(1500, TimDelayGMI))
		  {
			  UseHMISerial = true;
			  DelayGMI = false;
		  }
	  }

	  MachineState = CurrentState();
	  do
	  {


		  //*  End of Standard actions on front of cycle
		  //*========================================================================================



		  //*========================================================================================
		  //*  Application code
		  //*
		  //*  Name: UserFSM
		  //*  Version:
		  //*  Date:
		  //*  Author:
		  //*
		  //*  Short description:
		  //*
		  //*
		  //*========================================================================================
		  //*========================================================================================
		  //*============ START USER APPLICATION ======= START USER APPLICATION =====================
		  //*========================================================================================
		  //*========================================================================================
		  //
		  // State machine
		  //
		  // UserFSM contains the FSM software.
		  // Each phase exists of 2 parts, e.g. the phase actions and transition conditions.
		  // In the phase actions, all activated tags are mentioned. Deactivation is not needed, deactivation is
		  // done at the end of the main loop. In the conditions section, all conditions for the transitions to another
		  // phase are mentioned. When all condition are true, the new phase can be set. It is possible to programm
		  // more then 1 transition. In such a case it is necassery to keep the priority in mind.
		  //

	      
		  UserFSM();
          

          //*========================================================================================
          //*========================================================================================
          //*============  END USER APPLICATION ========= END USER APPLICATION =====================
          //*========================================================================================
          //*========================================================================================

          //*  Standard actions on the end of cycle. Don't change.

      MachineState = CurrentState();
    } while ( MachineState != 0 );

    ComToRunState();
	RemoveExpTimers();


    if ( CycleCalc == true ) CalcCycleCount();

    // Outputs mentioned in the phases are set. Digitals not activated are reset, unchanged analogs and servos keep there value!
    SetOutputs();
    SetAnaOutputs();
    SetServos();
	//SetMarkerText();

    // If there is inputdata on the serial monitor available?
    if (Serial.available()) 
      {
        delay(10);
        SerMonitorCommand();
      }

	// If there is inputdata available on the 2e serial line (Supervisor channel)?
	if ((UseHMISerial) || (DelayGMI))
	{
		if (Serial1.available()) HandlingHMISerial();
		HMIFromQueue();
	}

	// Serial2 used for GPS....(?)
	if (UseGPS)
	{
		//if (Serial2.available()) HandlingGPSSerial();
		if (Serial2.available()) HandlingHMISerial2();
	}

	// Serial 3, used for MEGA card communication
	if (UseIntercard)
	{
		IntercardChar01 = "";
		IntercardChar10 = "";
		IntercardInt001 = "";
		IntercardInt010 = "";
		if (Serial3.available()) HandlingIntercardSerial();
	}

  } //end else main loop after okee configuration  
}


