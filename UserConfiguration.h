// UserConfiguation.h
//
//
//*========================================================================================
//*========================================================================================
//*============  START USER CONFIGURATION FOR THE I/O   ===================================
//*========================================================================================
//*========================================================================================
//
// Define all used pins for the application.
//
// The range of digitals is commenly used. First to define the inputs,outputs and ultrasonics.
// The analog out en servo pins are in the PWN range of the board.
//
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
//
//%STARTGENERATION Generation directive, dont remove!
const byte NoInputPins  = 1;
const byte NoOutputPins = 1;
const byte NoUltrasonic = 3;
const byte NoAnaInPins  = 1;
const byte NoAnaOutPins = 1;
const byte NoServos     = 1;
const byte NoTemp       = 2;
String DummyTags[]      = { "" };
String InputTags[]      = { "DI1" };
String InputPins[]      = { "D22" };
String OutputTags[]     = { "DO1" };
String OutputPins[]     = { "D41" };
String AnaInTags[]      = { "AI1" };
String AnaInPins[]      = { "AI03" };
String AnaOutTags[]     = { "AO1" };
String AnaOutPins[]     = { "PWM05" };
String ServoTags[]      = { "SV1" };
String ServoPins[]      = { "PWM04" };
String UltraTags[]      = { "DM1","DM2","DM3" };
String UltraPins[]      = { "D43","D44","D45","D46","D47","D48" };
const byte OneWireChannel = 02;
String TempTags[]       = { "T1","T2" };
String TempPins[]       = { "PWM02" };
//%ENDGENERATION Generation directive dont remove!
// 
// Markers not yet implemented in the configurator
const byte NoMarkers = 0;
String MarkerTags[] = { "" };
//
// Declaration of all used states in the FSM.
// The first and last state "START"and "END" are obligatory
// and may not be removed, the rest is up to you
//
//%STARTGENERATION Generation directive, dont remove!
String PossibleFSMStates[] = { "START","LED-OFF","LED-ON","END" };
//%ENDGENERATION Generation directive, dont remove!
//
// Start defining User timers don't use predefined timers (stay out of range 200-300)
//
const byte timTask1 = 1;
const byte timTask2 = 2;
//
// End defining user tinmers
//
// START USER SPECIFIC DECLARATIONS
//
// Start User varaibles:
//
boolean AanUitTask1;
boolean AanUitTask2;
// End User varaibles
//
// use of the extra MEGA board for the messages, use MessageI2C and/or UseHMISerial, do not remove, only set true or false
//
//%STARTGENERATION Generation directive, dont remove!
boolean UseI2C = false;
boolean UseHMISerial = true;
boolean UseGPS = false;
boolean UseIntercard = false;
//%ENDGENERATION Generation directive, dont remove!
//
//*========================================================================================
//*========================================================================================
//*==============  END USER CONFIGURATION =================================================
//*========================================================================================
//*========================================================================================
