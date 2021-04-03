// UserFiniteStateMachine.h

//*========================================================================================
//*  Application code
//*
//*  Name: UserFSM()
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
// Each phase exists of 2 parts, e.g. the phase actions and transition conditions.
// In the phase actions, all activated tags are mentioned. Deactivation is not needed, deactivation is
// done at the end of the main loop. In the conditions section, all conditions for the transitions to another
// phase are mentioned. When all condition are true, the new phase can be set. It is possible to programm
// more then 1 transition. In such a case it is necassery to keep the priority in mind.
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
//*========================================================================================


void UserFSM()
{



	if (MachineState == FiniteState("START"))
    { 
      //Enter actions, run once
      if (EnterState()) {}; 
      //State actions  
      {};  
      //Conditions and transitions  
      {};  
      //Exit actions, run once
      if (ExitState()) {}; 
    };

	if (MachineState == FiniteState("LED-OFF"))
    { 
      //Enter actions, run once
      if (EnterState()) {}; 
      //State actions  
      {};  
      //Conditions and transitions  
      {};  
      //Exit actions, run once
      if (ExitState()) {}; 
    };

	if (MachineState == FiniteState("LED-ON"))
    { 
      //Enter actions, run once
      if (EnterState()) {}; 
      //State actions  
      {};  
      //Conditions and transitions  
      {};  
      //Exit actions, run once
      if (ExitState()) {}; 
    };












	//*%END USER APPLICATION Generation directive dont remove!=================================
	//*========================================================================================
	//*========================================================================================
}
