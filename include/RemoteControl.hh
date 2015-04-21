/*
 * This file is part of RHexLib, 
 *
 * Copyright (c) 2001 The University of Michigan, its Regents,
 * Fellows, Employees and Agents. All rights reserved, and distributed as
 * free software under the following license.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * 1) Redistributions of source code must retain the above copyright
 * notice, this list of conditions, the following disclaimer and the
 * file called "CREDITS" which accompanies this distribution.
 * 
 * 2) Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions, the following disclaimer and the file
 * called "CREDITS" which accompanies this distribution in the
 * documentation and/or other materials provided with the distribution.
 * 
 * 3) Neither the name of the University of Michigan, Ann Arbor or the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*********************************************************************
 * $Id: RemoteControl.hh,v 1.5 2001/07/20 02:00:31 ulucs Exp $
 *
 * The Remote Controller interface module
 *
 * This file contains the definition of the RemoteControl class 
 * and all the associated definitions. The member function definitions
 * are in RemoteControl.cc
 *
 * The remote control module presents a somewhat flexible interface to
 * the Hardware dials, mostly useful when they are connected to actual
 * R/C hardware. It does some filtering, provides facilities to avoid 
 * unwanted noise effects etc.
 *
 * Created       : Uluc Saranli, 01/25/2001
 * Last Modified : Eric Klavins, 07/19/2001
 *
 ********************************************************************/

#ifndef _REMOTECONTROL_HH
#define _REMOTECONTROL_HH

// Local includes
#include "ModuleManager.hh"
#include "Hardware.hh"

// Module specific constants ---------------------------------------

#define MAX_RC_STICKS 5

// The RemoteControl class -----------------------------------------
class RemoteControl : public Module {

public:

  RemoteControl( char *name );

  // possible directions of a joystick
  typedef enum { CENTER, NORTH, SOUTH, EAST, WEST, 
                 NORTHWEST, NORTHEAST, SOUTHWEST, SOUTHEAST } StickDir;

  void setThreshold ( float x ) { threshold = x; }
  void configure ( double right_delay, double left_delay );

  StickDir stickVal ( uint stick );
  StickDir leftStick ( void ) { return stickVal ( 0 ); }
  StickDir rightStick ( void ) { return stickVal ( 1 ); }

  // soon to be obsolete, don't use
  void configureStick( uint stick, uint steps, double delay,
					   uint xdial, uint ydial );

  float readStickX( uint stick ) { return stickCurXVal[ stick ]; };
  float readStickY( uint stick ) { return stickCurYVal[ stick ]; };
  void readStick( uint stick, float *xval, float *yval );
  void resetStick( uint stick );

  // Module base class interface
  void  init ( void ) { hardware = MMGetHardware(); };
  void  uninit ( void ) { };
  void  activate ( void );
  void  deactivate ( void ) {  };
  void  update ( void );

private:

  Hardware *hardware;

  // Configuration of the sticks
  bool enableStick[ MAX_RC_STICKS ];
  uint stickSteps[ MAX_RC_STICKS ];
  double stickDelay[ MAX_RC_STICKS ];
  uint stickXDial[ MAX_RC_STICKS ];
  uint stickYDial[ MAX_RC_STICKS ];

  double threshold;

  // Dynamic data
  float stickCurXVal[ MAX_RC_STICKS ];
  float stickCurYVal[ MAX_RC_STICKS ];
  float stickLastXVal[ MAX_RC_STICKS ];
  float stickLastYVal[ MAX_RC_STICKS ];
  double stickMark[ MAX_RC_STICKS ];

};

#endif

