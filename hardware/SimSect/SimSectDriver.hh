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
 * $Id: SimSectDriver.hh,v 1.3 2001/07/13 19:49:56 ulucs Exp $
 *
 * The SimSect engine driver
 *
 * This file contains the definition of the SimSectDriver class and
 * all the associated definitions. The member function definitions are
 * in SimSectDriver.cc
 *
 * This class is responsible from initializing and driving the RHexLib
 * interface to SimSect.
 *
 * Created       : Uluc Saranli, 03/11/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _SIMSECTDRIVER_HH
#define _SIMSECTDRIVER_HH


// System includes
#include <stdio.h>

// Local includes
#include "ModuleManager.hh"
#include "Hardware.hh"
#include "SS_Hexapod.hh"
#include "SS_HexVoltage.hh"

class SS_Integrator;

// The SimSectDriver class -----------------------------------------
class SimSectDriver : public Module {

private:

  // Internal bookkeeping --------------------------------------------

  // Body states
  double    acceleration[3];  // The acceleration vector in the body frame.
  double    angularvel[3];    // The angular velocity vector in the body frame.

  // Position and speed of motor shaft (rad, rad/s)
  double    pos[6], speed[6];

  double    basepos[6];  // Origin position for encoder count computation (rad)
  uint16    encCount[6]; // The encoder count
  double    command[6];  // Command voltage at motor terminals
  bool      encoderEnabled[6]; // Flag indicating encoder status

  // SimSect interface related data ----------------------------------
  SS_Hexapod    *hexapod;
  SS_Integrator *engine;
  SS_HexVoltage *controller;

  FILE          *logFile;    
  double        time;
  double        lastHipAngle[6];

  float         backEMFcoeff;  // precomputed coefficient for the back EMF

public:
  SimSectDriver( void );
  ~SimSectDriver( );

  // Module base class interface
  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void );
  void  update ( void );

  // Control methods
  void      enableEncoder( uint index, bool enable ) { encoderEnabled[index] = enable; };
  void      resetEncoder( uint index ) { encCount[index] = 0; basepos[index] = 0; };
  void      setCommand( uint index, double cmd );

  // Data access methods
  uint16    readEncoder( uint index ) { return encCount[ index ]; };
  double    getCommand( uint index) { return command[ index ]; };
  double    realHipAngle( uint index ) { return hexapod->getHipAngle( index ); };
  double    realHipSpeed( uint index ) { return hexapod->getHipSpeed( index ); };
  double    motorVoltage( uint index ) { return controller->getVoltage( index ); };
  double    motorCurrent( uint index ) { return controller->getCurrent( index ); };
  double    motorBackEMF( uint index ) { return controller->getBackEMF( index ); };

  void      bodyAcceleration( double *accel ) {
    for ( int i = 0; i < 3; i++ ) accel[ i ] = acceleration[i];
  };
  void      bodyRotation( double *R ) {
    double *bodyR = hexapod->getBodyRotation();
    for ( int i = 0; i < 9; i++ ) R[ i ] = bodyR[i];
  }
  void      bodyAngularVel( double *w ) {
    for ( int i = 0; i < 3; i++ ) w[ i ] = angularvel[i];
  };
  double    *hexapodState( void )  { return hexapod->getState(); };
  double    hexapodStateVar ( int index ) {
    double * x = hexapodState( );
    return x[ index ];
  };
  double    *hexapodParams( void ) { return hexapod->getParams(); };
  int       hexapodChart( void )   { return hexapod->getChart(); };
  bool      legChart ( uint leg_no ) {
    return ( ( hexapodChart() >> leg_no ) & 0x01 ) ? true : false ;
  };

};

#endif // _SIMSECTDRIVER_HH
