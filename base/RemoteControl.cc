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
 * $Id: RemoteControl.cc,v 1.6 2001/07/20 02:00:31 ulucs Exp $
 *
 * The Remote Controller interface module
 *
 * This file implements the member functions defined in RemoteControl.hh
 *
 * Created       : Uluc Saranli, 01/25/2001
 * Last Modified : Eric Klavins, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include <math.h>
#include "RemoteControl.hh"

RemoteControl::RemoteControl( char *name )
  : Module( name, 0, false, false ), threshold ( 0.5 ) {

  int i;

  for ( i = 0; i < MAX_RC_STICKS; i++ )
    enableStick[ i ] = false;

}

void RemoteControl::configureStick( uint stick, uint steps, double delay, 
                                    uint xdial, uint ydial ) {
  if ( stick >= MAX_RC_STICKS ) {
    MMWarning( "RemoteControl::configureStick", "Invalid stick id!" );
    return;
  }
  enableStick[ stick ] = true;

  stickSteps[ stick ] = steps;
  stickDelay[ stick ] = delay;
  stickXDial[ stick ] = xdial;
  stickYDial[ stick ] = ydial;

  stickCurXVal[ stick ] = 0;
  stickCurYVal[ stick ] = 0;
  stickLastXVal[ stick ] = 0;
  stickLastYVal[ stick ] = 0;
  stickMark[ stick ] = MMReadTime() + stickDelay[ stick ];
}

void RemoteControl::readStick( uint stick, float *xval, float *yval ) {
  *xval = stickCurXVal[ stick ];
  *yval = stickCurYVal[ stick ];
}

void RemoteControl::resetStick( uint stick ) {
  stickCurXVal[ stick ] = 0;
  stickCurYVal[ stick ] = 0;
  stickLastXVal[ stick ] = 0;
  stickLastYVal[ stick ] = 0;
  stickMark[ stick ] = MMReadTime() + stickDelay[ stick ];
}

void  RemoteControl::activate ( void ) {  
  int i;
  for ( i = 0; i < MAX_RC_STICKS; i++ )
    if ( enableStick[i] )
      resetStick( i );
};

void  RemoteControl::update ( void ) {
  int i;
  float xval, yval;

  for ( i = 0; i < MAX_RC_STICKS; i++ ) {

    if ( enableStick[i] ) {

      // Read the dials from the hardware
      xval = ( hardware->dials->read( stickXDial[ i ] ) );
      yval = ( hardware->dials->read( stickYDial[ i ] ) );

      xval = ( xval > 0.5 ) ? 1.0 : ( ( xval < -0.5 ) ? -1.0 : 0.0 );
      yval = ( yval > 0.5 ) ? 1.0 : ( ( yval < -0.5 ) ? -1.0 : 0.0 );

      // If the stick value has changed since we last read it, 
      // reset the timeout and restart
      if ( xval != stickLastXVal[ i ]
           || yval != stickLastYVal[ i ]) {
        //        stickCurXVal[ i ] = 0;
        //        stickCurYVal[ i ] = 0;
        stickMark[ i ] = MMReadTime() + stickDelay[ i ];
      } else if ( MMReadTime() > stickMark[ i ] ) {
        // Done with the timeout. We can set the value
        stickCurXVal[ i ] = stickLastXVal[ i ];
        stickCurYVal[ i ] = stickLastYVal[ i ];
      }
      stickLastXVal[ i ] = xval;
      stickLastYVal[ i ] = yval;
    }
  }
}

void RemoteControl::configure ( double right_delay, double left_delay ) {

  configureStick ( 1, 1, left_delay, 1, 0 );
  configureStick ( 0, 1, right_delay, 3, 2 );

}

RemoteControl::StickDir RemoteControl::stickVal ( uint stick ) {

  float x = readStickX( stick ),
        y = readStickY( stick );

  if ( x < -threshold ) { // West

    if ( y < -threshold ) return SOUTHWEST;
    else if ( y > threshold ) return NORTHWEST;
    else return WEST;

  } else if ( x > threshold ) { // East

    if ( y < -threshold ) return SOUTHEAST;
    else if ( y > threshold ) return NORTHEAST;
    else return EAST;

  } else { // Centered

    if ( y < -threshold ) return SOUTH;
    else if ( y > threshold ) return NORTH;
    else return CENTER;

  }

}
