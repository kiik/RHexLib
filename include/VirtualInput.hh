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
 * $Id: VirtualInput.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * The VirtualInput Module class definitions. This module is used to
 * simulate various user input components such as virtual dials and
 * switches.
 *
 * Created       : Uluc Saranli, 01/18/2001 ( based on Klavins's VirtualRC )
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _VIRTUALINPUT_HH
#define _VIRTUALINPUT_HH

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "ModuleManager.hh"
#include "StateMachine.hh"

#define MAX_VIRTUAL_CHANNELS 16

#define VI_DEBUG_MSG( ch, val ) { char msg[128]; sprintf( msg, "%s[%i]: Virtual channel %i switching to value %f\n", getName(), getIndex(), (ch), (val) ); MMMessage( msg ); }

typedef struct _script_entry {

  int channel;
  float value;
  CLOCK duration;
  struct _script_entry * next;

} SCRIPT_ENTRY;

class VirtualInput : public Module {

public:

  VirtualInput( void ) : Module ( "virtualinput", 0, false, false ) { };
  VirtualInput( char *name ) : Module ( name, 0, false, false ) { };
  VirtualInput( char *name, int index ) : Module ( name, index, false, false ) { };

  void initialize( char *path );

  float getChannelValue ( int i ) { 
    return ( i < MAX_VIRTUAL_CHANNELS ) ? channel[i] : 0.0; 
  };
  void printScript ( void );
  
  void init ( void );
  void uninit ( void ) { };
  void activate ( void ) { };
  void deactivate ( void ) { };
  void update ( void );

private:

  float channel[MAX_VIRTUAL_CHANNELS];
  CLOCK mark[MAX_VIRTUAL_CHANNELS];
  SCRIPT_ENTRY * firstEntry;
  SCRIPT_ENTRY * curEntry[MAX_VIRTUAL_CHANNELS];

};

#endif
