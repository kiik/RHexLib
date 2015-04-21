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
 * $Id: VirtualInput.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * Created       : Uluc Saranli, 01/18/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include "VirtualInput.hh"

// VirtualInput:initialize : Read the script file and initialize the structs
void VirtualInput::initialize( char *path ) {

  FILE *fp;
  int   i;
  
  firstEntry = NULL;

  if ( ( fp = fopen( path, "r" ) ) != NULL ) {

    int ch, dur;
    float val;
    SCRIPT_ENTRY * temp;

    fscanf ( fp, "%d %f %d", &ch, &val, &dur );

    while ( !feof ( fp ) ) {

      if ( firstEntry == NULL ) {
        firstEntry = new SCRIPT_ENTRY;
        temp = firstEntry;
      } else {
        temp->next = new SCRIPT_ENTRY;
        temp = temp->next;
      }

      temp->channel = ch;
      temp->value = val;
      temp->duration = dur * 1000;
      temp->next = NULL;

      fscanf ( fp, "%d %f %d", &ch, &val, &dur );
    }

    fclose ( fp );

  } else {
    char msg[256];

    sprintf( msg, "Cannot open script file \"%s\"!", path );
    MMWarning( "VirtualInput::initialize", msg );
  }

  for ( i = 0; i < MAX_VIRTUAL_CHANNELS; i++ ) {
    curEntry[i] = NULL;
    channel[i] = 0.0;
    mark[i] = 0;
  }


}

// VirtualInput::printScript : Prints the script for debugging purposes
void VirtualInput::printScript ( void ) {

  SCRIPT_ENTRY * temp = firstEntry;
  char msg[256];
  
  sprintf( msg, "Script contents for module \"%s\[%i]\":\n", getName(), getIndex() );
  MMMessage( msg );
  while ( temp != NULL ) {

    sprintf ( msg, "  * channel: %d\tvalue: %.2f\tduration: %fus\n",
              temp->channel, temp->value, CLOCK_TO_DOUBLE(temp->duration) );
    MMMessage( msg );
    temp = temp->next;

  }
      
}

// VirtualInput::init : Module initialization
void VirtualInput::init ( void ) {

  int i;
  SCRIPT_ENTRY *temp;

  for ( i = 0; i < MAX_VIRTUAL_CHANNELS; i++ ) {
    temp = firstEntry;

    while ( temp != NULL && temp->channel != i )
      temp = temp->next;

    if ( temp != NULL ) {
      curEntry[i] = temp;
      channel[i] = temp->value;
      mark[i] = 0;
    }
  }
}

// VirtualInput::update : Module update
void VirtualInput::update ( void ) {

  int i;
  CLOCK now = MMReadClock();

  for ( i = 0; i < MAX_VIRTUAL_CHANNELS; i++ ) {
    SCRIPT_ENTRY * temp = curEntry[i];
    // Scan all channels

    if ( temp != NULL ) {
      
      if ( now >= mark[i] + temp->duration ) {
        // The duration has expired for channel i. Find next entry
        temp = temp->next;

        while ( temp != NULL && temp->channel != i )
          temp = temp->next;

        curEntry[i] = temp;

        if ( temp != NULL ) {

          channel[i] = temp->value;

          mark[i] = now;

        } else channel[i] = 0.0;

        VI_DEBUG_MSG( i, channel[i] );
      }

    } // if ( temp != NULL )
  }
}
