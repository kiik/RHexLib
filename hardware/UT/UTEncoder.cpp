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

/** @file UTEncoder.cpp
 *  @brief Low level hardware interface to the Digital IO facilities of the University of Tartu version of RHex.
 *
 *  @author Meelik Kiik
 *  @version 0.1
 *  @date 21/04/2015 16:00
 *
 *  Created at 21/04/2015 16:00
 */

#include <math.h>

#include "UTComponents.h"


// UTEncoder::UTEncoder : Class constructor. Initializes the encoder chips
UTEncoder::UTEncoder( void ) {

  int encoder;

  MMMessage( "Initializing Encoder interface components..." );
  // Disable and reset the encoders
  for (encoder = 0; encoder < 6; encoder++) {
    disable( encoder );
    reset( encoder );
  }
  MMMessage( "done.\n" );
}

// UTEncoder::UTEncoder : Class destructor.
UTEncoder::~UTEncoder( void ) {

  int encoder;

  // Disable and reset the encoders
  for (encoder = 0; encoder < 6; encoder++) {
    disable( encoder );
    reset( encoder );
  }
}

// UTEncoder::enable : Enables the encoder counter for the selected
// channel.  Note that this procedure selects the appropriate card for
// the channel
void UTEncoder::enable( uint index ) {

  if ( index < 3 )
    DM6814Card[0]->enableEncoder( index );
  else if ( index < 6 )
    DM6814Card[1]->enableEncoder( index - 3 );
  else
    MMWarning( "UTEncoder::enable", "Encoder index out of bounds!" );
}

// UTEncoder::disable : Disables the encoder counter for the selected
// channel.  Note that this procedure selects the appropriate card for
// the channel
void UTEncoder::disable( uint index ) {

  if ( index < 3 )
    DM6814Card[0]->disableEncoder( index );
  else if ( index < 6 )
    DM6814Card[1]->disableEncoder( index - 3 );
  else
    MMWarning( "UTEncoder::disable", "Encoder index out of bounds!" );

}

// UTEncoder::reset : Resets the current encoder count for the selected
// channel.  Note that this procedure selects the appropriate card for
// the channel
void UTEncoder::reset( uint index ) {

  if ( index < 3 )
    DM6814Card[0]->clearEncoder( index );
  else if ( index < 6 )
    DM6814Card[1]->clearEncoder( index - 3 );
  else
    MMWarning( "UTEncoder::reset", "Encoder index out of bounds!" );

}



