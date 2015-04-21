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
 * $Id: McAnalog.cc,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Low level hardware interface to the Analog IO facilities of the Michigan
 * version of RHex
 *
 * Created       : Uluc Saranli, 12/20/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <math.h>
#include "McComponents.hh"
#include "McAnalogProbe.hh"

// McAnalog::McAnalog : Class constructor
McAnalog::McAnalog( void ) {
  int channel;
  bool config_polling;
  int config_channels;
  char msg[128];

  MMMessage( "Initializing Analog IO components..." );

  // Reset all the analog outputs to 0V
  for ( channel = 0; channel < 8; channel++ ) {
    MPC550Card->analogOut( channel, 0x0900 );
  }

  config_channels = int( MMGetFloatSymbol( "analog_channels", ADC_CHANNELS ) );
  config_polling = bool( int ( MMGetFloatSymbol( "analog_polling", 
                                                 ADC_POLLING ) ) );

  if ( config_polling )
    sprintf( msg, "ADC:poll[%ich].", config_channels );
  else 
    sprintf( msg, "adc:normal[%ich].", config_channels );
  MMMessage( msg );

  // Create, Add and activate the AnalogProbe module
  probe = new McAnalogProbe( config_channels, config_polling );
  MMAddModule ( probe, ADC_MODULE_PERIOD, 
                ADC_MODULE_OFFSET, ADC_MODULE_ORDER );
  MMActivateModule ( probe );

  outputCoeff = 1.0 / ( AOUT_MAX - AOUT_MIN ) * ( (1 << DAC_BITS) - 1 );

  MMMessage( "done.\n" );
}

// McAnalog::~McAnalog : Class destructor
McAnalog::~McAnalog( ) {
  int channel;

  // Remove the AnalogProbe module
  MMRemoveModule ( probe );
  delete probe;

  // Reset all the analog outputs to 0V
  for ( channel = 0; channel < 8; channel++ ) {
    MPC550Card->analogOut( channel, 0x0000 );
  }
}






