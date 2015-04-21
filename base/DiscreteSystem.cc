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
 * $Id: DiscreteSystem.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * DiscreteSystem class
 *
 * Created       : Haldun Komsuoglu, 01/18/2001
 * Last Modified : Haldun Komsuoglu, 06/27/2001
 *
 ********************************************************************/

#include "DiscreteSystem.hh"
#include "Buffer.hh"
#include <stdio.h>

//-----------------------------------------------------------------------
//++++++++++++++++++++++++ DiscreteSystem Class +++++++++++++++++++++++++
//-----------------------------------------------------------------------

/* Implements the following difference equation describing a discrete 
   system:

              y[0] = -b[1]y[-1] - b[2]y[-2] - ... - b[M]y[-M]
                     +a[0]x[0]  + a[1]x[-1] + ... + a[N]x[-N]
*/


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* DiscreteSystem::DiscreteSystem( ) : Default discrete system constructor
                                       Designs a unity gain amplifier sys.
                                       With no subsampling.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
DiscreteSystem::DiscreteSystem( void ){
  
  double p[1] = {1};
  
  BuildStructure( 1, 1);
  SetParam( INPUT_COEFF, p, 1);
  SetParam( OUTPUT_COEFF, p, 1);
}
/*----------------------------------------------------------------------*/





/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* DiscreteSystem::BuildStructure( N, M ) : Creates a discrete system 
                                            object with specified input 
                                            output buffer lengths, and 
                                            resets the buffers and coeffs
                                            to zero. Subsampling is
                                            disabled...

      N   : (int) The buffer/coeff list length for the input
      M   : (int) The buffer/coeff list length for the output

    Note: The input and output buffer lengths has to be equal or larger
          than 1. (N>1,M=1) is FIR, (N=1,M=1) is scaling, (M>=N>1) is IIR

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DiscreteSystem::BuildStructure( int N,
                                     int M ){

  // Generate the input related buffers and coefficient list
  if (N < 1) {
    printf( "Error: (DiscreteSystem.cc) non-positive input buffer size..." );
    N = 1;
  }     
  x.Redefine( N );
  a.Redefine( N );

  // Generate the output related buffers and coefficent list
  if (M < 1) {
    printf( "Error: (DiscreteSystem.cc) non-positive input buffer size..." );
    M = 1;
  }     
  y.Redefine( M );
  b.Redefine( M );

  // Reset the subsampling to NO subsampling, i.e., 1:1
  SubSampleRate = 1;
  SubSampleCounter = SubSampleRate;
}
/*----------------------------------------------------------------------*/




/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* DiscreteSystem::SetParam( btype, p[], K ) : Loads the list of values
                                               in p[] into the picked
                                               parameter chosen by "btype"
                                               Check below list for the
                                               assignment of the elements,
                                               {p0, p1, ... ,pK}

      btype   : (DSParamType) Indicator selecting the buffer to be loaded
                          INPUT_BUFFER    : Input sample buffer
                                              x_i = p_i
                          INPUT_COEFF     : FIR component coefficients 
                                            (multiplying x)
                                              a_i = p_i
                          OUTPUT_BUFFER   : Output sample buffer
                                              y_i = p_i
                          OUTPUT_COEFF    : IIR component coefficients 
                                            (multiplying y)
                                              b_i = p_i
                          SUBSAMPLE_RATIO : Subsample related parameters
                                              SubsampleRatio = p[0]
                                              SubsampleCounter = p[1]
                          
      p[]     : (double[]) The list of values to be assigned to the chosen
                buffer
      K       : Length of the given list of values.

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DiscreteSystem::SetParam( DSParamType btype,
                               double p[],
                               int K){

  // Fill the selected buffer with the given list of values

  
  switch (btype) {
  case INPUT_BUFFER:       // Input sample buffer
    x.Fill(p, K);
    return;
  case INPUT_COEFF:        // Input sample coefficient list
    a.Fill(p, K);
    return;
  case OUTPUT_BUFFER:      // Output sample buffer
    y.Fill(p, K);
    return;
  case OUTPUT_COEFF:       // Output sample coefficient list
    p[0] = 0;  // The current value of output is not used in 
               // difference equ., hence, this value is set to 
               // zero for precaution
    b.Fill(p, K);
        return;
  case SUBSAMPLE_RATIO:    // Subsampling Ratio
    SubSampleRate = int(p[0]);
    SubSampleCounter = int(p[1]);
    break;
  }
}
/*----------------------------------------------------------------------*/





/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* DiscreteSystem::Process( x_new ) : Gets a new input sample, computes 
                                      the difference equation, and 
                                      updates the relavent buffers

      x_new   : (double) The new input sample

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DiscreteSystem::Process( double x_new ){

  double ax , by;

  // Check the sub-sampling counter and process the given data accordingly
  SubSampleCounter--;
  if (SubSampleCounter==0){
    // Reset the subsampling counter
    SubSampleCounter = SubSampleRate;

    // Add the new input sample to the input buffer
    x.push( x_new );

    // Compute the difference equation
    ax = MultiplySum( a, x, 1 );
    by = MultiplySum( b, y, 0 );
    y.push( ax - by );


  }
}
/*----------------------------------------------------------------------*/



