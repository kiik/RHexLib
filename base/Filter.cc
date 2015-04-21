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
 * $Id: Filter.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * DiscreteSystem class
 *
 * Created       : Haldun Komsuoglu, 01/22/2001
 * Last Modified : Haldun Komsuoglu, 06/27/2001
 *
 ********************************************************************/

#include "Filter.hh"
#include "PolyRatio.hh"
#include "DiscreteSystem.hh"
#include <stdio.h>
#include <math.h>

#define   MAX_NO_OF_COEFF    400

/*-----------------------------------------------------------------------*/
/*++++++++++++++++++++++++++++ Filter Class +++++++++++++++++++++++++++++*/
/*-----------------------------------------------------------------------*/

/* Designs a digital filter and implements it using a difference equation 
   which takes the form:

              y[0] = -b[1]y[-1] - b[2]y[-2] - ... - b[M]y[-M]
                 +a[0]x[0]  + a[1]x[-1] + ... + a[N]x[-N]

   Filter is implemented as a cascade 2nd order sub filters which
   are implemented in Direct Form I

*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::Filter( ... ) : Filter module constructors. Also designs a 
   default unity gain filter.

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
Filter::Filter( void ) 
    : Module( "filter", 0, true, false ) {

  // Design a ddefault unity gain amplifier filter 
  FilterHead = NULL; 
  FilterLast = FilterHead;

  // Set the default sampling frequency
  Ts = 1E-3;         

}

Filter::Filter( char *name ) 
  : Module( name, 0, true, false ) {

  // Design a ddefault unity gain amplifier filter 
  FilterHead = NULL; 
  FilterLast = FilterHead;

  // Set the default sampling frequency
  Ts = 1E-3;         

}

Filter::Filter( char *name, int index ) 
  : Module( name, index, true, false ) {

  // Design a ddefault unity gain amplifier filter 
  FilterHead = NULL; 
  FilterLast = FilterHead;

  // Set the default sampling frequency
  Ts = 1E-3;         

}

/*----------------------------------------------------------------------*/


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::AppendSubFilter( SF ) : Appends the given sub filter discrete 
                                   system to the end of the linked list 
                                   of sub filters.

     SF     : (DiscreteSystem&) Discrete system representation of the
              sub-filter. Accessed via referencing

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Filter::AppendSubFilter( DiscreteSystem *SF) {

  DiscreteSystemLinkedList  *FilterNew;

  // Create a new filter object and copy the given discrete system structure
  // to it.
  FilterNew = new DiscreteSystemLinkedList; 
  FilterNew->next = NULL;
  FilterNew->sys = SF;
  
  // Append this new object to the end of the linked list
  if (FilterLast == NULL) {
    FilterLast = FilterNew;
    FilterHead = FilterNew;
  }  
  else {
    FilterLast->next = FilterNew;
    FilterLast = FilterNew;
  }

}
/*----------------------------------------------------------------------*/




/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::ClearFilterList( ) : Clears the entire list and equates the
                                pointers to NULL.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Filter::ClearFilterList( void ) {

  DiscreteSystemLinkedList  *FilterDummy;

  while (FilterHead != NULL) {
    FilterDummy = FilterHead->next;
    delete FilterHead;
    FilterHead = FilterDummy;
  }

  FilterHead = NULL;
  FilterLast = NULL;

}
/*----------------------------------------------------------------------*/


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::BilinearTransformation( subFpntr ) : Applys bilinear 
                                                transformation to the 
                                                given ratio of polynomial.

     subFpntr : (PolyRatio&) The sub filter transfer function in Laplace 
                domain stored in a ratio of polynomial.

     return   : (int) Error code
                      0  : No error

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
int Filter::BilinearTransformation( PolyRatio *subFpntr ) {

  double      aBLT[] = {2/Ts, -2/Ts};
  double      bBLT[] = {1, +1};
  PolyRatio   BLT(aBLT, 1, bBLT, 1, -1);

  *subFpntr <<= BLT;

  return 0;
}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::AdjustGain( gain ) : Scales the coefficients so that their sum
                                is equal to gain. Used to set the DC gain 
                                of a FIR filter represented by the coef
                                array, a.

     a    : (double[]) The array where the resulting coefficients to be
            stored
     M    : (int) Size of the given array
     gain : (double) Desired gain

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Filter::AdjustGain( double a[], 
             int    M,
             double gain) {

  double DC = 0;
  int    n;

  // Compute the current sum
  for (n = 0 ; n < M ; n++) {
    DC = DC + a[n];
  }

  // Scale the coefficients so that pass band gain is "gain"
  for (n = 0 ; n < M ; n++) {
    a[n] = a[n] / DC * gain;
  }
}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::SincFunc( a, wc, M ) : Generates and stores a sequence of 
                                  numbers into a[] which are computed
                                  from the Sinc function w/ 
                                  parameters wc and M.

     a    : (double[]) The array where the resulting coefficients to be
            stored
     M    : (int) Order of the filter (# of tabs)
     wc   : (double) Normalized cut of frequency (Fc/Fs*2*pi)


++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Filter::SincFunc( double a[], 
               int M,
               double wc) {
  int    n;
  double C;
  int    oddM = M%2;

  C = ( ( (double)(M) - 1) / 2 );

  // Compute the basic SINC function with the specified wc and M
  for (n = 0 ; n < M ; n++) {
    if ((oddM > 0) && (n == C))
      a[n] = wc/M_PI;
    else
      a[n] = sin( wc*(n - C) ) / ( M_PI*(n - C) );
  }

}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::Design_RC( p ) : Discrete RC filter design utility.

     p     : (FilterParametersType) Filter parameters structure
             Extract the following coefficients:
                  p.Gp = gain     : (double) Pass band gain
                  p.Fc = Fc       : (double) Time constant for 
                                    the filter
                            
   return : (int) Error code
             0  : No error
             -1 : Unrealistic (non-positive) time constant 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
int Filter::Design_RC( FilterParam_t* p) {

  double gain = p->Gp;
  double Fs   = p->Fs;
  double Fc   = p->Fc;
  double tau  = 1/p->Fc;
  
  DiscreteSystem  *NewSubFilter;

  // Error control
  if (Fc>=Fs/2) return -1;

  // Build the transfer function in the Laplace domain
  double NumCoef[1] = {gain/tau};
  double DenCoef[2] = {1/tau, 1};

  // Convert it into the z-domain
  PolyRatio FilterTF( NumCoef, 0, DenCoef, 1, 1 );
  BilinearTransformation( &FilterTF );
  FilterTF.Unity_b0();

  // Fill the coefficient buffers of the new discrete system
  // with the values obtained from the z-domain transfer function
  NewSubFilter = new DiscreteSystem;
  NewSubFilter->BuildStructure( 2, 2 );
  NewSubFilter->SetParam( INPUT_COEFF, FilterTF.NumCoeffListPntr(), 2 );
  NewSubFilter->SetParam( OUTPUT_COEFF, FilterTF.DenCoeffListPntr(), 2 );

  ClearFilterList();
  AppendSubFilter( NewSubFilter );

  return 0;
}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::Design_BUTTERWORTH( p ) : Discrete BUTTERWORTH filter design 
                                     utility.

     p     : (FilterParam_t) Filter parameters structure
             Extract the following coefficients:
                  p.Gp = gain        : (double) Pass band gain
                  p.Fc = Fc          : (double) Time constant for 
                                                the filter
                  p.parameter[0] = N : (int) Number of poles (must be even)

     return : (int) Error code
              0  : No error
              -1 : Unrealistic (non-positive) time constant 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
int Filter::Design_BUTTERWORTH( FilterParam_t* p) {

  double gain_per_subfilt = pow( p->Gp, 1/3 );
  double Fs   = p->Fs;
  double Fc   = p->Fc;
  int    N    = int(p->parameter[0]);

  int    i;

  double Omega_c_val = Omega_c( Fc ); 
  double Omega_c_val_2 = Omega_c_val * Omega_c_val;
  double NumCoef[1];
  double DenCoef[3];

  DiscreteSystem  *NewSubFilter;

  // Error control
  if (Fc>=Fs/2) return -1;

  // Make sure that the filter order is even
  if (N%2 == 1) N++;
  
  ClearFilterList();

  // Design sub filters of second order and append them to the
  // sub filter linked list
  for (i = 1 ; i < N ; i = i+2 ) {
    DenCoef[0] = Omega_c_val_2;
    DenCoef[1] = -2 * Omega_c_val * cos( M_PI * i / (2*N) + M_PI/2 );
    DenCoef[2] = 1;
    NumCoef[0] = gain_per_subfilt * Omega_c_val_2;
    PolyRatio FilterTF( NumCoef, 0, DenCoef, 2, 1 );
    BilinearTransformation( &FilterTF );
    FilterTF.Unity_b0();

    // Fill the coefficient buffers of the new discrete system
    // with the values obtained from the z-domain transfer function
    NewSubFilter = new DiscreteSystem;
    NewSubFilter->BuildStructure( 3, 3 );
    NewSubFilter->SetParam( INPUT_COEFF, FilterTF.NumCoeffListPntr(), 3 );
    NewSubFilter->SetParam( OUTPUT_COEFF, FilterTF.DenCoeffListPntr(), 3 );
    AppendSubFilter( NewSubFilter );
  }

  return 0;
}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::Design_BOXCAR( p ) : Discrete BOXCAR FIR filter design utility.

     p     : (FilterParam_t) Filter parameters structure
             Extract the following coefficients:
                  p.Gp = gain         : (double) Pass band gain
                  p.Fc = Fc           : (double) Low pass filter cut-off
                                                 frequency (in Hz)
                  p.parameter[0] = M  : Order (# tabs) of the FIR filter

     return : (int) Error code
              0  : No error
              -1 : Unrealistic bandwidth 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
int Filter::Design_BOXCAR( FilterParam_t* p) {

  double gain = p->Gp;
  double Fc   = p->Fc;
  double Fs   = p->Fs;
  double Omega_c_val = Omega_c( Fc ); 
  int    M    = int(p->parameter[0]);

  double a[MAX_NO_OF_COEFF];
  double b[1];
  double wc = Fc/Fs*2*M_PI;

  DiscreteSystem  *NewSubFilter;

  // Error control
  if (Fc>=Fs/2) return -1;

  // If the filter order is not specified compute it from the desired
  // cut off frequency
  if (M == 0)
    M = int(1/Omega_c_val*M_PI*Fs);

  // Compute the coefficients to be assigned to the discrete system
  SincFunc( a, M, wc );
  AdjustGain( a, M, gain );
  b[0] =  1;

  // Fill the coefficient buffers of the new discrete system
  // with the values obtained from the z-domain transfer function
  NewSubFilter = new DiscreteSystem;
  NewSubFilter->BuildStructure( M, 1 );
  NewSubFilter->SetParam( INPUT_COEFF, a, M );
  NewSubFilter->SetParam( OUTPUT_COEFF, b, 1 );

  AppendSubFilter( NewSubFilter );

  return 0;
}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::Design_HAMMING( p ) : Discrete HAMMING FIR filter design 
                                 utility.

     p     : (FilterParam_t) Filter parameters structure
             Extract the following coefficients:
                  p.Gp = gain         : (double) Pass band gain
                  p.Fc = Fc           : (double) Low pass filter cut-off
                                                 frequency (in Hz)
                  p.parameter[0] = M  : Order (# tabs) of the FIR filter

     return : (int) Error code
                      0  : No error
                      -1 : Unrealistic bandwidth 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
int Filter::Design_HAMMING( FilterParam_t* p) {

  double gain = p->Gp;
  double Fc   = p->Fc;
  double Fs   = p->Fs;
  int    M    = int(p->parameter[0]);
  double Omega_c_val = Omega_c( Fc ); 

  double a[MAX_NO_OF_COEFF];
  double b[1];
  double wc = Fc/Fs*2*M_PI;

  int    n;

  DiscreteSystem  *NewSubFilter;

  // Error control
  if (Fc>=Fs/2) return -1;

  // If the filter order is not specified compute it from the desired
  // cut off frequency
  if (M == 0)
    M = int(1/Omega_c_val*2*M_PI*Fs);

  // Compute the coefficients to be assigned to the discrete system
  SincFunc( a, M, wc );
  for (n = 0 ; n < M ; n++) {  // Hanning window multiplication
    a[n] = a[n] * (0.54 - 0.46*cos( 2*M_PI*n / (M-1)));
  }
  AdjustGain( a, M, gain );
  b[0] =  1;
  
  // Fill the coefficient buffers of the new discrete system
  // with the values obtained from the z-domain transfer function
  NewSubFilter = new DiscreteSystem;
  NewSubFilter->BuildStructure( M, 1 );
  NewSubFilter->SetParam( INPUT_COEFF, a, M );
  NewSubFilter->SetParam( OUTPUT_COEFF, b, 1 );
  
  AppendSubFilter( NewSubFilter );


  return 0;
}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::Design_HANNING( p ) : Discrete HANNING FIR filter design 
                                 utility.

     p     : (FilterParam_t) Filter parameters structure
             Extract the following coefficients:
                  p.Gp = gain         : (double) Pass band gain
                  p.Fc = Fc           : (double) Low pass filter cut-off
                                                 frequency (in Hz)
                  p.parameter[0] = M  : Order (# tabs) of the FIR filter

     return : (int) Error code
                      0  : No error
                      -1 : Unrealistic bandwidth 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
int Filter::Design_HANNING( FilterParam_t* p) {

  double gain = p->Gp;
  double Fc   = p->Fc;
  double Fs   = p->Fs;
  int    M    = int(p->parameter[0]);
  double Omega_c_val = Omega_c( Fc ); 

  double a[MAX_NO_OF_COEFF];
  double b[1];
  double wc = Fc/Fs*2*M_PI;

  int    n;

  DiscreteSystem  *NewSubFilter;

  // Error control
  if (Fc>=Fs/2) return -1;

  // If the filter order is not specified compute it from the desired
  // cut off frequency
  if (M == 0)
    M = int(1/Omega_c_val*2*M_PI*Fs);

  // Compute the coefficients to be assigned to the discrete system
  SincFunc(a, M, wc);
  for (n = 0 ; n < M ; n++) {  // Hanning window multiplication
    a[n] = a[n] * 0.5*(1 - cos( 2*M_PI*n / (M-1)));
  }
  AdjustGain( a, M, gain );
  b[0] =  1;
  
  // Fill the coefficient buffers of the new discrete system
  // with the values obtained from the z-domain transfer function
  NewSubFilter = new DiscreteSystem;
  NewSubFilter->BuildStructure( M, 1 );
  NewSubFilter->SetParam( INPUT_COEFF, a, M );
  NewSubFilter->SetParam( OUTPUT_COEFF, b, 1 );
  
  AppendSubFilter( NewSubFilter );

  return 0;
}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::Design_BLACKMAN( p ) : Discrete BLACKMAN FIR filter design 
                                 utility.

     p     : (FilterParam_t) Filter parameters structure
             Extract the following coefficients:
                  p.Gp = gain         : (double) Pass band gain
                  p.Fc = Fc           : (double) Low pass filter cut-off
                                                 frequency (in Hz)
                  p.parameter[0] = M  : Order (# tabs) of the FIR filter

     return : (int) Error code
                      0  : No error
                      -1 : Unrealistic bandwidth 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
int Filter::Design_BLACKMAN( FilterParam_t* p) {

  double gain = p->Gp;
  double Fc   = p->Fc;
  double Fs   = p->Fs;
  int    M    = int(p->parameter[0]);
  double Omega_c_val = Omega_c( Fc ); 

  double a[MAX_NO_OF_COEFF];
  double b[1];
  double wc = Fc/Fs*2*M_PI;

  int    n;

  DiscreteSystem  *NewSubFilter;

  // Error control
  if (Fc>=Fs/2) return -1;

  // If the filter order is not specified compute it from the desired
  // cut off frequency
  if (M == 0)
    M = int(1/Omega_c_val*2*M_PI*Fs);

  // Compute the coefficients to be assigned to the discrete system
  SincFunc( a, M, wc );
  for (n = 0 ; n < M ; n++) {  // Hanning window multiplication
    a[n] = a[n] * (0.42 -  0.5*cos(2*M_PI*n/(M-1)) 
                + 0.08*cos(4*M_PI*n/(M-1))
                  );
  }
  AdjustGain( a, M, gain );
  b[0] =  1;
  
  // Fill the coefficient buffers of the new discrete system
  // with the values obtained from the z-domain transfer function
  NewSubFilter = new DiscreteSystem;
  NewSubFilter->BuildStructure( M, 1 );
  NewSubFilter->SetParam( INPUT_COEFF, a, M );
  NewSubFilter->SetParam( OUTPUT_COEFF, b, 1 );

  AppendSubFilter( NewSubFilter );

  return 0;
}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::Design( Ftype, p ) : Filter design interface.

     Ftype : (FilterType) Filter type indicator
     p     : (FilterParam_t) Filter parameters structure

     return : (int) Error code
                      0  : No error
                      1  : Unknown filter request
                      <0 : Problem with the specific filter design tool

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
int Filter::Design( FilterParam_t* p){

  // Set the generic parameters
  Ts = 1/(p->Fs);     // Sampling period (sec)

  // Design the filter coefficients
  switch (p->Ftype) {
  case RC:            // RC filter (IIR)
    return Design_RC( p );
  case BUTTERWORTH:   // Butterworth filter (IIR)
    return Design_BUTTERWORTH( p );
  case HAMMING:       // Hamming window filter (FIR)
    return Design_HAMMING( p );
  case HANNING:       // Hanning window filter (FIR)
    return Design_HANNING( p );
  case BOXCAR:        // Boxcar window filter (FIR)
    return Design_BOXCAR( p );
  case BLACKMAN:      // Blackman window filter (FIR)
    return Design_BLACKMAN( p );
  default:
    return 1;
  }

  return 1;
}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::UserDefinedFilter( a, b ) : User defined arbitary filter.

     a     : (double[]) The coefficient list for the input buffer
     Na    : (int) The lenght of the input coefficient list
     b     : (double[]) The coefficient list for the output buffer
     Nb    : (int) The lenght of the output coefficient list

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Filter::UserDefinedFilter( double a[], int Na,  double b[], int Nb){

  DiscreteSystem  *NewSubFilter;

  // Create the filter linked list using the user defined filter
  ClearFilterList();

  // Fill the coefficient buffers of the new discrete system
  // with the values obtained from the z-domain transfer function
  NewSubFilter = new DiscreteSystem;
  NewSubFilter->BuildStructure( Na, Nb );
  NewSubFilter->SetParam( INPUT_COEFF, a, Na );
  NewSubFilter->SetParam( OUTPUT_COEFF, b, Nb );
  
  AppendSubFilter( NewSubFilter );

}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::update( void ) : Filter processing update function.

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Filter::update( void ) {

  DiscreteSystemLinkedList  *FilterPntr = FilterHead;
  double                    x_i = source->value();

  // Process each sub-filter and propagate their outputs through
  while (FilterPntr != NULL){
    FilterPntr->sys->Process( x_i );
    x_i = FilterPntr->sys->output();
    FilterPntr = FilterPntr->next;
  }

  // Latch the final output
  y = x_i;

}
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Filter::activate( void ) : Filter processing activate function.

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Filter::activate( void ) {

  DiscreteSystemLinkedList * FilterPntr = FilterHead;

  // Reset the filter buffers
  while (FilterPntr != NULL) {
    FilterPntr->sys->ResetBuffers();
    FilterPntr = FilterPntr->next;
  }

}
/*----------------------------------------------------------------------*/
