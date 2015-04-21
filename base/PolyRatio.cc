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
 * $Id: PolyRatio.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * Ratios of Polynomials representation class 
 *
 * Created       : Haldun Komsuoglu, 01/21/2001
 * Last Modified : Haldun Komsuoglu, 06/27/2001
 *
 ********************************************************************/

#include "PolyRatio.hh"
#include <stdio.h>

/*----------------------------------------------------------------------*/
/*+++++++++++++++++++++ Ratio of Polynomial Class ++++++++++++++++++++++*/
/*----------------------------------------------------------------------*/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* PolyRatio::operator<<=( PRsub ) : Substitutes the polynomial, Psub,
                                     instead of the independent variable
                     of "this" polynomial generating a
                     new polynomial, P(Psub(x)). Result is
                     stored in "this".

      Psub   : (Polynomial&) The polynomial to be substituted into
               "this" polynomial. Access via reference

      return : (Polynomial&) Resulting polynomial, 
               Pres(x) = P(Psub(x)), Accessed via reference.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
PolyRatio& PolyRatio::operator<<=( const PolyRatio& PRsub ){

#define   MAXORDER    30

  int          i;
  int          N = Num.Order();
  int          M = Den.Order();
  int   maxOrder = 0;
  
  if (N > M)
    maxOrder = N;
  else
    maxOrder = M;


  Polynomial   Ns[MAXORDER+1];
  Polynomial   Ds[MAXORDER+1];
  Polynomial   TempPoly;
  PolyRatio    PRres;

  // Prepare the numerator powers
  Ns[0] = 1;
  Ns[0].setPowSign( PRsub.Num.getPowSign() );
  for (i = 1 ; i <= maxOrder ; i++){
    Ns[i]  = PRsub.Num;
    Ns[i] *= Ns[i-1];
  }

  // Prepare the denominator powers
  Ds[0] = 1;
  Ds[0].setPowSign( PRsub.Den.getPowSign() );
  for (i = 1 ; i <= maxOrder ; i++){
    Ds[i]  = PRsub.Den;
    Ds[i] *= Ds[i-1];
  }

  // Compute the numerator of the result
  PRres.Num = 0;
  for (i = 0 ; i <= N ; i++){
    TempPoly   = Polynomial(Num.getEntry(i));
    TempPoly  *= Ns[i];
    TempPoly  *= Ds[N-i];
    PRres.Num += TempPoly;
  }
  if (getPowSign()*M > getPowSign()*N)
    PRres.Num *= Ds[M-N];

  // Compute the denominator of the result
  PRres.Den = 0;
  for (i = 0 ; i <= M ; i++){
    TempPoly   = Polynomial(Den.getEntry(i));
    TempPoly  *= Ns[i];
    TempPoly  *= Ds[M-i];
    PRres.Den += TempPoly;
  }
  if (getPowSign()*N > getPowSign()*M)
    PRres.Den *= Ds[N-M];

  Num = PRres.Num;
  Den = PRres.Den;

  return *this;

}
/*----------------------------------------------------------------------*/
