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
 * $Id: Tokenizer.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * This file all the datatypes and function prototypes for
 * the Tokenizer class. The functions declared in this
 * file are defined in Tokenizer.c. 
 * The Tokenizer class is used to parse an input stream
 * into a stream of tokens, which are then interpreted by
 * a higher level parser.
 *
 * Created       : Uluc Saranli, 12/17/1998
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _TOKENIZER_HH
#define _TOKENIZER_HH

// Standard includes
#include <stdio.h>

// Constants -------------------------------------------------------

#define MAX_TOKEN_LENGTH        128

// Input stream types
#define TOKEN_INPUT_UNKNOWN 0
#define TOKEN_INPUT_FILE    1
#define TOKEN_INPUT_STRING  2

// Token type definitions
#define TOKEN_INVALID   0
#define TOKEN_EOF       1
#define TOKEN_INTEGER   2
#define TOKEN_FLOAT     3
#define TOKEN_STRING    4
#define TOKEN_SYMBOL    5
#define TOKEN_EQUAL     6   // =
#define TOKEN_SEMICOLON 7   // ;
#define TOKEN_POUND     8   // #
#define TOKEN_LBRACKET  9   // {
#define TOKEN_RBRACKET  10  // }

#define NO_ERROR 0
#define TOKENIZER_ERROR -1

// Data types -----------------------------------------------------

typedef struct {
  int   type;
  
  int   ivalue;
  float fvalue;
  char  svalue[MAX_TOKEN_LENGTH];
} Token;

// The Tokenizer class ---------------------------------------------
class Tokenizer {

 public:
  Tokenizer( void );
  ~Tokenizer( );

  int open( int type, const char *file_or_str );
  int get( Token *token );
  void close( void );
  int eoi( void );
  
 private:
  int        inputType;

  FILE*      inputFile;
  const char *inputStr;
  int        strPtr;

  char       lastChar;

  int end_of_input( void );
  int get_next_char( void );
  int skip_whitespace( void );
  int get_number_token( Token *token );
  int get_symbol_token( Token *token );
  int get_string_token( Token *token );
  void skip_line( void );

};

#define EXPECT_TOKEN( T_IN, TYPE_IN, TOKEN_IN )           \
  ( T_IN ).get( ( TOKEN_IN ) );                           \
  if ( ( TOKEN_IN )->type != ( TYPE_IN ) ) {              \
    MMWarning( "EXPECT_TOKEN macro",                      \
               "Parse error, invalid token. Aborting!" ); \
    return false;                                         \
  }

#define GET_TOKEN( T_IN, TOKEN_IN ) \
  ( T_IN ).get( ( TOKEN_IN ) );

#endif
