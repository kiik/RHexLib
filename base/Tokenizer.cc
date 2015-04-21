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
 * $Id: Tokenizer.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * This file implements the functions decalred in Tokenizer.h
 *
 * Created       : Uluc Saranli, 12/17/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include <ctype.h>
#include "ModuleManager.hh"
#include "Tokenizer.hh"

// Tokenizer::Tokenizer : Class constructor
Tokenizer::Tokenizer( void ) {

  inputType = TOKEN_INPUT_UNKNOWN;
  inputFile = NULL;
  inputStr = NULL;
  strPtr = 0;
  lastChar = 0;

}

// Tokenizer::~Tokenizer : Class destructor
Tokenizer::~Tokenizer( void ) {
  close(); 
}

// Tokenizer::open : Initializes the tokenizer with the specified input type
int Tokenizer::open( int type, const char *file_or_str ) {

  if (type == TOKEN_INPUT_FILE) {
    // Input type : file

    if ( ( inputFile = fopen( file_or_str, "r") ) == NULL )
      return TOKENIZER_ERROR;

    inputType = TOKEN_INPUT_FILE;

  } else if ( type == TOKEN_INPUT_STRING ) {
    // Input type : string

    inputType = TOKEN_INPUT_STRING;
    inputStr = file_or_str;

  } else
    return TOKENIZER_ERROR;

  return get_next_char();
}

// Tokenizer::get : Retrieves the next token from the input stream.
int Tokenizer::get( Token *token ) {

  int errno;

  while (1) {
    if ( ( errno = skip_whitespace() ) < 0 )
      return errno;

    if ( isdigit( lastChar ) || lastChar == '-' || lastChar == '.') {
      // Number token
      return get_number_token( token );
      
    } else if ( isalpha(lastChar) ) {
      // Symbol or keyword token
      return get_symbol_token( token );

    } else if ( lastChar == '\"' ) {
      // String token
      return get_string_token( token );

    } else if ( lastChar == '=' ) {

      token->type = TOKEN_EQUAL;

      return get_next_char();

    } else if ( lastChar == ';' ) {

      token->type = TOKEN_SEMICOLON;

      return get_next_char();

    } else if ( lastChar == '{' ) {

      token->type = TOKEN_LBRACKET;

      return get_next_char();

    } else if ( lastChar == '}' ) {

      token->type = TOKEN_RBRACKET;

      return get_next_char();

    } else if ( lastChar == '#' ) {

      skip_line();

    } else if ( end_of_input() ) {

      token->type = TOKEN_EOF;

      return get_next_char();

    } else {
      // Unknown token
      token->type = TOKEN_INVALID;
      
      return NO_ERROR;
    }
  }
}

// Tokenizer::close : Closes the input stream.
void Tokenizer::close( void ) {

  if ( inputType == TOKEN_INPUT_FILE ) {

    if ( inputFile != NULL )
      fclose( inputFile );

    inputFile = NULL;

  } else {

    inputStr = NULL;
    strPtr = 0;

  }
}

// Tokenizer::eoi : Checks the end of input
int Tokenizer::eoi( void ) {

  return end_of_input();
}
// Tokenizer::end_of_input : Checks whether the end of the input stream is
//   reached.
int Tokenizer::end_of_input( void ) {

  if ( inputType == TOKEN_INPUT_FILE ) {
    // Input type : file

    return feof( inputFile );

  } else {
    // Input type : string

    return ( lastChar == 0 );
  }
}

// Tokenizer::get_next_char : Retrieves the next character from the input stream
int Tokenizer::get_next_char( void ) {

  if ( inputType == TOKEN_INPUT_FILE ) {
    // Input type : file

      lastChar = fgetc( inputFile );

  } else {
    // Input type : string

    lastChar = inputStr[strPtr];
    if ( lastChar != 0 )
      strPtr += 1;

  }

  return NO_ERROR;

}

// Tokenizer::skip_whitespace :Skips all the whitespace following the current 
//  input position 
int Tokenizer::skip_whitespace( void ) {
  int   errno;

  while ( isspace( lastChar ) && !end_of_input() ) {

    if ( ( errno = get_next_char() ) < 0 )
      return errno;
  } 

  return NO_ERROR;
}

// Tokenizer::get_number_token : Retrieves a number token from the input
int Tokenizer::get_number_token( Token *token ) {

  char  str[MAX_TOKEN_LENGTH];
  int   strP = 0;
  bool  isfloat = false;
  int   errno;

  while ( lastChar == '.' || lastChar == 'e' 
          || lastChar == '-' || isdigit( lastChar ) ) {
    str[ strP++ ] = lastChar;

    if ( lastChar == '.' || lastChar == 'e' || lastChar == '-')
      isfloat = true;

    if ( ( errno = get_next_char() ) < 0 )
      return errno;
  }
  str[ strP ] = 0;

  if ( isfloat ) {

    if ( sscanf( str, "%f", &token->fvalue ) == 0 ) {
      token->type = TOKEN_INVALID;

    } else {

      token->type = TOKEN_FLOAT;
    }
  } else {

    if ( sscanf( str, "%i", &token->ivalue ) == 0 ) {

      token->type = TOKEN_INVALID;
    } else {

      token->type = TOKEN_INTEGER;
    }
  }

  return NO_ERROR;
}

// Tokenizer::get_number_token : Retrieves a symbol token from the input
int Tokenizer::get_symbol_token( Token *token ) {

  int   strP = 0;
  int   errno;

  if ( !isalpha( lastChar ) && lastChar != '_' ) {

    token->type = TOKEN_INVALID;
    return NO_ERROR;
  }

  while ( !isspace( lastChar ) ) {
    token->svalue[ strP++ ] = lastChar;

    if ( !isalpha( lastChar ) && !isdigit( lastChar ) && lastChar != '_') {

      token->type = TOKEN_INVALID;
      return NO_ERROR;
    }

    if ( ( errno = get_next_char() ) < 0 )
      return errno;
  }

  token->svalue[ strP ] = 0;
  token->type = TOKEN_SYMBOL;

  return NO_ERROR;
}

// Tokenizer::get_number_token : Retrieves a string token from the input
int Tokenizer::get_string_token( Token *token ) {

  int   strP = 0;
  int   errno;
  
  // Skip the opening quote
  if ( ( errno = get_next_char() ) < 0 )
    return errno;

  while ( lastChar != '\"' ) {
    token->svalue[ strP++ ] = lastChar;

    if ( ( errno = get_next_char() ) < 0 )
      return errno;
  }

  // Skip the closing quote
  if ( ( errno = get_next_char() ) < 0 )
    return errno;

  token->svalue[ strP ] = 0;
  token->type = TOKEN_STRING;

  return NO_ERROR;
}

// Tokenizer::skip_line : Skips the current line from the input stream
void Tokenizer::skip_line( void ) {

  while ( lastChar != '\n' && !end_of_input( ) )
    get_next_char();
}

