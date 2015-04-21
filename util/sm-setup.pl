#!/usr/bin/perl
#
# This file is part of RHexLib, 
#
# Copyright (c) 2001 The University of Michigan, its Regents,
# Fellows, Employees and Agents. All rights reserved, and distributed as
# free software under the following license.
# 
#  Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
# 
# 1) Redistributions of source code must retain the above copyright
# notice, this list of conditions, the following disclaimer and the
# file called "CREDITS" which accompanies this distribution.
# 
# 2) Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions, the following disclaimer and the file
# called "CREDITS" which accompanies this distribution in the
# documentation and/or other materials provided with the distribution.
# 
# 3) Neither the name of the University of Michigan, Ann Arbor or the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

######################################################################
# $Id: sm-setup.pl,v 1.6 2001/08/06 18:21:40 ulucs Exp $
#
# Sets up state machine header (*.hh) and code (*.cc) files 
# from a transition table.
#
# Created       : Eric Klavins, 02/04/2001
# Last Modified : Eric Klavins, 06/27/2001
#
# usage: ./sm-setup.pl ClassName
#
# There should be a file named "ClassName.dsc" in the same 
# directory. In that file, there should be multiple lines like:
#
#   Transition fromState myEvent toState
#
# and a single line of the form
#
#  Initial myState
#
# All state and event names mentioned in the *.dsc file 
# should be lowercase, as they will refer to instantiated objects. 
# This program will create class names from your names by 
# capitalizing the first letter of each name. 
# 

# get classname
#
$classname = $ARGV[0];

#
# make object name
#
@temp = split ( '', $classname );
$temp[0] =~ y/A-Z/a-z/;
$objectname = join ( '', @temp );

#
# name of transition description file
#
$dscfile = $ARGV[0] . ".dsc";

#
# name of output files
#
$header = $ARGV[0] . ".hh";
$headerguard = $ARGV[0];
$headerguard =~ tr/a-z/A-Z/;
$code = $ARGV[0] . ".cc";

if ( -e $header || -e $code ) {

    print "$header and/or $code already exist, aborting.\n";
    exit;

}

#
# get transitions and initial state
#

open ( FILE, $dscfile ) || die ( "Cannot open $dscfile: $!" );

$i=0;

while ( <FILE> ) {

  s/\n$//;
  @line = split ( /[\t ]+/, $_ );

  if ( $line[0] =~ /Transition/ ) {

    if ( $#line != 3 ) { die ( "$dscfile has a syntax error: $!" ); }
 
    #
    # make the transition statement
    #
    $trans[$i++] = "Transition ( $line[1], $line[2], $line[3] );";

    #
    # get state names
    #
    $line1 = 1;
    $line3 = 1;
    for ( $j=0; $j <= $#states; $j++ ) {

      if ( $states[$j] =~ $line[1] ) { $line1 = 0; }
      if ( $states[$j] =~ $line[3] ) { $line3 = 0; }

    }
    if ( $line1 == 1 ) { $states[$j++] = $line[1]; }
    if ( $line3 == 1 ) { $states[$j] = $line[3]; }

    #
    # get event names
    #
    $line2 = 1;
    for ( $j=0; $j <= $#events; $j++ ) {

      if ( $events[$j] =~ $line[2] ) { $line2 = 0; }

    }
    if ( $line2 == 1 ) { $events[$j] = $line[2]; }

  } elsif ( $line[0] =~ /Initial/ ) {

    if ( $#line != 1 ) { die ( "$dscfile has a syntax error: $!" ); }
    $initialstate = $line[1];
 
  }

}

close ( FILE );

if ( length ( $initialstate ) == 0 ) {

  print "No initial state. Specify one in the *.dsc file by \"Initial ( MyState )\"\n";
  exit;

}

#
# make event class names
#
for ( $j=0; $j <= $#events; $j++ ) {

  @temp = split ( '', $events[$j] );
  $temp[0] =~ tr/a-z/A-Z/;
  $eventclasses[$j] = join ( '', @temp );

}

#
# make state class names
#
for ( $j=0; $j <= $#states; $j++ ) {

  @temp = split ( '', $states[$j] );
  $temp[0] =~ tr/a-z/A-Z/;
  $stateclasses[$j] = join ( '', @temp );

}

#
# print the header file
#

open ( FILE, ">$header" );

print FILE "/*
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
 * file called \"CREDITS\" which accompanies this distribution.
 * 
 * 2) Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions, the following disclaimer and the file
 * called \"CREDITS\" which accompanies this distribution in the
 * documentation and/or other materials provided with the distribution.
 * 
 * 3) Neither the name of the University of Michigan, Ann Arbor or the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * \"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

/*
 *
 * RHEXLIB 
 *
 * $header
 *
 */

";

print FILE "#ifndef _" . $headerguard . "_HH\n"
         . "#define _" . $headerguard . "_HH\n\n";

print FILE "#include \"ModuleManager.hh\"
#include \"StateMachine.hh\"

class $classname : public StateMachine {

  public:

    $classname ( void );
    ~$classname ( void );
    void init ( void );
    void activate ( void );
    void deactivate ( void );

  private:

    // events\n";

for ( $j=0; $j <= $#events; $j++ ) {
  print FILE "    EventObject ( $eventclasses[$j] ) * $events[$j]" . ";\n"
}

print FILE "
    // states\n";

for ( $j=0; $j <= $#states; $j++ ) {
  print FILE "    StateObject ( $stateclasses[$j] ) * $states[$j]" . ";\n"
}

print FILE "
};

#endif\n";

close ( FILE );

#
# print code file
#

open ( FILE, ">$code" );

print FILE "/*
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
 * file called \"CREDITS\" which accompanies this distribution.
 * 
 * 2) Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions, the following disclaimer and the file
 * called \"CREDITS\" which accompanies this distribution in the
 * documentation and/or other materials provided with the distribution.
 * 
 * 3) Neither the name of the University of Michigan, Ann Arbor or the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * \"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

/*
 *
 * RHEXLIB 
 *
 * $code
 *
 */

#include \"$header\"

#define OWNER ( ( $classname * ) owner )

// Events ------------------------------------------------------------
";

for ( $j=0; $j <= $#events; $j++ ) {
  print FILE "bool $classname" . "::$eventclasses[$j]" . "::check ( void ) { return false; }\n"
}

printf FILE "\n// States ------------------------------------------------------------\n";

for ( $j=0; $j <= $#states; $j++ ) {
  print FILE "void $classname" . "::$stateclasses[$j]" . "::entry ( void ) {}\n";
  print FILE "void $classname" . "::$stateclasses[$j]" . "::during ( void ) {}\n";
  print FILE "void $classname" . "::$stateclasses[$j]" . "::exit ( void ) {}\n\n";
}

$name = $classname;
$name =~ tr/A-Z/a-z/;
print FILE $classname . "::$classname ( void ) : StateMachine ( \"$name\" ) {

  // allocate events\n";

for ( $j=0; $j <= $#events; $j++ ) {
  print FILE "  $events[$j] = new $eventclasses[$j] ( this, \"$eventclasses[$j]\" );\n";
}

print FILE "\n  // allocate states\n";

for ( $j=0; $j <= $#states; $j++ ) {
  print FILE "  $states[$j] = new $stateclasses[$j] ( this, \"$states[$j]\" );\n";
}

print FILE "\n  // transitions ( in the form < From, Event, To > )\n";

for ( $j=0; $j <= $#trans; $j++ ) {
  print FILE "  $trans[$j]\n";
}

print FILE "\n  // the initial state
  initialize ( $initialstate );

} 

$classname". "::~$classname ( void ) {\n\n";

for ( $j=0; $j <= $#events; $j++ ) {
  print FILE "  if ( $events[$j] ) delete ( $events[$j] );\n";
}

for ( $j=0; $j <= $#states; $j++ ) {
  print FILE "  if ( $states[$j] ) delete ( $states[$j] );\n";
}

print FILE "
}

void $classname". "::init ( void ) {

  StateMachine::init();

}

void $classname". "::activate ( void ) {

  StateMachine::activate();

}

void $classname". "::deactivate ( void ) {

  StateMachine::deactivate();

}
";

close ( FILE );
