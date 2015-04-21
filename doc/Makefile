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
 # $Id: Makefile,v 1.4 2001/07/20 15:14:06 ulucs Exp $
 #
 # The Makefile for the Module Manager
 #
 # Created       : Uluc Saranli, 07/12/2001
 # Last Modified : Uluc Saranli, 07/12/2001
 #
 ######################################################################

 # ***********************************************************
 # ** This Makefile was originally created by Richard Groff **
 # ***********************************************************
 #
 # - By convention, we have a main tex file $(FILE).tex, from which 
 # various sections of the manual are input.  
 #
 # - "make spell" will run "ispell -t" on # $(FILE).tex and on all files
 # inputed in $(FILE).tex which have the .tex suffix explicitly
 # written out.
 #
 # - In latex, \input{file.tex} and \input{file} will both input the
 # file.tex, but only the first will cause "make spell" to spell check
 # file.tex.  Thus, we input files I don't want to spell check, such as
 # macros, with \input{file}, but sections of papers with
 # \input{file.tex}
 #
 # - The TITLE variable provides the name for the generated postscript file
 # - The GRAPHICSDIR variable should be a : separated list of directories to 
 # search for latex files
 # - DVIVIEW and DVIPS specify the preferences for the dvi viewer and
 # dvi-postscript conversion, respectively.

TITLE   = RHexManual
FILE	= body
TEXINPUTS = 

# commands
RM  = rm
LATEX = latex
BIBTEX = bibtex
MAKEINDEX = makeindex
DVIPS = dvips -t letter
DVIVIEW = xdvi -p 600 -s 8 -k
DISTILL = distill
#

all:
	@echo "Usage for Latex make file:"
	@echo "make dvi   --   run latex on $(FILE).tex, produce $(FILE).dvi"
	@echo "make full  --   run latex, bibtex, latex, latex on $(FILE).tex"
	@echo "make ps    --   Do 'make full' on $(FILE).tex and then 'dvips'"
	@echo "                to produce $(TITLE).ps"
	@echo "make view  --   start the dvi viewer, \"$(DVIVIEW)\" to look"
	@echo "                at $(FILE).dvi" 
	@echo "make spell --   run 'ispell -t' on $(FILE).tex and all *.tex"
	@echo "                files \\input-ed from $(FILE).tex" 
	@echo "make bib   --   run bibtex on $(FILE).tex"
	@echo "make clean --   remove all temp files"

dvi:
	sh -c '\
	       export TEXINPUTS=.:$(TEXINPUTS):      ; \
	       $(LATEX) \\nonstopmode\\input $(FILE).tex   '

bib:
	bibtex $(FILE)

full:
	sh -c '\
	       export TEXINPUTS=.:$(TEXINPUTS):       ; \
	       $(LATEX) \\nonstopmode\\input $(FILE).tex  ; \
	       $(BIBTEX) $(FILE)                          ; \
	       $(LATEX) \\nonstopmode\\input $(FILE).tex  ; \
	       $(MAKEINDEX) $(FILE)                       ; \
	       $(LATEX) \\nonstopmode\\input $(FILE).tex  ; '


ps:
	sh -c '\
	       export TEXINPUTS=.:$(TEXINPUTS):       ; \
	       $(LATEX) \\nonstopmode\\input $(FILE).tex  ; \
	       $(BIBTEX) $(FILE)                          ; \
	       $(LATEX) \\nonstopmode\\input $(FILE).tex  ; \
	       $(MAKEINDEX) $(FILE)                       ; \
	       $(LATEX) \\nonstopmode\\input $(FILE).tex  ; \
	       $(DVIPS) -o $(TITLE).ps $(FILE).dvi       ; '
	@echo Wrote file $(TITLE).ps  

pdf:
	sh -c '\
	       export TEXINPUTS=.:$(TEXINPUTS):       ; \
	       $(LATEX) \\nonstopmode\\input $(FILE).tex  ; \
	       $(BIBTEX) $(FILE)                          ; \
	       $(LATEX) \\nonstopmode\\input $(FILE).tex  ; \
	       $(MAKEINDEX) $(FILE)                       ; \
	       $(LATEX) \\nonstopmode\\input $(FILE).tex  ; \
	       $(DVIPS) -Ppdf -o $(TITLE)_pdf.ps $(FILE).dvi       ; '
	@echo Wrote file $(TITLE)_pdf.ps
	$(DISTILL) $(TITLE)_pdf.ps $(TITLE).pdf  

$(FILE).dvi: 
	make full

clean:
	$(RM) *%  *~ *.dvi *.idx *.aux *.bbl *.blg *.ilg *.ind *.lof *.log *.lot *.lof *.toc *.loa 2> /dev/null ; true 

realclean:
	$(RM) *%  *~ *.dvi $(TITLE).ps $(TITLE)_pdf.ps *.idx *.aux *.bbl *.blg *.ilg *.ind *.lof *.log *.lot *.lof *.toc *.loa 2> /dev/null ; true 

view: $(FILE).dvi
	sh -c '\
	       export TEXINPUTS=.:$(TEXINPUTS):      ; \
	       $(DVIVIEW) $(FILE).dvi &'	

spell:
	ispell -t $(FILE).tex `grep '\.tex' $(FILE).tex | sed 's/.*\\input{\(.*\)\.tex}/\1.tex/'`










