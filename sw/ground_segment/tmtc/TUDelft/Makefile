# 
#   $Id: Makefile 3641 2009-07-03 21:53:55Z hecto $
#   Copyright (C) 2003-2006 Pascal Brisset, Antoine Drouin
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.  
# 

# Quiet
Q=@

include ../../../../conf/Makefile.local
CONF = ../../../../conf
VAR = ../../../../var

all: sony visionresult

clean:
	rm -f sony *.bak *~ core *.o .depend *.opt *.out *.cm*

OCAMLC = ocamlc
OCAMLOPT = ocamlopt
OCAMLLIB = ../../../lib/ocaml
OCAMLNETINCLUDES=$(shell ocamlfind query -r -i-format netstring)
OCAMLNETCMA=$(shell ocamlfind query -r -a-format -predicates byte netstring)
INCLUDES= -I $(OCAMLLIB) -I ../../multimon -I +lablgtk2 -I +xml-light $(OCAMLNETINCLUDES)
LIBPPRZCMA=$(OCAMLLIB)/lib-pprz.cma


vision : visionresult.ml
	$(OCAMLC) -custom $(OCAMLCFLAGS) $(INCLUDES) unix.cma str.cma xml-light.cma $(LIBS) gtkInit.cmo $^ -o $@


sony : sony.cmo ../../../lib/ocaml/xlib-pprz.cma ../../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -I ../../cockpit -o $@ unix.cma str.cma xml-light.cma lablgtk.cma lablglade.cma $(OCAMLNETCMA) glibivy-ocaml.cma lib-pprz.cma lablgnomecanvas.cma xlib-pprz.cma gtkInit.cmo  $<

visionresult : visionresult.cmo ../../../lib/ocaml/xlib-pprz.cma ../../../lib/ocaml/lib-pprz.cma
	@echo OL $@
	$(Q)$(OCAMLC) -custom $(INCLUDES) -I ../../cockpit -o $@ unix.cma str.cma xml-light.cma lablgtk.cma lablglade.cma $(OCAMLNETCMA) glibivy-ocaml.cma lib-pprz.cma lablgnomecanvas.cma xlib-pprz.cma gtkInit.cmo  $<


%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) -c $<

%.cmx : %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(INCLUDES) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $<


CC = gcc
GLIB_CFLAGS  = -Wall  `pkg-config glib-2.0 --cflags`
GLIB_LDFLAGS =  `pkg-config glib-2.0 --libs` -lglibivy -lpcre
GTK_CFLAGS  = -Wall  `pkg-config gtk+-2.0 --cflags`
GTK_LDFLAGS =  `pkg-config gtk+-2.0 --libs` -lglibivy -lpcre


#
# Dependencies
#

.depend: Makefile
	ocamldep -I ../../../lib/ocaml *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif

