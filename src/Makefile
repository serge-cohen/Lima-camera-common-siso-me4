# siso-me4 plugin Makefile
# Copyright (C) 2013 IPANEMA USR3461, CNRS/MCC.
# Written by Serge Cohen <serge.cohen@synchrotron-soleil.fr>
#
# This file is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 3 of
# the License, or (at your option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public
# License along with this file. If not, see <http://www.gnu.org/licenses/>.



siso-me4-objs = SiSoME4Grabber.o SiSoME4SerialLine.o

SRCS = $(siso-me4-objs:.o=.cpp) 

CXXFLAGS += \
	-I../include \
	-I../../../../hardware/include \
	-I../../../../common/include \
	-I/opt/siso/include \
	-Wall -pthread -fPIC -g

all: SiSoME4.o

SiSoME4.o: $(siso-me4-objs)
	$(LD) -o $@ -r $+

clean:
	rm -f *.o *.P

%.o : %.cpp
	$(COMPILE.cpp) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)

.PHONY: check-syntax
check-syntax:
	$(CXX) -Wall -Wextra -fsyntax-only $(CXXFLAGS) $(CHK_SOURCES)