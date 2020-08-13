
SHELL = /bin/sh

CXX:=ccache $(CXX)

include Mk/libs.mk
include Mk/comrob.mk

CXXFLAGS+=-std=c++17
CPPFLAGS+=$(LOCAL_CFLAGS)
LDFLAGS+=$(LOCAL_LDFLAGS)

CPPFLAGS+=$(CRL_CFLAGS) $(BOOST_CFLAGS) $(CAIRO_CFLAGS) $(LOG4CXX_CPPFLAGS)
LDFLAGS+=$(CRL-ALGORITHM) $(CRL-GUI_LDFLAGS) $(CRL_LDFLAGS) $(CAIRO_LDFLAGS) $(BOOST_LDFLAGS) $(LOG4CXX_LDFLAGS) -lstdc++fs

CXXFLAGS+=-O0 -march=native -DDEBUG -g

OBJS=\
     t_mgctsp.o\
     grasp-constrained.o\
     coords.o\

TARGET=t_mgctsp

all: $(TARGET)

bin: $(TARGET)

$(OBJS): %.o: %.cc
	$(CXX) -c $< $(CXXFLAGS) $(CPPFLAGS) -o $@

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) -o $@

clean:
	$(RM) $(OBJS) $(TARGET)

fresh: clean all


