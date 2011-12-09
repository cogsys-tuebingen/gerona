# Indicates compilator to use
CC      = g++

# Specifies compilator options
IDIR    = ./STANN/include
DEBUG   = -g -fno-stack-protector
CFLAGS  = -O3 -Wall -D LINUX -ftemplate-depth-520 -I$(IDIR) $(DEBUG) -I/home/khan/rabot/include/opencv # `pkg-config --cflags opencv` 
LDFLAGS =  $(DEBUG)
LDLIBS  = -g -L/home/khan/rabot/lib -lcxcore -lcv -lcvaux -lhighgui -lml # `pkg-config --libs opencv` 

# Files extensions .cpp, .o
SUFFIXES = .cpp .o 
.SUFFIXES: $(SUFFIXES) .

# Name of the main program
PROG  = surf

# Object files .o necessary to build the main program
OBJS  = fasthessian.o integral.o descriptor.o mymain.o LBPmain.o LTPmain.o LATPmain.o CCHmain.o surf.o utils.o ipoint.o LBP.o LTP.o LATP.o CCH.o

all: $(PROG)

# Compilation and link
$(PROG): $(OBJS)
	$(CC) $(LDFLAGS) -o $(PROG) $(OBJS) $(LDLIBS)

.cpp.o:
	$(CC)   $(CFLAGS) -c $< -o $@

clean:
	-rm -f $(PROG)
	-rm -f *.o
