CXX = g++
INCLUDES = -I/usr/include
CXXFLAGS = -std=c++11 -O3 $(INCLUDES)
LIBS = -L. -Llib -lomap3isp -lopencv_highgui -lopencv_core -lopencv_imgproc

.SUFFIXES: .cc
.cc.o:
	$(CXX) $(CXXFLAGS) -c $<

SRC = libfreenect_cv.cpp

OBJ = $(addstuff .o, $(basename $(SRC)))

all: libfreenect_cv

main: $(OBJ)
	g++ -o eyeBug $(OBJ) $(LIBS)

depend:
	makedepend $(CXXFLAGS) -Y $(SRC)

clean:
	rm -f $(OBJ)

