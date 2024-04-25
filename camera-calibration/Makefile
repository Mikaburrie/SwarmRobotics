INC=-I.. `pkg-config --cflags opencv4`
LIB=`pkg-config --libs opencv4` -lm
MODULES=camera_calibration

.PHONY: all
all: ${MODULES}

camera_calibration: camera_calibration.cpp
	g++ $(INC) -o $@ $< ${LIB}

.PHONY: clean
clean:
	rm -f ${MODULES}
