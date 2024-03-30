INC=-I.. `pkg-config --cflags opencv4`
LIB=`pkg-config --libs opencv4` -lm
MODULES=cube_detector camera_calibration

.PHONY: all
all: ${MODULES}

cube_detector: cube_detector.cpp
	g++ $(INC) -o $@ $< ${LIB}

camera_calibration: camera_calibration.cpp
	g++ $(INC) -o $@ $< ${LIB}

.PHONY: clean
clean:
	rm -f ${MODULES}
