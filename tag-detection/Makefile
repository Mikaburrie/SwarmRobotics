INC=-I.. `pkg-config --cflags opencv4`
LIB=`pkg-config --libs opencv4` -lm
MODULES=cube_detector

.PHONY: all
all: ${MODULES}

cube_detector: cube_detector.cpp
	g++ $(INC) -o $@ $< ${LIB}

.PHONY: clean
clean:
	rm -f ${MODULES}
