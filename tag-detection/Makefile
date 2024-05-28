INC=-I.. `pkg-config --cflags opencv4`
LIB=`pkg-config --libs opencv4` -lm
MODULES=robot_detector tag_demo

.PHONY: all
all: ${MODULES}

robot_detector: robot_detector.cpp
	g++ $(INC) -o $@ $< ${LIB}

tag_demo: tag_demo.cpp octo_tags.cpp
	g++ $(INC) -o $@ $< ${LIB}

.PHONY: clean
clean:
	rm -f ${MODULES}
