LIB=-lpigpio
MODULES=motor_control

.PHONY: all
all: ${MODULES}

motor_control: motor_control.c
	gcc -pthread -o $@ $< ${LIB}

.PHONY: clean
clean:
	rm -f ${MODULES}
