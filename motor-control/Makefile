MODULES=motor_control send_motor_command

.PHONY: all
all: ${MODULES}

motor_control: motor_control.c
	gcc -pthread -o $@ $< -lpigpio
	
send_motor_command: send_motor_command.c
	gcc -o $@ $<

.PHONY: clean
clean:
	rm -f ${MODULES}
