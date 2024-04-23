#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#include "motor_command.h"

int main(int argc, char* argv[]) {
	// Check for correct argument count
	if (argc != 3) {
		printf("Expected 2 integer arguments\n");
		return 1;
	}
	
	int status = 0;
	
	status = openMotorControlFifo();
	if (status == -1) {
		printf("Failed to open pipe\n");
		return 1;
	}
	
	status = sendMotorCommand(atoi(argv[1]), atoi(argv[2]));
	if (status == -1) {
		printf("Failed to send command. Make sure to run with sudo\n");
		return 1;
	}
	
	closeMotorControlFifo();
	return 0;
}
