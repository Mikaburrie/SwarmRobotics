// Defines functions to send commands to motor_control.c
// from other programs using a named FIFO pipe

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

int _motorControlFileDescriptor = -1;

int openMotorControlFifo() {
	// Open non-blocking FIFO pipe
	const char* fifoPath = "/tmp/motor_control";
	_motorControlFileDescriptor = open(fifoPath, O_WRONLY | O_NONBLOCK);
	
	// Return -1 if failed
	return (_motorControlFileDescriptor == -1)*-1;
}

int sendMotorCommand(int left, int right) {
	// Return -1 if FIFO pipe is not open
	if (_motorControlFileDescriptor == -1) return -1;
	
	// Create data for motor command
	int data[2];
	data[0] = left;
	data[1] = right;
	
	// Write data to non-blocking FIFO pipe
	int bytesWritten = write(_motorControlFileDescriptor, data, sizeof(data));
	
	// Return -1 if not all bytes are written
	return (bytesWritten != sizeof(data))*-1;
}

void closeMotorControlFifo() {
	// Close FIFO pipe
	close(_motorControlFileDescriptor);
	_motorControlFileDescriptor = -1;
}
