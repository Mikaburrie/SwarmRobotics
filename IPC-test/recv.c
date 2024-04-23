#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>

#define MAX_BUF 1024

static volatile int running = 1;
void handleInterruptSignal() {
	running = 0;
}

int main() {
	signal(SIGINT, handleInterruptSignal);

    int fileDescriptor;
    char* fifoPath = "/tmp/fifo";
	int dataBuffer[MAX_BUF];
    
    // Create and open non-blocking FIFO pipe
    mkfifo(fifoPath, 0666);
	fileDescriptor = open(fifoPath, O_RDONLY | O_NONBLOCK);
	
	while (running) {
		// Read and display data if available
		int length = read(fileDescriptor, dataBuffer, MAX_BUF);
		if (length > 0) printf("Received %d bytes: %d, %d\n", length, dataBuffer[0], dataBuffer[1]);	
	}
	
    // Close file and remove named FIFO pipe
	close(fileDescriptor);
    unlink(fifoPath);

    return 0;
}

