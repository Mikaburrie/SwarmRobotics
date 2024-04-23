#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

int main() {
    int fileDescriptor;
    char* fifoPath = "/tmp/fifo";
    int data[2] = {1235, -9581};

    // Write data to non-blocking FIFO pipe
    fileDescriptor = open(fifoPath, O_WRONLY | O_NONBLOCK);
    int bytesWritten = write(fileDescriptor, data, sizeof(data));
    printf("File descriptor: %d, Bytes written: %d\n", fileDescriptor, bytesWritten);
    close(fileDescriptor);

    return 0;
}

