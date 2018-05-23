

#include "UDG_Hokuyo.h"

int OpenSerialPort(const char* device)
{
	int fd = open(device, O_RDWR | O_NOCTTY);
	struct termios options;
  tcgetattr(fd, &options);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_oflag &= ~(ONLCR | OCRNL);
  tcsetattr(fd, TCSANOW, &options);
  return fd;
}

