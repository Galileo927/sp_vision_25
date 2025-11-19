#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

// --- Function to set serial port attributes ---
int set_serial_attributes(int fd, int speed)
{
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    tools::logger()->error(
      "Error {} from tcgetattr: {}", errno, std::strerror(errno));
    return -1;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
  tty.c_iflag &= ~IGNBRK;                      // disable break processing
  tty.c_lflag = 0;                             // no signaling chars, no echo,
                                               // no canonical processing
  tty.c_oflag = 0;                             // no remapping, no delays
  tty.c_cc[VMIN] = 0;                          // read doesn't block
  tty.c_cc[VTIME] = 5;                         // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls,
                                      // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    tools::logger()->error(
      "Error {} from tcsetattr: {}", errno, std::strerror(errno));
    return -1;
  }
  return 0;
}

int main()
{
  const char * portname = "/dev/ttyACM0";
  tools::logger()->info("Opening serial port: {}", portname);

  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    tools::logger()->error(
      "Error {} opening {}: {}", errno, portname, std::strerror(errno));
    return 1;
  }

  // Set speed to 115200 bps, 8n1 (no parity)
  set_serial_attributes(fd, B115200);

  tools::Exiter exiter;
  tools::logger()->info("Starting to read from serial port. Press Ctrl+C to exit.");

  while (!exiter.exit()) {
    char buf[128];
    int n = read(fd, buf, sizeof(buf));
    if (n > 0) {
      // Null-terminate the buffer to be safe
      buf[n] = '\0';
      // Print raw data as a string
      tools::logger()->info("Received {} bytes: {}", n, std::string(buf, n));
      
      // You can also print in hex to see non-printable characters
      // std::string hex_str;
      // for(int i=0; i<n; ++i) {
      //   char hex_buf[4];
      //   sprintf(hex_buf, "%02X ", (unsigned char)buf[i]);
      //   hex_str += hex_buf;
      // }
      // tools::logger()->info("Received (HEX): {}", hex_str);

    } else if (n < 0) {
      tools::logger()->error("Error reading from serial: {}", std::strerror(errno));
      break;
    }
    // If n == 0, it's a timeout, just loop again.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  close(fd);
  tools::logger()->info("Serial port closed.");
  return 0;
}
