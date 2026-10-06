#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <memory>
#include <fstream>
#include "racecar_driver.h"
#include "blocking_stop.hpp"
#include "serial_deadline.hpp"
// Global variables for serial port
static int serial_fd = -1;
static std::unique_ptr<racecar_control::BlockingStop> blocking_guard;
static racecar_control::BlockingStop::Result write_packet(uint16_t motor_pwm,uint16_t servo_pwm);

extern "C" {

// Initialize serial port for racecar
int art_racecar_init(int speed, char *dev) {
  if (serial_fd >= 0) {
    close(serial_fd);
  }

  serial_fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd < 0) {
    std::cerr << "Failed to open serial port: " << dev << std::endl;
    return -1;
  }

  // Blocking-stop protection needs a bounded sender; never wait inside write().
  const int flags=fcntl(serial_fd,F_GETFL,0);
  if(flags<0 || fcntl(serial_fd,F_SETFL,flags|O_NONBLOCK)<0)return -1;

  // Configure serial port
  struct termios options;
  tcgetattr(serial_fd, &options);

  // Set baud rate
  speed_t baud;
  switch (speed) {
  case 9600:
    baud = B9600;
    break;
  case 19200:
    baud = B19200;
    break;
  case 38400:
    baud = B38400;
    break;
  case 57600:
    baud = B57600;
    break;
  case 115200:
    baud = B115200;
    break;
  default:
    baud = B38400;
    break;
  }

  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  // 8N1 (8 bits, no parity, 1 stop bit)
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // Disable hardware flow control
  options.c_cflag &= ~CRTSCTS;

  // Enable receiver, ignore control lines
  options.c_cflag |= (CREAD | CLOCAL);

  // Raw input
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  // Raw output
  options.c_oflag &= ~OPOST;

  tcsetattr(serial_fd, TCSANOW, &options);

  std::cout << "Serial port initialized: " << dev << " at " << speed << " baud"
            << std::endl;
  return 0;
}

// Send command to motor and servo
void blocking_stop_start(uint16_t neutral,uint16_t center,int brake_enabled,
                        uint16_t brake_pwm,double brake_s,double timeout_s,
                        int dry_run,const char *state_file) {
  const std::string path(state_file);
  blocking_guard=std::make_unique<racecar_control::BlockingStop>(
      [dry_run](uint16_t m,uint16_t s){return dry_run ? racecar_control::BlockingStop::ok:write_packet(m,s);},
      [path,dry_run](racecar_control::BlockingStop::Fault reason,bool sent){
        const char *name=reason==racecar_control::BlockingStop::control_blocked ? "CONTROL_LOOP_BLOCKED":
          reason==racecar_control::BlockingStop::serial_blocked ? "SERIAL_WRITE_BLOCKED":"SERIAL_WRITE_FAILED";
        std::ofstream out(path,std::ios::trunc);out<<"BLOCKING_STOP "<<name<<"\n";out.close();
        std::fprintf(stderr,"BLOCKING_STOP reason=%s stop_write=%s persistent_latch=%s%s\n",
          name,sent ? "accepted":"FAILED",out ? "saved":"FAILED",dry_run ? " DRY_RUN":"");
      },neutral,center,brake_enabled!=0,brake_pwm,brake_s,timeout_s);
  blocking_guard->start();
}
void blocking_stop_heartbeat(){if(blocking_guard)blocking_guard->heartbeat();}
int blocking_stop_fault(){return blocking_guard ? blocking_guard->fault():0;}
const char *blocking_stop_reason(){
  switch(blocking_stop_fault()){
    case 1:return "CONTROL_LOOP_BLOCKED";
    case 2:return "SERIAL_WRITE_BLOCKED";
    case 3:return "SERIAL_WRITE_FAILED";
    default:return "none";
  }
}
int blocking_stop_reset(){return !blocking_guard || blocking_guard->reset() ? 0:1;}
void blocking_stop_shutdown(){if(blocking_guard)blocking_guard->stop();}
unsigned char send_cmd_guarded(uint16_t *motor_pwm,uint16_t *servo_pwm){
  return blocking_guard ? (blocking_guard->send(*motor_pwm,*servo_pwm) ? 0:1):
    static_cast<unsigned char>(write_packet(*motor_pwm,*servo_pwm));
}
unsigned char send_cmd(uint16_t motor_pwm,uint16_t servo_pwm){return send_cmd_guarded(&motor_pwm,&servo_pwm);}

} // extern C

static racecar_control::BlockingStop::Result write_packet(uint16_t motor_pwm,uint16_t servo_pwm) {
  // Last serial-layer guard; the node applies the tighter calibrated limits.
  if (motor_pwm < 500 || motor_pwm > 2500 || servo_pwm < 500 || servo_pwm > 2500) {
    return racecar_control::BlockingStop::io_error;
  }
  if (serial_fd < 0) {
    return racecar_control::BlockingStop::io_error;
  }

  // Create command packet in MCU-expected format (7 bytes):
  // [0]=0xAA (header)
  // [1]=motor low, [2]=motor high
  // [3]=servo low, [4]=servo high
  // [5]=checksum (sum of bytes 1..4 mod 256)
  // [6]=0x55 (tail)
  unsigned char cmd[7] = {0};
  cmd[0] = 0xAA;
  cmd[1] = motor_pwm & 0xFF;        // motor low
  cmd[2] = (motor_pwm >> 8) & 0xFF; // motor high
  cmd[3] = servo_pwm & 0xFF;        // servo low
  cmd[4] = (servo_pwm >> 8) & 0xFF; // servo high

  // Calculate checksum as sum of bytes 1..4 (matching MCU code)
  unsigned int sum = 0;
  for (int i = 1; i <= 4; ++i) {
    sum += cmd[i];
  }
  cmd[5] = sum & 0xFF;
  cmd[6] = 0x55;

  // Avoid printing every frame at control frequency on the embedded computer.

  // Send command
  return racecar_control::write_with_deadline(serial_fd,cmd,7,20);
}
