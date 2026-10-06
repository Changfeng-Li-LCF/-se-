
//
//racecar
//

#ifndef RACECAR_DRIVER
#define RACECAR_DRIVER
#include <stdint.h>
#include <unistd.h>

#if defined(__cplusplus)
extern "C" {
#endif




int Open_Serial_Dev(char *dev);
unsigned char send_cmd_guarded(uint16_t *motor_pwm, uint16_t *servo_pwm);
void blocking_stop_start(uint16_t neutral, uint16_t center, int brake_enabled,
                        uint16_t brake_pwm, double brake_s, double timeout_s,
                        int dry_run, const char *state_file);
void blocking_stop_heartbeat();
int blocking_stop_fault();
const char *blocking_stop_reason();
int blocking_stop_reset();
void blocking_stop_shutdown();

int art_racecar_init(int speed,char *dev);//设置波特率和串口设备


unsigned char send_cmd(uint16_t motor_pwm,uint16_t servo_pwm);//发送指令(电机PWM,舵机PWM),单位为us.







#if defined(__cplusplus)
}
#endif
#endif
