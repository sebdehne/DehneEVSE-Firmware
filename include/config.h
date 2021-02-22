#define CLIENT_ID 1
#define SERVER_IP "192.168.1.1"
#define SERVER_PORT 9091
//#define DEBUG

void connectToWifi();
u_int32_t toInt(byte buf[]);
void intToByteArray(uint32_t input, byte dst[]);

void PWM_1kHz_tcc0_PIN_D3_start(uint8_t dutyCycle);
void PWM_1kHz_tcc0_PIN_D3_adjust_duty_cycle(uint8_t dutyCycle);

#define REQUEST_TYPE_PING 1
#define REQUEST_TYPE_FIRMWARE 2
#define RESPONSE_TYPE_PONG 1
