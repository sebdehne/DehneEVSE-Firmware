#define SERVER_IP "192.168.1.1"
#define SERVER_PORT 9091
#define SERVER_CONNECTION_TIMEOUT_IN_MS 10000
#define SERVER_CHARGING_TIMEOUT_IN_MS 300000
#define VERSION 9
//#define DEBUG
#define LOG_BUFFER_SIZE 1024
#define REQUEST_BODY_BUFFER_SIZE 1024


/* 
 * 
 * | SAMD21 PIN-name | SAMD21 PIN# | MRK 1010 PIN | Nano 33 IoT | F / TCC
 * | PA02 AIN[0]     | 3           | A0           | A0          | 
 * | PB02 AIN[10]    | 47          | A1           | A1          | 
 * | PB03 AIN[11]    | 48          | A2           | A7          | 
 * | PA04 AIN[4]     | 9           | A3           |             | 
 * | PA05 AIN[5]     | 10          | A4           |             | 
 * | PA06 AIN[6]     | 11          | A5           |             | 
 * | PA07 AIN[7]     | 12          | A6           |             | 
 * | PA11 AIN[19]    | 16          | D3           | A2          | TCC0/WO[3]
 * | PA10 AIN[18]    | 15          |              | A3          | 
 * | PB08 AIN[2]     | 7           |              | A4          | 
 * | PB09 AIN[3]     | 8           |              | A5          | 
 * | PA09 AIN[17]    | 14          |              | A6          | 
 * | PB11            | 20          |              | D3          | TCC0/WO[5]
 * | PB10            | 19          |              | D2          | TCC0/WO[4]
 *
 */