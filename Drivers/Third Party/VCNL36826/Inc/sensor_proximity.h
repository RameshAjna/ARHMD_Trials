
#ifndef _PROXIMITY_H_
#define _PROXIMITY_H_

#include <stdio.h>
#include <stdlib.h>


#define prox_dimension                     1
#define prox_unit                          (prox_dimension)
#define prox_buffer_size                   1

typedef enum {
	False = -1,
	True ,
}return_type_t;

#define VCNL36826S_SLAVE_ADD 0x60
#define ST_CONF 0x00
#define PS_CONF1 0x03
#define PS_CONF3 0x04
#define PS_THDL 0x05
#define PS_THDH 0x06
#define PS_CANC 0x07
#define PS_AC_LPPERI 0x08
#define PS_DATA 0xF8
#define VCNL36826S_FLAG 0xF9
#define DEV_ID 0xFA
#define CALIB_DATA 0xFB


#define LEVEL1_THDH 20
#define LEVEL1_THDL 10
#define LEVEL2_THDH 70
#define LEVEL2_THDL 50
#define LEVEL3_THDH 200
#define LEVEL3_THDL 150
#define PS_INT_DIS	(0x00<<2)
#define PS_INT_HILO	(0x01<<2)
#define PS_INT_NOR	(0x01<<3)

uint8_t prox_probe(void);
uint8_t prox_open(void);
uint8_t prox_close(void);
uint16_t prox_get_data(void);
uint8_t prox_isr(void);
uint8_t prox_ioctl(uint8_t cmd, volatile uint8_t *piData, uint8_t length);
uint8_t prox_sleep(void);
uint8_t prox_measure(uint8_t val);
uint8_t prox_selftest(volatile uint8_t *piData);

//sensor_infor_t *get_prox_descriptor(void);

/**Sensor APIs**/
uint8_t SET_PS_LowThreshold(uint16_t LowThreshold);
uint8_t SET_PS_HighThreshold(uint16_t HighThreshold);
uint16_t GET_PS_LowThreshold(void);
uint16_t GET_PS_HighThreshold(void);
uint8_t SET_PS_INT(uint8_t InterruptMode);

/** Conversion APIs **/
float convert_to_MM(uint16_t counts);
uint16_t convert_to_counts(uint16_t mm);

#endif /*_PROXIMITY_H_*/

