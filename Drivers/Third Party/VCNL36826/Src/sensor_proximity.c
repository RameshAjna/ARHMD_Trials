#include "sensor_proximity.h"
#include "I2C_Functions.h"

#define PROX_RANGE 	(float)200		//200mm operating range
#define PROX_RES 	(float)4096  	//12-bits

static struct TransferData Data;
const float multiplier = (PROX_RANGE/PROX_RES);
const float count_mul = (PROX_RES/PROX_RANGE);

SemaphoreHandle_t proxMutex = {0};

uint8_t prox_probe(void)
{
    return True;
}

uint8_t prox_open(void)
{
	if(proxMutex == 0){
		proxMutex = xSemaphoreCreateMutex();
	}
    uint8_t ret = 0;

    Data.WData[0] = 0x83;
        Data.WData[1] = 0x02;
        Data.Slave_Address = VCNL36826S_SLAVE_ADD;
        Data.RegisterAddress = ST_CONF;
        //ret = sensor_i2c_write(prox, ST_CONF, (uint8_t*)buf, 2);
        ret = WriteI2C_Bus(&Data);
        if(ret != True){
            return False;
        }
    // Write Standby Command for command 0x00
 // 0x00_L Start=1, Standby =1, bit0 =1
 // 0x00_H bit 1 =1
    //buf[0] = 0x83;
    //buf[1] = 0x02;
    Data.WData[0] = 0x83;
    Data.WData[1] = 0x02;
    Data.Slave_Address = VCNL36826S_SLAVE_ADD;
    Data.RegisterAddress = ST_CONF;
    //ret = sensor_i2c_write(prox, ST_CONF, (uint8_t*)buf, 2);
    ret = WriteI2C_Bus(&Data);
    if(ret != True){
        return False;
    }
 // Write Command for command 0x03
    // 0x03_L PS_Period = 40ms, PS_PERS = 4, PS_INT = logic by logic hi/low mode,  PS_SD = Power on
    // 0x03_H PS_IT =1T, PS_MPS = 1, bit 3 =1, PS_HG=0, PS_INT = trigger by logic hi/low mode
    //buf[0] = 0xB4;
    //buf[1] = 0x04;
    Data.WData[0] = 0xB4; //changed from 0xB4
    Data.WData[1] = 0x04; //changed from 0x04 //changed integration time(PS_IT)
    Data.Slave_Address = VCNL36826S_SLAVE_ADD;
    Data.RegisterAddress = PS_CONF1;
    //ret = sensor_i2c_write(prox, PS_CONF1, (uint8_t*)buf, 2);
    ret = WriteI2C_Bus(&Data);
    if(ret != True){
        return False;
    }

    // Write Command for command 0x08 for the PS Low power mode
    // when PS Low power mode enable by PS_LPEN =1, the chip PS_period is decided by PS_LPPER setting
    // 0x08_H PS_LPPER= 40ms
    //buf[0] = 0x00;
    //buf[1] = 0x01;
    Data.WData[0] = 0x00;
    Data.WData[1] = 0x01;  //changed from 0x01
    Data.Slave_Address = VCNL36826S_SLAVE_ADD;
    Data.RegisterAddress = PS_AC_LPPERI;
    //ret = sensor_i2c_write(prox, PS_AC_LPPERI, (uint8_t*)buf, 2);
    ret = WriteI2C_Bus(&Data);
    if(ret != True){
        return False;
    }
 
 
    // Write Command for command 0x04
    // 0x04_L PS_AF = Auto Mode, PS_FOR_TRIG = no
    // 0x04_H VCSEL_I = 20mA
    //buf[0] = 0x00;
    //buf[1] = 0x07;
    Data.WData[0] = 0x00;
    Data.WData[1] = 0x07;
    Data.Slave_Address = VCNL36826S_SLAVE_ADD;
    Data.RegisterAddress = PS_CONF3;
    //ret = sensor_i2c_write(prox, PS_CONF3, (uint8_t*)buf, 2);
    ret = WriteI2C_Bus(&Data);
    if(ret != True){
        return False;
    }

    return ret;
}
uint8_t prox_close(void)
{
    //sensor_infor_t *prox = get_sensor_descript(SENSOR_PROXI);
    //uint8_t buf[2] = {0};

	uint8_t ret = 0;

    // 0x08_H PS_LPPER: low power mode disable
    //buf[0] = 0x00;
    //buf[1] = 0x00;
	Data.WData[0] = 0x00;
    Data.WData[1] = 0x00;
    Data.Slave_Address = VCNL36826S_SLAVE_ADD;
    Data.RegisterAddress = PS_AC_LPPERI;
    //ret = sensor_i2c_write(prox, PS_AC_LPPERI, (uint8_t*)buf, 2);
    ret = WriteI2C_Bus(&Data);
    if(ret != True){
        return False;
    }

    // 0x03_L PS_SD :1 Power shutdown
    //buf[0] = 0x01;
    //buf[1] = 0x00;
    Data.WData[0] = 0x01;
    Data.WData[1] = 0x00;
    Data.Slave_Address = VCNL36826S_SLAVE_ADD;
    Data.RegisterAddress = PS_CONF1;
    //ret = sensor_i2c_write(prox, PS_CONF1, (uint8_t*)buf, 2);
    ret = WriteI2C_Bus(&Data);
    if(ret != True){
        return False;
    }

    // 0x00_L Standby =0
    //buf[0] = 0x81;
    //buf[1] = 0x02;
    Data.WData[0] = 0x81;
    Data.WData[1] = 0x02;
    Data.Slave_Address = VCNL36826S_SLAVE_ADD;
    Data.RegisterAddress = ST_CONF;
    //ret = sensor_i2c_write(prox, ST_CONF, (uint8_t*)buf, 2);
    ret = WriteI2C_Bus(&Data);
    if(ret != True){
        return False;
    }

    return True;
}
uint16_t prox_get_data(void)
{
    //sensor_infor_t *prox = get_sensor_descript(SENSOR_PROXI);
/*
    uint8_t lsb = 0, msb = 0;
    uint16_t ps_value = 0, prox_raw = 0;
*/
    uint16_t ret = 0;
    
    Data.RegisterAddress = PS_DATA;
    Data.Slave_Address = VCNL36826S_SLAVE_ADD;
    //ret = sensor_i2c_read(prox, PS_DATA, (uint8_t*)buf, 2);
    ret = ReadI2C_Bus(&Data);
    if(ret != HAL_OK){
//    	printf("prox read err: %d\r\n", ret);
        return False;
    }
/*
    lsb = buf[0];
    msb = buf[1];
    ps_value = (uint16_t)msb;
    prox_raw = ((ps_value << 8) | (uint16_t)lsb);
    sensor_dbg(BRCM_TRACE_INFO, "prox raw : %d \r\n", prox_raw);
*/


    ret = ((uint16_t)Data.RData[1]<<8|(uint16_t)Data.RData[0]);
    //sprintf(debugBuffer, "Lbyte - 0 %x 1 %x ret %ld \r\n", Data.RData[0], Data.RData[1], ret);
    //HAL_UART_Transmit(&huart2, debugBuffer, strlen(debugBuffer), 100);
    //buf[0] = Data.RData[1] << 8;
    //buf[1] = Data.RData[0];

    return ret;
}


uint8_t prox_isr(void)
{
    return True;
}

uint8_t prox_ioctl(uint8_t cmd, volatile uint8_t *piData, uint8_t length)
{
    return True;
}

uint8_t prox_sleep(void)
{
    return True;
}

uint8_t prox_measure(uint8_t val)
{
    return True;
}

uint8_t prox_selftest(volatile uint8_t *piData)
{
    return True;
}

/**Sensor APIs**/

//Set the Low Threshold
uint8_t SET_PS_LowThreshold(uint16_t LowThreshold)
{
	uint8_t ret = 0;
	uint8_t LowByte = (LowThreshold & 0xFF);
	uint8_t HighByte = LowThreshold>>8;

	Data.Slave_Address = VCNL36826S_SLAVE_ADD;
	Data.RegisterAddress = PS_THDL;
	Data.WData[0] = LowByte;
	Data.WData[1] = HighByte;
	ret = WriteI2C_Bus(&Data);
	if(ret != HAL_OK){
		return False;
	}

	return True;
}

//Set the High Threshold
uint8_t SET_PS_HighThreshold(uint16_t HighThreshold)
{
	uint8_t ret = 0;
	uint8_t LowByte = (HighThreshold & 0xFF);
	uint8_t HighByte = HighThreshold>>8;

	Data.Slave_Address = VCNL36826S_SLAVE_ADD;
	Data.RegisterAddress = PS_THDH;
	Data.WData[0] = LowByte;
	Data.WData[1] = HighByte;
	ret = WriteI2C_Bus(&Data);

	if(ret != HAL_OK){
		return False;
	}

	return True;
}

//Get the Low Threshold
uint16_t GET_PS_LowThreshold(void)
{
	uint16_t ret = 0;

	Data.Slave_Address = VCNL36826S_SLAVE_ADD;
	Data.RegisterAddress = PS_THDL;
	ret = ReadI2C_Bus(&Data);
	if(ret != HAL_OK){
		return False;
	}

	ret = ((uint16_t)Data.RData[1]<<8|(uint16_t)Data.RData[0]);
	return ret;
}

//Get the High Threshold
uint16_t GET_PS_HighThreshold(void)
{
	uint16_t ret = 0;

	Data.Slave_Address = VCNL36826S_SLAVE_ADD;
	Data.RegisterAddress = PS_THDH;
	ret = ReadI2C_Bus(&Data);
	if(ret != HAL_OK){
		return False;
	}

	ret = ((uint16_t)Data.RData[1]<<8|(uint16_t)Data.RData[0]);
	return ret;
}

//Set the Interrupt Mode
uint8_t SET_PS_INT(uint8_t InterruptMode)
{

	Data.Slave_Address = VCNL36826S_SLAVE_ADD;
	Data.RegisterAddress = PS_CONF1;
	if(ReadI2C_Bus(&Data) != HAL_OK){
		return False;
	}

	Data.WData[0] = (Data.RData[0]&~(0x03<<2))|InterruptMode;
	Data.WData[1] = Data.RData[1];
	if(WriteI2C_Bus(&Data) != HAL_OK){
		return False;
	}

	return True;
}

void read_reg(uint8_t addr)
{
	Data.Slave_Address = VCNL36826S_SLAVE_ADD;
	Data.RegisterAddress = addr;
	if(ReadI2C_Bus(&Data) == HAL_OK)
	printf("CANCELLATION: %x %x\r\n", Data.RData[0], Data.RData[1]);
	else
		printf("reg rd err\r\n");
}

float convert_to_MM(uint16_t counts)
{
	float retval = (float)counts * multiplier;

	return ((float)200 - retval);
}

uint16_t convert_to_counts(uint16_t mm)
{
	float retval = (float)mm * count_mul;

	return (4096 - (uint16_t)retval);
}


#if 0 /* NOT_IMPLEMENTED */
static uint8_t buffer_prox[prox_unit * prox_buffer_size];
static const struct inject_operations prox_i_ops = {
    .before_open = null_operation,
    .after_open = after_open_default,
    .before_close = null_operation,
    .after_close = after_close_default,
    .add_rate = add_rate_default,
    .remove_rate = remove_rate_default,
    .select = select_default,
};

static const struct sensor_func_t prox_func = {
    .probe = prox_probe,
    .open = prox_open,
    .close = prox_close,
    .read = prox_read,
    .sleep = prox_sleep,
    .measure = prox_measure,
    .selftest = prox_selftest,
    .ioctl = prox_ioctl,
    .isr = prox_isr,
};

extern freq_client null_client;
static sensor_infor_t prox_descriptor = {
    .sensor_id = SENSOR_PROXI,

    .proved = False,
    .hit = False,
    .buf_start = 0,
    .buf_end = 0,
    .used_count = 0,
    .offset = 0,
    .level = -1,
    .addr = VCNL36826S_SLAVE_ADD,

    .dimension = prox_dimension,
    .unit_size = prox_unit,
    .buffer = buffer_prox,
    .buf_length = sizeof(buffer_prox),
    .i_ops = &prox_i_ops,

    .sending_delays[0].flist_end = 0,
    .sending_delays[0].flist_size = 0,
    .sending_delays[0].current = &null_client,

    .sending_delays[1].flist_end = 0,
    .sending_delays[1].flist_size = 0,
    .sending_delays[1].current = &null_client,

    .sampling_delays.flist_end = 0,
    .sampling_delays.flist_size = 0,
    .sampling_delays.current = &null_client,

    .s_func = &prox_func,
};

sensor_infor_t *get_prox_descriptor(void)
{
    return &prox_descriptor;
}
#endif /*NOT_IMPLEMENTED*/

