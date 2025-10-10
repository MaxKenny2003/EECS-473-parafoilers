
// Include BMP5 Sensor API
#include "bmp5.h"

// Include Project Header
#include "altimeter.h"





/***********************************
       I2C Register Functions
***********************************/
// bool alt_i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len){

// }

// bool alt_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len){

// }

// GQ: Where do I define the functors in this file from the above functions^^^???
/***********************************
        Altimeter Functions
***********************************/

bool alt_init(alt_t *ctx, uint8_t addr, i2c_write_reg write_fn, i2c_read_reg  read_fn){
    
} // alt_init
