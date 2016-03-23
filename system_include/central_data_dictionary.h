#ifndef CENTRAL_DATA_DICTIONARY_H_
#define CENTRAL_DATA_DICTIONARY_H_

enum {
    POWERBOARD_I2C_KILLSWITCH, //Kill switch true or false
    POWERBOARD_I2C_VPOWERBUS,   //Power Bus ADC value (Voltage)
    POWERBOARD_I2C_VTHRUSTBUS,  //Thruster Bus ADC value (Voltage)
    POWERBOARD_I2C_IPOWERBUS,   //Power Bus ADC value (Current)
    POWERBOARD_I2C_ITHRUSTBUS,  //Thruster Bus ADC value (Current)
} powerboard_i2c_registers;

#endif /* CENTRAL_DATA_DICTIONARY_H_ */