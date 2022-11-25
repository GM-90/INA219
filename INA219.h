/******************************************************************************
* TI INA219 hi-side i2c current/power monitor Library
*
* Libreria Original por:
*    John De Cristofaro (6 May 2012)
*
* Modificado por:
*    Gabriel Mirabelli (18-01-2022)
*
* Probado en STM32
*
* This library does not handle triggered conversion modes. It uses the INA219
* in continuous conversion mode. All reads are from continous conversions.
*
* A note about the gain (PGA) setting:
*	The gain of the ADC pre-amplifier is programmable in the INA219, and can
*	be set between 1/8x (default) and unity. This allows a shunt voltage 
*	range of +/-320mV to +/-40mV respectively. Something to keep in mind,
*	however, is that this change in gain DOES NOT affect the resolution
*	of the ADC, which is fixed at 1uV. What it does do is increase noise
*	immunity by exploiting the integrative nature of the delta-sigma ADC.
*	For the best possible reading, you should set the gain to the range
*	of voltages that you expect to see in your particular circuit. See
*	page 15 in the datasheet for more info about the PGA.
*
*
* MIT license
******************************************************************************/
#ifndef ina219_h
#define ina219_h


#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>


// INA219 memory registers
#define CONFIG_R         0x00    // configuration register
#define V_SHUNT_R        0x01    // differential shunt voltage
#define V_BUS_R          0x02    // bus voltage (wrt to system/chip GND)
#define P_BUS_R          0x03    // system power draw (= V_BUS * I_SHUNT)
#define I_SHUNT_R        0x04    // shunt current
#define CAL_R            0x05    // calibration register

#define INA_RESET        0xFFFF  // send to CONFIG_R to reset unit

#define CONFIG_DEFAULT   0x399F

// config. register bit labels
#define RST     15
#define BRNG    13
#define PG1     12
#define PG0     11
#define BADC4   10
#define BADC3    9
#define BADC2    8
#define BADC1    7
#define SADC4    6
#define SADC3    5
#define SADC2    4
#define SADC1    3
#define MODE3    2
#define MODE2    1
#define MODE1    0

typedef enum {
    FSR_16V,
    FSR_32V
} Bus_Voltage_Range;

typedef enum {
    Shunt_range_40mV,
    Shunt_range_80mV,
    Shunt_range_160mV,
    Shunt_range_320mV
} PGA;

typedef enum {
    Samples_1_Resolution_9bits    = 0,
    Samples_1_Resolution_10bits   = 1,
    Samples_1_Resolution_11bits   = 2,
    Samples_1_Resolution_12bits   = 3,
    Samples_2_Resolution_12bits   = 9,
    Samples_4_Resolution_12bits   = 10,
    Samples_8_Resolution_12bits   = 11,
    Samples_16_Resolution_12bits  = 12,
    Samples_32_Resolution_12bits  = 13,
    Samples_64_Resolution_12bits  = 14,
    Samples_128_Resolution_12bits = 15,
} ADC_Resolution;

typedef enum {
    Power_down,
    Shunt_voltage_triggered,
    Bus_voltage_triggered,
    Shunt_and_bus_triggered,
    ADC_off,
    Shunt_voltage_continuous,
    Bus_voltage_continuous,
    Shunt_and_bus_continuous
} Op_Mode;

// default values
#define D_I2C_ADDRESS    0x40 // (64)



class INA219
{
  public:
    INA219(uint8_t addr=D_I2C_ADDRESS);
    
    bool testConnection();

    void calibrate(float r_shunt, float i_max_expected);

    void configure(Bus_Voltage_Range range = FSR_32V, PGA gain = Shunt_range_320mV, ADC_Resolution bus_adc = Samples_1_Resolution_12bits, ADC_Resolution shunt_adc = Samples_1_Resolution_12bits, Op_Mode mode = Shunt_and_bus_continuous);

    void reset();

    int16_t shuntVoltageRaw();
    int16_t busVoltageRaw();
    float shuntVoltage();
    float busVoltage();
    float shuntCurrent();
    float busPower();


  private:
    uint8_t i2c_address;
    float r_shunt, current_lsb, power_lsb;
    
    //Registros de configuracion y calibracion
    uint16_t CONFIG, CAL;

    //Constantes de conversion de Tension
    const float LSB_Shunt_Voltage_Register = 0.00001;       //10uV
    const float LSB_Bus_Voltage_Register = 0.004;           //4mV

    int16_t read16(uint8_t addr);
    void write16(uint8_t addr, uint16_t data);

};

#endif