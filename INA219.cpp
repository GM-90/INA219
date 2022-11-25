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

#include "INA219.h"

INA219::INA219(uint8_t addr) {
    i2c_address = addr;
    CONFIG=CONFIG_DEFAULT;
}

bool INA219::testConnection(){
    uint8_t error;
    Wire.beginTransmission(i2c_address);
    error = Wire.endTransmission();

    if( !error ){
      return true;
    }
    else{
      return false;
    }
}



// calibration of equations and device
// @param shunt_val             Valor de la resistencia SHUNT
// @param i_max_expected        Valor Maximo de corriente en el Shunt. Definido por diseño.
//                              El sensor ajusta el valor maximo de medicion a este valor de corriente.
void INA219::calibrate(float shunt_val, float i_max_expected)
{
    uint16_t cal;
    r_shunt = shunt_val;
    current_lsb = i_max_expected / 32767;
    cal = (uint16_t) ((0.04096)/(current_lsb*r_shunt));
    power_lsb = current_lsb * 20;

    CAL=cal;
    write16(CAL_R, CAL);
}


//Configurar Rangos de medicion y ganancia del PGA
//@param range                  Rango de voltaje de entrada. Modifica la ganancia para la medicion de Tension.
//@param gain                   Ganancia de entrada para la medicion de Corriente.
//@param bus_adc                Resolucion del ADC para la medicion de Tension.
//@param shunt_adc              Resolucion del ADC para la medicion de Corriente.
//@param mode                   Modo de operacion.
void INA219::configure(Bus_Voltage_Range range, PGA gain, ADC_Resolution bus_adc, ADC_Resolution shunt_adc, Op_Mode mode)
{
    CONFIG = 0;
    CONFIG |= (range << BRNG | gain << PG0 | bus_adc << BADC1 | shunt_adc << SADC1 | mode);
    write16(CONFIG_R, CONFIG);		
}

// resets the INA219
void INA219::reset()
{
    write16(CONFIG_R, INA_RESET);
    delay(5);
}

// returns the raw binary value of the shunt voltage
int16_t INA219::shuntVoltageRaw()
{
    return read16(V_SHUNT_R);
}

// returns the shunt voltage in volts.
float INA219::shuntVoltage()
{
    float temp;
    temp = shuntVoltageRaw();
    return (temp * LSB_Shunt_Voltage_Register);
}

// returns raw bus voltage binary value
int16_t INA219::busVoltageRaw()
{
    return read16(V_BUS_R);
}

// returns the bus voltage in volts
float INA219::busVoltage()
{
    int16_t temp;
    temp = busVoltageRaw();
    temp >>= 3;
    return (temp * LSB_Bus_Voltage_Register);
}

// returns the shunt current in amps
float INA219::shuntCurrent()
{
    return (read16(I_SHUNT_R) * current_lsb);
}

// returns the bus power in watts
float INA219::busPower()
{
    return (read16(P_BUS_R) * power_lsb);
}


/**********************************************************************
* 			INTERNAL I2C FUNCTIONS			      *
**********************************************************************/

// writes a 16-bit word (d) to register pointer (a)
// when selecting a register pointer to read from, (d) = 0
void INA219::write16(uint8_t a, uint16_t d) {
    uint8_t temp;
    temp = (uint8_t)d;
    d >>= 8;
    Wire.beginTransmission(i2c_address); // start transmission to device

    #if ARDUINO >= 100
        Wire.write(a); // sends register address to read from
        Wire.write((uint8_t)d);  // write data hibyte 
        Wire.write(temp); // write data lobyte;
    #else
        Wire.send(a); // sends register address to read from
        Wire.send((uint8_t)d);  // write data hibyte 
        Wire.send(temp); // write data lobyte;
    #endif

    Wire.endTransmission(); // end transmission
    delay(1);
}


int16_t INA219::read16(uint8_t dir) {
    uint16_t ret;

    // move the pointer to reg. of interest, null argument
    write16(dir, 0);
    
    Wire.requestFrom((int)i2c_address, 2);	// request 2 data bytes

    #if ARDUINO >= 100
        ret = Wire.read(); // rx hi byte
        ret <<= 8;
        ret |= Wire.read(); // rx lo byte
    #else
        ret = Wire.receive(); // rx hi byte
        ret <<= 8;
        ret |= Wire.receive(); // rx lo byte
    #endif

    Wire.endTransmission(); // end transmission

    return ret;
}
