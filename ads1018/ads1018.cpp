/**
*  Arduino Library for Texas Instruments ADS1018 - 12-Bit Analog-to-Digital Converter with 
*  Internal Reference and Temperature Sensor
*  
*  @author Vishnu Easwaran E <easwaranvishnu@gmail.com>
*  derived from the work of Alvaro Salazar <alvaro@denkitronik.com>
*
*/

/**
 * The MIT License
 *
 * Copyright 2018 Vishnu Easwaran E <easwaranvishnu@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "ads1018.h"
#include "Arduino.h"

// #define DEBUG_GETTEMPERATURE  //Debug getTemperature() method

#ifdef DEBUG_BEGIN
    #define DEBUG_BEGIN(x) decodeConfigRegister(x)
#else
    #define DEBUG_BEGIN(x)
#endif

#ifdef DEBUG_GETADCVALUE
    #define DEBUG_GETADCVALUE(x) decodeConfigRegister(x)
#else
    #define DEBUG_GETADCVALUE(x)
#endif

#ifdef DEBUG_GETTEMPERATURE
    #define DEBUG_GETTEMPERATURE(x) decodeConfigRegister(x)
#else
    #define DEBUG_GETTEMPERATURE(x) decodeConfigRegister(x)
#endif

/*
 * Constructor of the class
 * @param io_pin_cs a byte indicating the pin to be use as the chip select pin (CS)
 */
ADS1018::ADS1018(uint8_t io_pin_cs) {
    cs = io_pin_cs;
}

/*
 * This method initialize the SPI port and the config register
 */
void ADS1018::begin() {
    pinMode(cs, OUTPUT);
	digitalWrite(cs, HIGH);
    SPI.begin();
    SPI.beginTransaction(SPISettings(SCLK, MSBFIRST, SPI_MODE1));
	configRegister.bits={RESERVED, VALID_CFG, PULLUP, TEMP_MODE, RATE_3300SPS, CONTINUOUS, FSR_4096, AIN_0, START_NOW}; //Default values
    DEBUG_BEGIN(configRegister); 										//Debug this method: print the config register in the Serial port
}

/*
 * Getting the temperature in degrees celsius from the internal sensor of the ADS1018
 * @return A double (32bits) containing the temperature in degrees celsius of the internal sensor
 */
double ADS1018::getTemperature() {
    uint16_t convRegister;
    uint8_t  dataMSB, dataLSB, configMSB, configLSB, count=0;
	if(lastSensorMode==TEMP_MODE)
		count=1;  									//Lucky you! We don't have to read twice the sensor
	else
		configRegister.bits.sensorMode=TEMP_MODE;	//Sorry but we will have to read twice the sensor
    do{
		digitalWrite(cs, LOW);
		delayMicroseconds(10);
		dataMSB = SPI.transfer(configRegister.byte.msb);
		dataLSB = SPI.transfer(configRegister.byte.lsb);
		configMSB = SPI.transfer(configRegister.byte.msb);
		configLSB = SPI.transfer(configRegister.byte.lsb);
		digitalWrite(cs, HIGH);
		delayMicroseconds(10);  
		count++;
	}while (count<=1);	//We make two readings because the second reading is the temperature.
    
	// DEBUG_GETTEMPERATURE(configRegister);	//Debug this method: print the config register in the Serial port

	convRegister = (((dataMSB)<<8 | (dataLSB)) >> 4);	//Moving MSB and LSB to 16 bit and making it right-justified; 4 because 12bit value
	convRegister &= 0x0FFF; 							//Making sure first 4 bits are 0

    if((convRegister) >= 0x0800){
		convRegister=((~convRegister)+1 & 0x0fff);	//Applying binary twos complement format
        return (double)(convRegister*0.125*-1);
    }
    return (double)convRegister*0.125;
}

/*
 * Setting the sampling rate specified in the config register
 * @param samplingRate It's the sampling rate: RATE_8SPS, RATE_16SPS, RATE_32SPS, RATE_64SPS, RATE_128SPS, RATE_250SPS, RATE_475SPS, RATE_860SPS
 */
void ADS1018::setSamplingRate(uint8_t samplingRate){
	configRegister.bits.rate=samplingRate;
}

/*
 * Setting the full scale range in the config register
 * @param fsr The full scale range: FSR_6144 (±6.144V)*, FSR_4096(±4.096V)*, FSR_2048(±2.048V), FSR_1024(±1.024V), FSR_0512(±0.512V), FSR_0256(±0.256V). (*) No more than VDD + 0.3 V must be applied to this device.
 */
void ADS1018::setFullScaleRange(uint8_t fsr){
	configRegister.bits.pga=fsr;
}

/*
 * Setting the inputs to be adquired in the config register. 
 * @param input The input selected: Diferential inputs: DIFF_0_1, DIFF_0_3, DIFF_1_3, DIFF_2_3. Single ended input: AIN_0, AIN_1, AIN_2, AIN_3
 */
void ADS1018::setInputSelected(uint8_t input){
	configRegister.bits.mux=input;
}

/*
 * Setting to continuous adquisition mode
 */
void ADS1018::setContinuousMode(){
	configRegister.bits.operatingMode=CONTINUOUS;
}

/*
 * Setting to single shot adquisition and power down mode
 */
void ADS1018::setSingleShotMode(){
	configRegister.bits.operatingMode=SINGLE_SHOT;
}

/*
 * Disabling the internal pull-up resistor of the DOUT pin
 */
void ADS1018::disablePullup(){
	configRegister.bits.operatingMode=NO_PULLUP;
}

/*
 * Enabling the internal pull-up resistor of the DOUT pin
 */
void ADS1018::enablePullup(){
	configRegister.bits.operatingMode=PULLUP;
}

/*
 * Decoding a configRegister structure and then print it out to the Serial port
 * @param configRegister The config register in "union Config" format
 */
void ADS1018::decodeConfigRegister(union Config configRegister){
	String message=String();
	switch(configRegister.bits.singleStart){
		case 0: message="NOINI"; break;
		case 1: message="START"; break;
	}
    message+=" ";
	switch(configRegister.bits.mux){
		case 0: message+="A0-A1"; break;
	    case 1: message+="A0-A3"; break;
		case 2: message+="A1-A3"; break;
		case 3: message+="A2-A3"; break;
		case 4: message+="A0-GD"; break;
		case 5: message+="A1-GD"; break;
		case 6: message+="A2-GD"; break;
		case 7: message+="A3-GD"; break;
		}
    message+=" ";
	switch(configRegister.bits.pga){
		case 0: message+="6.144"; break;
	    	case 1: message+="4.096"; break;
		case 2: message+="2.048"; break;
		case 3: message+="1.024"; break;
		case 4: message+="0.512"; break;
		case 5: message+="0.256"; break;
		case 6: message+="0.256"; break;
		case 7: message+="0.256"; break;
		}
    message+=" ";		
	switch(configRegister.bits.operatingMode){
		case 0: message+="CONT."; break;
	   	case 1: message+="SSHOT"; break;
	}
    message+=" ";		
	switch(configRegister.bits.rate){
		case 0: message+="128SPS"; break;
	    case 1: message+="250SPS"; break;
		case 2: message+="490SPS"; break;
		case 3: message+="920SPS"; break;
		case 4: message+="1600SP"; break;
		case 5: message+="2400SP"; break;
		case 6: message+="3300SP"; break;
	}
    message+=" ";		
	switch(configRegister.bits.sensorMode){
		case 0: message+="ADC_M"; break;
	    case 1: message+="TMP_M"; break;
	}
    message+=" ";		
	switch(configRegister.bits.pullUp){
		case 0: message+="DISAB"; break;
	    case 1: message+="ENABL"; break;
	}
    message+=" ";		
	switch(configRegister.bits.noOperation){
		case 0: message+="INVAL"; break;
	    case 1: message+="VALID"; break;
		case 2: message+="INVAL"; break;
		case 3: message+="INVAL"; break;
	}
    message+=" ";		
	switch(configRegister.bits.reserved){
		case 0: message+="RSRV0"; break;
	    case 1: message+="RSRV1"; break;
	}	
	Serial.println("\nSTART MXSEL PGASL MODES RATES  ADTMP PLLUP NOOPE RESER");
	Serial.println(message);
}