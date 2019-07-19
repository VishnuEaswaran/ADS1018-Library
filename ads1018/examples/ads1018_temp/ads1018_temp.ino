#include <ads1018.h>

/**
*  Basic Example for temeperature sensing using Arduino Library for Texas Instruments ads1018 - a 16-Bit Analog-to-Digital Converter with 
*  Internal Reference and Temperature Sensor
*  
*/

//Definition of the Arduino pin to be used as the chip select pin (SPI CS pin). Example: pin 5
#define CS 7

//Creating an ADS1018 object (object's name is ads1018)
ADS1018 ads1018(CS);

void setup(){
    
    Serial.begin(9600);
    ads1018.begin(); //Initialize the ads1018. Default setting: PULLUP RESISTOR, ADC MODE, RATE 3300SPS, SINGLE SHOT, ±0.4096V, DIFFERENTIAL AIN0-AIN1

}

void loop(){
    
    Serial.println(String(ads1018.getTemperature(),5)+" C"); //Getting temperature of the internal sensor
    delay(200); //You can use a delay to save power. The ads1018 will be in power down state during all the delay time. (Optional)
}
