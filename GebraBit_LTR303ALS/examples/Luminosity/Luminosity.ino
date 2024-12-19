#include "GebraBit_LTR303ALS.h"

GebraBit_LTR303ALS LTR303ALS;

void setup() {
    Wire.begin();           // Initialize the I2C bus
    Serial.begin(9600);     // Initialize serial communication for debugging

    GB_LTR303ALS_initialize(&LTR303ALS); // Initialize the HTU2XD sensor
    GB_LTR303ALS_Configuration(&LTR303ALS); // Configure the HTU2XD sensor
}

void loop() {
    GB_LTR303ALS_Get_Data(&LTR303ALS); // Read data from the sensor

    Serial.print("Lux: ");
    Serial.print(LTR303ALS.ALS_LUX);
    Serial.println(" lx");

    delay(2000); // Delay between readings
}
