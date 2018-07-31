#include "Arduino.h"
#include <stdint.h>
#include <TinyWireS.h>
#include "wiring_private.h"

/*
 * ArduCopter Parameters
 * RNGFND_TYPE = 7 (LightWareI2C)
 * RNGFND_ADDR = 102 (I2C Address of lidar in decimal).
 * RNGFND_SCALING = 1
 * RNGFND_MIN_CM = 10
 * RNGFND_MAX_CM = 80
 * RNGFND_GNDCLEAR = 10 (Distance between detector and ground in cm)
 */

/*
 * Pinout:
 *     -------
 * RST |o   8| +5V
 * A3  |2   7| SCL
 * A2  |3   6| PWM
 * GND |4   5| SDA
 *     -------
 * WARNING: Don't forget the pullup resistors!
 */

/*
 * I2C Port 0:
 *  Pin 19 (A5) SCL0
 *  Pin 18 (A4) SDA0
 *
 * Write Address: 0x66
 * Read Address:  0x67
 */
#define I2C_SLAVE_ADDRESS       0x66 // Between 0x01 and 0x7F
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

/*
 * GP2Y0A21YK0F Sensors:
 *  10-80 cm distance
 *  Max 20Hz measure rate
 *
 * Equations:
 *  https://acroname.com/blog/linearizing-sharp-ranger-data
 *  V = 1/(R+4.0)
 *  V/5 = val/1024
 */
// Returns valid measurement data or infinity (0xFF)
#define GET_DISTANCE(x)  ((x > 80) ? (CONVERT_I_2_D(x)) : (0xFF))
// Takes 10-bit value, and converts it to distance in cm
#define CONVERT_I_2_D(x) ((uint8_t) (6787/(x-3)-4))
// Milliseconds per blink (10Hz)
#define SENSOR_UPDATE_MS    100

/*
 * Sensor pinout
 */
#define SENSOR_PIN          0x03 // PB3 -> A3 -> Pin 2
#define LED_PIN             0x04 // PB4 -> A2 -> Pin 3

/*
 * Function declarations
 */
void requestEvent(void);
void receiveEvent(uint8_t bytesReceived);
void update(void);
void StartADCConversion(void);
uint8_t IsADCBusy(void);
uint16_t GetADCValue(void);

/*
 * Global variables
 */
uint8_t         ledStatus;
uint16_t        voltage;
volatile uint8_t distance;

void StartADCConversion(void) {
    // Set the analog reference and select the channel
	ADMUX = SENSOR_PIN;
    // Start the conversion
    sbi(ADCSRA, ADSC);
}

uint8_t IsADCBusy(void) {
    // ADSC is cleared when the conversion finishes
    return bit_is_set(ADCSRA, ADSC);
}

uint16_t GetADCValue(void) {
    uint8_t low, high;
    low = ADCL;
    high = ADCH;
    return (high << 8) | low;
}

void update(void) {
    voltage = GetADCValue();
    distance = GET_DISTANCE(voltage);
}

// Send register map on each request
void requestEvent() {
    TinyWireS.send(distance);
}

void receiveEvent(uint8_t bytesReceived) {
    while (bytesReceived > 0) {
        TinyWireS.receive();
        bytesReceived--;
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    ledStatus = 0;

    // Initialize I2C slave protocol
    TinyWireS.begin(I2C_SLAVE_ADDRESS);
    TinyWireS.onRequest(requestEvent);
    TinyWireS.onReceive(receiveEvent);

    StartADCConversion();
}

void loop() {
    TinyWireS_stop_check();
    if (!IsADCBusy()) {
        update();
        tws_delay(SENSOR_UPDATE_MS);
        StartADCConversion();

        if (ledStatus) {
            ledStatus = 0;
            digitalWrite(LED_PIN, LOW);
        } else {
            ledStatus = 1;
            digitalWrite(LED_PIN, HIGH);
        }
    }
}
