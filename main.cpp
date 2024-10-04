#include "MicroBit.h"
#include <cstdio>

#define RX_PIN 6

#define SCL_PIN 8
#define SDA_PIN 16

#define ENABLED_ENABLED 6
#define ENABLED_DISABLED 1

#define TXD_ADDRESS 0x51C

#define ACCELEROMETER_ADDRESS 0x19
#define CTRL_REG1 0x60

#define OUT_X_L_A 0x28 
#define OUT_X_H_A 0x29 
#define OUT_Y_L_A 0x2A 
#define OUT_Y_H_A 0x2B 
#define OUT_Z_L_A 0x2C 
#define OUT_Z_H_A 0x2D 

/**
 * @brief Creates delay
 * Uses busy loop that uses parameter as number of cycles
 * @param us represents microseconds of delay
 */
void delay(volatile uint32_t us) {
    for (volatile uint32_t i = 0; i < us; ++i);
}

/**
 * @brief Serial port bit bang implementation
 * Sets the DIR and OUT registers
 * Configures rx pin as an input using DIR register
 * Sets rx channel low to represent start bit
 * Loops through each character in input string
 * Loops through each bit of the character
 * If bit = 1, sets rx channel high
 * If bit - 0, sets rx channel low
 * Delays for roughly 4us to create 1152000 baud rate
 * Once done, sets rx channel high to represent stop bit
 * @param str 
 */
void bitBangSerial(char *str) {
    volatile uint32_t *d = (uint32_t *)0x50000514; //DIR
    volatile uint32_t *p = (uint32_t *)0x50000504; //OUT

    *d |= (1UL << RX_PIN); 
    *p &= ~(1UL << RX_PIN); // start bit

    while (*str != '\0') {
        char currentChar = *str++;
        for (int i = 0; i < 8; ++i) {

            if (currentChar & (1 << i)) {
                *p |= (1UL << RX_PIN); // sets 
            } else {
                *p &= ~(1UL << RX_PIN); //clears
            }

            currentChar >>= 1;
            delay(64);
        }
    }
    *d |= (1UL << RX_PIN); // stop bit
}

/**
 * @brief Clears various i2c status flags 
 */
void reset_i2c_flags() {
    NRF_TWI0->EVENTS_STOPPED = NRF_TWI0->EVENTS_RXDREADY = NRF_TWI0->EVENTS_TXDSENT =
        NRF_TWI0->EVENTS_ERROR = NRF_TWI0->EVENTS_BB = NRF_TWI0->EVENTS_SUSPENDED =
            NRF_TWI0->ERRORSRC = 0;
}

/**
 * @brief Configures the TWI peripheral
 * Disables TWI peripheral
 * Configures SCL and SDA pins
 * Sets the accelerometer address
 * Sets i2c bus frequency
 * Creates shortcut between local events and tasks
 * resets i2c status flags
 * Enables TWI peripheral
 */
void configureTWI() {
    NRF_TWI0->ENABLE = ENABLED_DISABLED;
    NRF_P0->PIN_CNF[SCL_PIN] = NRF_P0->PIN_CNF[SDA_PIN] = 0x00000600;
    NRF_TWI0->PSEL.SCL = SCL_PIN;
    NRF_TWI0->PSEL.SDA = SDA_PIN;
    NRF_TWI0->ADDRESS = ACCELEROMETER_ADDRESS;
    NRF_TWI0->FREQUENCY = NRF_TWIM_FREQ_250K; //250kbps
    NRF_TWI0->SHORTS = 1;
    reset_i2c_flags();
    NRF_TWI0->ENABLE = ENABLED_ENABLED;
}

/**
 * @brief Configures the accelerometer's control register to allow normal power mode 
 * and non-zero data rate
 * Sets the TxD register to the control registers address
 * Starts i2c transmission
 * Waits until 1st byte of data is sent
 * Waits until 2nd byte is sent
 * Repeats write sequence for the new value for the control reg
 * Stops the task
 */
void configureAccelerometer() {
    NRF_TWI0->TXD = CTRL_REG1;
    NRF_TWI0->TASKS_STARTTX = 1;  
    while (!NRF_TWI0->EVENTS_TXDSENT);  
    NRF_TWI0->EVENTS_TXDSENT = 0;

    NRF_TWI0->TXD = 0x27;
    NRF_TWI0->TASKS_RESUME = 1;
    while (!NRF_TWI0->EVENTS_TXDSENT);  
    NRF_TWI0->EVENTS_TXDSENT = 0;

    NRF_TWI0->TASKS_RESUME = NRF_TWI0->TASKS_STOP = 1;  
}

/**
 * @brief Reads a byte of data from the give register
 * Sets TXD to reg address
 * Starts the task
 * Waits until the twi is ready to receive data
 * Reads the data
 * Ends the task
 * @param registerAddress the reg to read from
 * @return uint8_t the byte read
 */
uint8_t readByteFromReg(uint8_t registerAddress) {
    NRF_TWI0->TXD = registerAddress;
    NRF_TWI0->TASKS_STARTTX = 1; 
    NRF_TWI0->TASKS_RESUME = 1;
    while (!NRF_TWI0->EVENTS_RXDREADY);
    NRF_TWI0->EVENTS_RXDREADY = 0;
    uint8_t data = NRF_TWI0->RXD;
    NRF_TWI0->TASKS_RESUME = NRF_TWI0->TASKS_STOP = 1;
    return data;
}

/**
 * @brief 
 * Reads the high byte and low byte for each axis,
 * and combines them into a uint16_t
 * Formats and prints the data
 */
void readAccelerometerData() {
    uint16_t x = (int16_t)((readByteFromReg(OUT_X_H_A) << 8) | readByteFromReg(OUT_X_L_A));

    uint16_t y = (int16_t)((readByteFromReg(OUT_Y_H_A) << 8) | readByteFromReg(OUT_Y_L_A));

    uint16_t z = (int16_t)((readByteFromReg(OUT_Z_H_A) << 8) | readByteFromReg(OUT_Z_L_A)); 

    printf("[X: %d] [Y: %d] [Z: %d]\r\n", x, y, z);
}

/**
 * @brief 
 * Configures the twi and accelerometer to read data
 * Transmits a string via our ttl serial transmitter
 *  Continuously reads data from the accelerometer, updating every 5 seconds
 */
void showAccelerometerSample() {
    configureTWI();
    configureAccelerometer();

    char msg[] = "hello";
    bitBangSerial(msg);

    while (1) {
        readAccelerometerData();
        delay(5000000); 
    }
}