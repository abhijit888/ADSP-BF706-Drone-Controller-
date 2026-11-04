# ADSP-BF706-Drone-Controller-
Drone control system using dual ADSP-BF706 processors with UART communication, joystick input processing and four-channel PWM output for flight controller interfacing.

**Successfully developed code on the BF706 board to generate the required frequency and duty-cycle control signals for all four drone movements: Throttle, Yaw, Pitch, and Roll, without using the original drone transmitter. 
Generated and fed these control signals directly to the drone receiver, enabling the KK2.1 flight controller to receive and process the control inputs.**

**Transmitter Code-**

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/platform.h>
#include <cdefBF706.h>

// Driver includes
#include <drivers/twi/adi_twi.h>
#include <drivers/uart/adi_uart.h>

// ==========================================
// Definitions & Handles
// ==========================================
#define ADS1115_ADDRESS         0x48
#define ADS1115_REG_CONVERT     0x00
#define ADS1115_REG_CONFIG      0x01
#define TWI_DEV_NUM             0
#define TWI_MEM_SIZE            ADI_TWI_MEMORY_SIZE

#define UART_DEV_NUM            0
#define UART_MEM_SIZE           ADI_UART_BIDIR_INT_MEMORY_SIZE

static uint8_t TwiDriverMemory[TWI_MEM_SIZE];
static ADI_TWI_HANDLE hTwiDevice;

static uint8_t UartDriverMemory[UART_MEM_SIZE];
static ADI_UART_HANDLE hUartDevice;

extern int32_t adi_initComponents(void);

// Hardware Center baselines for physical joysticks
static const float BASE_CENTERS[4] = {15450.0f, 4500.0f, 12620.0f, 16050.0f};

// Hardware setup
void Setup_Pin_Mux(void) {
    *pREG_PORTB_FER_SET = (1 << 8);
    *pREG_PORTB_MUX &= ~(3 << (8 * 2));
}

bool Init_I2C(void) {
    if (adi_twi_Open(TWI_DEV_NUM, ADI_TWI_MASTER, TwiDriverMemory, TWI_MEM_SIZE, &hTwiDevice) != ADI_TWI_SUCCESS)
        return false;

    adi_twi_SetBitRate(hTwiDevice, 100);
    adi_twi_SetHardwareAddress(hTwiDevice, ADS1115_ADDRESS);
    return true;
}

bool Init_UART(void) {
    if (adi_uart_Open(UART_DEV_NUM, ADI_UART_DIR_TRANSMIT, UartDriverMemory, UART_MEM_SIZE, &hUartDevice) != ADI_UART_SUCCESS)
        return false;

    adi_uart_SetMode(hUartDevice, ADI_UART_MODE_UART);
    adi_uart_SetBaudRate(hUartDevice, 115200);
    adi_uart_SetWordLen(hUartDevice, ADI_UART_WORDLEN_8BITS);
    adi_uart_SetNumStopBits(hUartDevice, ADI_UART_ONE_STOPBIT);
    adi_uart_EnableTx(hUartDevice, true);
    return true;
}

void ADS1115_WriteRegister(uint8_t reg, uint16_t value) {
    uint8_t txBuffer[3];
    txBuffer[0] = reg;
    txBuffer[1] = (uint8_t)(value >> 8);
    txBuffer[2] = (uint8_t)(value & 0xFF);
    adi_twi_Write(hTwiDevice, txBuffer, 3, false);
}

uint16_t ADS1115_ReadRegister(uint8_t reg) {
    uint8_t rxBuffer[2] = {0, 0};
    adi_twi_Write(hTwiDevice, &reg, 1, true);
    adi_twi_Read(hTwiDevice, rxBuffer, 2, false);
    return (uint16_t)((rxBuffer[0] << 8) | rxBuffer[1]);
}

int16_t Read_Joystick_Raw(uint8_t channel) {
    uint16_t config = 0x81E3;

    switch (channel) {
        case 0: config |= (0x4 << 12); break; // A0
        case 1: config |= (0x5 << 12); break; // A1
        case 2: config |= (0x6 << 12); break; // A2
        case 3: config |= (0x7 << 12); break; // A3
        default: return 0;
    }

    ADS1115_WriteRegister(ADS1115_REG_CONFIG, config);

    uint16_t status = 0;
    uint32_t timeout = 0;
    do {
        status = ADS1115_ReadRegister(ADS1115_REG_CONFIG);
        timeout++;
        if (timeout > 100000) break;
    } while ((status & 0x8000) == 0);

    return (int16_t)ADS1115_ReadRegister(ADS1115_REG_CONVERT);
}

// Maps physical ADC readings into explicit custom ranges
uint16_t Normalize_Joystick(uint8_t channel, int16_t raw_val) {
    float center = BASE_CENTERS[channel];
    float val = (float)raw_val;

    // Range offsets
    float base_offset = 1.0f + (channel * 1000.0f);   // 1, 1001, 2001, 3001
    float center_val  = 500.0f + (channel * 1000.0f); // 500, 1500, 2500, 3500
    float max_val     = 1000.0f + (channel * 1000.0f); // 1000, 2000, 3000, 4000

    // Threshold check (+/- 100 counts around raw hardware center)
    if (val >= (center - 100.0f) && val <= (center + 100.0f)) {
        return (uint16_t)center_val;
    }

    float norm = center_val;
    if (val < center) {
        float ratio = val / center;
        norm = base_offset + (ratio * (center_val - base_offset));
    } else {
        float ratio = (val - center) / (32767.0f - center);
        norm = center_val + (ratio * (max_val - center_val));
    }

    if (norm < base_offset) norm = base_offset;
    if (norm > max_val) norm = max_val;

    return (uint16_t)norm;
}

int main(void) {
    uint16_t norm_joystick[4] = {0};
    uint8_t packet[12];

    if (adi_initComponents() != 0) return 1;

    Setup_Pin_Mux();

    if (!Init_I2C() || !Init_UART()) {
        printf("TX Board: Init Failed!\n");
        return 1;
    }

    printf("TX Board: Fixed Range Engine Running...\n");

    while (1) {
        for (uint8_t i = 0; i < 4; i++) {
            int16_t raw = Read_Joystick_Raw(i);
            norm_joystick[i] = Normalize_Joystick(i, raw);
        }

        packet[0] = 0xAA;
        packet[1] = 0x55;

        uint8_t checksum = 0;
        for (int i = 0; i < 4; i++) {
            uint8_t msb = (uint8_t)((norm_joystick[i] >> 8) & 0xFF);
            uint8_t lsb = (uint8_t)(norm_joystick[i] & 0xFF);

            packet[2 + (i * 2)] = msb;
            packet[3 + (i * 2)] = lsb;

            checksum ^= msb ^ lsb;
        }
        packet[10] = checksum;
        packet[11] = 0x0D;

        adi_uart_Write(hUartDevice, packet, 12);

        printf("TX Range Data -> J1: %4d | J2: %4d | J3: %4d | J4: %4d\n",
               norm_joystick[0], norm_joystick[1], norm_joystick[2], norm_joystick[3]);

        for (volatile int delay = 0; delay < 300000; delay++);
    }

    return 0;
}

**Receiver Code-**

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/platform.h>
#include <cdefBF706.h>

// Driver includes
#include <drivers/uart/adi_uart.h>
#include <services/tmr/adi_tmr.h>

// ==========================================
// Timing & PWM Definitions
// ==========================================
#define UART_DEV_NUM            0
#define UART_MEM_SIZE           ADI_UART_BIDIR_INT_MEMORY_SIZE

#ifndef ADI_TMR_MEMORY_SIZE
#define ADI_TMR_MEMORY_SIZE     (128u)
#endif

#define SCLK0_FREQ              100000000
#define PWM_FREQ_HZ             50                              // 50 Hz PWM
#define PERIOD                  (SCLK0_FREQ / PWM_FREQ_HZ)       // 2,000,000 clock cycles

#define PULSE_MIN_DUTY          (uint32_t)(PERIOD * 0.05f)      // 5% Duty Cycle  (1.0 ms)
#define PULSE_CENTER_DUTY       (uint32_t)(PERIOD * 0.075f)     // 7.5% Duty Cycle (1.5 ms)
#define PULSE_MAX_DUTY          (uint32_t)(PERIOD * 0.10f)      // 10% Duty Cycle (2.0 ms)

static uint8_t UartDriverMemory[UART_MEM_SIZE];
static ADI_UART_HANDLE hUartDevice;

// Hardware Pin mapping: J1(Yaw)->PA05(TMR0), J2(Throttle)->PA06(TMR1), J3(Pitch)->PA07(TMR2), J4(Roll)->PA11(TMR6)
static const uint32_t TimerIDs[4] = {0, 1, 2, 6};
static ADI_TMR_HANDLE hTimers[4];
static uint8_t TimerMemory[4][ADI_TMR_MEMORY_SIZE];

extern int32_t adi_initComponents(void);

// Pin Mux Initialization
void Setup_Pin_Mux(void) {
    // Enable PB09 as UART0 RX
    *pREG_PORTB_FER_SET = (1 << 9);
    *pREG_PORTB_MUX &= ~(3 << (9 * 2)); 

    // Enable Peripheral Function for PA05, PA06, PA07, PA11
    *pREG_PORTA_FER_SET = (1 << 5) | (1 << 6) | (1 << 7) | (1 << 11);

    // PA05 (TMR0), PA06 (TMR1), PA07 (TMR2) use MUX Function 0
    *pREG_PORTA_MUX &= ~((3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)));

    // PA11 (TMR6) uses MUX Function 1
    *pREG_PORTA_MUX &= ~(3 << (11 * 2));
    *pREG_PORTA_MUX |=  (1 << (11 * 2));
}

bool Init_UART(void) {
    if (adi_uart_Open(UART_DEV_NUM, ADI_UART_DIR_RECEIVE, UartDriverMemory, UART_MEM_SIZE, &hUartDevice) != ADI_UART_SUCCESS)
        return false;

    adi_uart_SetMode(hUartDevice, ADI_UART_MODE_UART);
    adi_uart_SetBaudRate(hUartDevice, 115200);
    adi_uart_SetWordLen(hUartDevice, ADI_UART_WORDLEN_8BITS);
    adi_uart_SetNumStopBits(hUartDevice, ADI_UART_ONE_STOPBIT);
    adi_uart_EnableRx(hUartDevice, true);

    return true;
}

// Custom Duty Mapping Function
uint32_t Process_Channel_Duty(uint8_t channel, uint16_t val, float *duty_pct) {
    float fval = (float)val;
    float min_val, center_min, center_max, max_val;

    switch (channel) {
        case 0: // J1: Yaw
            min_val    = 38.0f;
            center_min = 156.0f;
            center_max = 166.0f;
            max_val    = 238.0f;

            if (fval >= center_min && fval <= center_max) {
                *duty_pct = 7.5f;
                return PULSE_CENTER_DUTY;
            }
            else if (fval < center_min) {
                if (fval <= min_val) {
                    *duty_pct = 10.0f;
                    return PULSE_MAX_DUTY;
                }
                float ratio = (center_min - fval) / (center_min - min_val);
                *duty_pct = 7.5f + (ratio * 2.5f);
                return (uint32_t)(PULSE_CENTER_DUTY + (ratio * (PULSE_MAX_DUTY - PULSE_CENTER_DUTY)));
            }
            else {
                if (fval >= max_val) {
                    *duty_pct = 5.0f;
                    return PULSE_MIN_DUTY;
                }
                float ratio = (fval - center_max) / (max_val - center_max);
                *duty_pct = 7.5f - (ratio * 2.5f);
                return (uint32_t)(PULSE_CENTER_DUTY - (ratio * (PULSE_CENTER_DUTY - PULSE_MIN_DUTY)));
            }

        case 1: // J2: Throttle
            min_val = 1294.0f;
            max_val = 1538.0f;
            
            if (fval <= min_val) {
                *duty_pct = 5.0f;
                return PULSE_MIN_DUTY;
            }
            if (fval >= max_val) {
                *duty_pct = 10.0f;
                return PULSE_MAX_DUTY;
            }
            {
                float ratio = (fval - min_val) / (max_val - min_val);
                *duty_pct = 5.0f + (ratio * 5.0f);
                return (uint32_t)(PULSE_MIN_DUTY + (ratio * (PULSE_MAX_DUTY - PULSE_MIN_DUTY)));
            }

        case 2: // J3: Pitch (Log: 2152 to 2291, Center: 2200 +/- 5 => 2195 to 2205) - FLIPPED DIRECTION
            min_val    = 2152.0f;
            center_min = 2195.0f;
            center_max = 2205.0f;
            max_val    = 2291.0f;

            // 1. Idle Deadzone (+/- 5 around baseline)
            if (fval >= center_min && fval <= center_max) {
                *duty_pct = 7.5f;
                return PULSE_CENTER_DUTY;
            }
            // 2. Moving Backward (Pulled Back) -> Scales UP to 10% Duty Cycle
            else if (fval < center_min) {
                if (fval <= min_val) {
                    *duty_pct = 10.0f;
                    return PULSE_MAX_DUTY;
                }
                float ratio = (center_min - fval) / (center_min - min_val);
                *duty_pct = 7.5f + (ratio * 2.5f);
                return (uint32_t)(PULSE_CENTER_DUTY + (ratio * (PULSE_MAX_DUTY - PULSE_CENTER_DUTY)));
            }
            // 3. Moving Forward (Pushed Forward) -> Scales DOWN to 5% Duty Cycle
            else {
                if (fval >= max_val) {
                    *duty_pct = 5.0f;
                    return PULSE_MIN_DUTY;
                }
                float ratio = (fval - center_max) / (max_val - center_max);
                *duty_pct = 7.5f - (ratio * 2.5f);
                return (uint32_t)(PULSE_CENTER_DUTY - (ratio * (PULSE_CENTER_DUTY - PULSE_MIN_DUTY)));
            }

        case 3: // J4: Roll
            min_val    = 3118.0f;
            center_min = 3161.0f;
            center_max = 3171.0f;
            max_val    = 3321.0f;

            if (fval >= center_min && fval <= center_max) {
                *duty_pct = 7.5f;
                return PULSE_CENTER_DUTY;
            }
            else if (fval < center_min) {
                if (fval <= min_val) {
                    *duty_pct = 10.0f;
                    return PULSE_MAX_DUTY;
                }
                float ratio = (center_min - fval) / (center_min - min_val);
                *duty_pct = 7.5f + (ratio * 2.5f);
                return (uint32_t)(PULSE_CENTER_DUTY + (ratio * (PULSE_MAX_DUTY - PULSE_CENTER_DUTY)));
            }
            else {
                if (fval >= max_val) {
                    *duty_pct = 5.0f;
                    return PULSE_MIN_DUTY;
                }
                float ratio = (fval - center_max) / (max_val - center_max);
                *duty_pct = 7.5f - (ratio * 2.5f);
                return (uint32_t)(PULSE_CENTER_DUTY - (ratio * (PULSE_CENTER_DUTY - PULSE_MIN_DUTY)));
            }

        default:
            *duty_pct = 7.5f;
            return PULSE_CENTER_DUTY;
    }
}

int main(void) {
    uint8_t byte1 = 0, byte2 = 0;
    uint8_t rx_data[10];
    uint16_t norm_joystick[4] = {0};
    float duty_cycles[4] = {7.5f, 5.0f, 7.5f, 7.5f};

    if (adi_initComponents() != 0) return 1;

    Setup_Pin_Mux();

    if (!Init_UART()) {
        printf("RX Board: UART Init Failed!\n");
        return 1;
    }

    // Initialize PWM Timers (PA05, PA06, PA07, PA11) at 50 Hz
    for (int i = 0; i < 4; i++) {
        adi_tmr_Open(TimerIDs[i], TimerMemory[i], ADI_TMR_MEMORY_SIZE, NULL, NULL, &hTimers[i]);
        adi_tmr_SetMode(hTimers[i], ADI_TMR_MODE_CONTINUOUS_PWMOUT);
        adi_tmr_SetPeriod(hTimers[i], PERIOD);
        adi_tmr_SetWidth(hTimers[i], PERIOD - ((i == 1) ? PULSE_MIN_DUTY : PULSE_CENTER_DUTY));
        adi_tmr_Enable(hTimers[i], true);
    }

    printf("RX Board Active: Exact 50 Hz PWM Engine Running...\n");

    while (1) {
        adi_uart_Read(hUartDevice, &byte1, 1);

        if (byte1 == 0xAA) {
            adi_uart_Read(hUartDevice, &byte2, 1);

            if (byte2 == 0x55) {
                adi_uart_Read(hUartDevice, rx_data, 10);

                if (rx_data[9] == 0x0D) {
                    uint8_t calc_checksum = 0;
                    for (int i = 0; i < 8; i++) {
                        calc_checksum ^= rx_data[i];
                    }

                    if (calc_checksum == rx_data[8]) {
                        for (int ch = 0; ch < 4; ch++) {
                            norm_joystick[ch] = (uint16_t)((rx_data[ch * 2] << 8) | rx_data[(ch * 2) + 1]);

                            uint32_t pulse_width = Process_Channel_Duty(ch, norm_joystick[ch], &duty_cycles[ch]);

                            // Invert high pulse for positive active width setting
                            adi_tmr_SetWidth(hTimers[ch], (PERIOD - pulse_width));
                        }

                        /*printf("RX Output (50Hz) -> J1(Yaw): %4.1f%% | J2(Thr): %4.1f%% | J3(Pit): %4.1f%% | J4(Rol): %4.1f%%\n",
                               duty_cycles[0], duty_cycles[1], duty_cycles[2], duty_cycles[3]);*/
                    }
                }
            }
        }
    }

    return 0;
}
