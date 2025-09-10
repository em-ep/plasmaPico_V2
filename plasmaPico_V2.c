#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/sem.h"
#include "pico/time.h"
#include "pico/malloc.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "tusb.h"

#include "pinsToggle.pio.h"
#include "debug.h"


// Declare System Variables
// Pin "Addresses"
uint32_t S4 = 0x00000008; // 1000
uint32_t S3 = 0x00000004; // 0100
uint32_t S2 = 0x00000002; // 0010
uint32_t S1 = 0x00000001; // 0001
static const uint startPin = 10; // S1 output pin. Rest are assigned sequentially (GP10, pin 14)
#define TRIGGER_PIN 2 // (GP2, pin 4)

// Pulse States
uint32_t __not_in_flash("pwm") stop2free, free2stop, free2poss, free2neg, poss2free, neg2free;
uint32_t __not_in_flash("pwm") freeCycle, possCycle, negCycle;
uint32_t __not_in_flash("pwm") nextState, cycleCount;

// Pulse Charicteristics
static volatile uint32_t __not_in_flash("pwm") delay;
static volatile uint32_t __not_in_flash("pwm") target;

// Helps with lower bound on DCP edge case
uint8_t __not_in_flash("pwm") last_state = 0;

semaphore_t pwm_sem;

// The Number of the PIO State Machine
uint __not_in_flash("pwm") sm;


// === PID Controller ===
typedef struct {
    /* Controller Gains */
    float Kp;
    float Ki;
    float Kd;

    /* Output Limits */
    float limMin;
    float limMax;

    /* Integrator Limits */
    float limMinInt;
    float limMaxInt;

    /* Sample Time (s) */
    float T;

    /* Controller "Memory" */
    float integrator;
    float prevError;        // for integrator
    float differentiator;
    float prevMeasurement;  // for differentiator

    /* Controller Output */
    float out;

} pid_controller;


void pid_controller_init(pid_controller *pid) {
    // Clear controller variables
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
}


float __time_critical_func(pid_controller_update)(pid_controller *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;

    // Proportional
    float proportional = pid->Kp * error;

    // Integral
    pid->integrator = pid->Ki * pid->T * (pid->integrator + error);

    // TODO: Add integrator clamping

    // Derivative
    pid->differentiator = error - pid->prevError;

    // Compute output and apply limits
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {pid->out = pid->limMax;}
    else if (pid->out < pid->limMin) {pid->out = pid->limMin;}

    // Store error and measurement for next cycle
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    // Return controller output
    return pid->out;
}


// === Status LED Initialization ===
#define LED_PIN 25 // Onboard LED

void init_led() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

typedef enum {
    STATE_WAITING_USB,    // Blink slow
    STATE_IDLE,           // Double-blink
    STATE_RECEIVING,      // Long blink
    STATE_DATA_READY,     // LED on
    STATE_OUTPUT_SHOT     // Blink fast
} SystemState;

static SystemState current_state = STATE_WAITING_USB;

uint64_t last_led_update = 0;
bool led_on = false;

void update_led() {
    uint64_t now = time_us_64() / 1000; // Current time in milliseconds

    switch (current_state) {
        case STATE_WAITING_USB:
            // Short blink, long pause (100ms on, 900ms off)
            if (now - last_led_update >= (led_on ? 100 : 900)) {
                led_on = !led_on;
                gpio_put(LED_PIN, led_on);
                last_led_update = now;
            }
            break;

        case STATE_RECEIVING:
            // Long blink, short pause (100ms off, 900ms on)
            if (now - last_led_update >= (led_on ? 900 : 100)) {
                led_on = !led_on;
                gpio_put(LED_PIN, led_on);
                last_led_update = now;
            }
            break;
        
        case STATE_DATA_READY:
            gpio_put(LED_PIN, 1); // LED on
            break;

        case STATE_IDLE:
            // Double-blink, pause (100ms on, 100ms off, 100ms on, 700ms off)
            static uint8_t blink_phase = 0;
            if (now - last_led_update >=
                (blink_phase == 0 ? 100 :
                 blink_phase == 1 ? 100 : 
                 blink_phase == 2 ? 100 : 700)) {
                    
                gpio_put(LED_PIN, (blink_phase % 2 == 0));
                blink_phase = (blink_phase + 1) % 4;
                last_led_update = now;
            }
            break;
            

        case STATE_OUTPUT_SHOT:
            // Rapid flicker (50ms on/off)
            if (now - last_led_update >= 50) {
                led_on = !led_on;
                gpio_put(LED_PIN, led_on);
                last_led_update = now;
            }
            break;
    }
}


// === Serial Configuration ===
// Uncomment ONE of these to choose the interface:
#define USE_USB_CDC   // Use USB serial
//#define USE_UART      // Use hardware UART (GPIO pins)

#ifdef USE_UART
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0 // Change depending on GPIO configuration
#define UART_RX_PIN 1 // ^
#endif

// Serial Protocol Constants
#define START_BYTE 0xAA
#define END_BYTE 0xFF
#define MAX_PULSES 16000 // Max pulses per packet - ensure agreement with the transmitter

// Message Bytes
typedef enum {
    MSG_PWM = 0x01,
    MSG_MANUAL = 0x02,
    MSG_CONFIG = 0x03,
} MessageType;
MessageType current_msg_type;

#define MSG_RETURN_RX 0x80
#define MSG_RETURN_PID 0x81
#define MSG_RETURN_ADC 0x82

// Recieved pulse buffer
uint8_t __not_in_flash("pwm") *rx_buffer = NULL;
uint8_t __not_in_flash("pwm") *pulse_buffer = NULL;
uint16_t num_pulses = 0;

// ADC Block
uint8_t* adc_block;

// pid->out Block
uint8_t* pid_block;

// === Hardware-Agnostic Serial Helpers ===
bool serial_connected() {
    #ifdef USE_USB_CDC
    return stdio_usb_connected();
    #elif defined(USE_UART)
    return true; // UART is always "connected"
    #endif
}

void init_serial() {
    #ifdef USE_USB_CDC
    stdio_init_all(); // Initializes USB CDC
    current_state = STATE_WAITING_USB;
    while (!serial_connected()) {
        update_led();
        sleep_ms(10); // Wait for USB connection
    }

    #elif defined(USE_UART)
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    #endif

    LOG_INFO("Serial Connected!");
    current_state = STATE_IDLE; // Connected, now idle
}

int read_serial_byte() {
    #ifdef USE_USB_CDC
    return getchar_timeout_us(0); // Non-blocking USB read
    #elif defined(USE_UART)
    return uart_is_readable(UART_ID) ? uart_getc(UART_ID) : PICO_ERROR_TIMEOUT; // Non-blocking UART read, returns -1 if no data
    #endif
}


void handle_message() {
    /*
    Handles the completed input buffer depending upon the message type as described below:

    MSG_PWM: Loads the pulse data from rx_buffer into its own array
    MSG_MANUAL: Parses and applies manual switch configuration using helper function
    MSG_CONFIG: Parses and sets new configuration
    */

    int switchState[4];

    switch (current_msg_type) {

        case MSG_PWM:
        pulse_buffer = rx_buffer;
        break;

        case MSG_MANUAL:
            // Gets the switch configurations from rx_buffer and applies them to the output pins
            for (int i = 0; i < 5; i++) {
                if (rx_buffer[3+i] <= 1) {
                    gpio_put(startPin + i, rx_buffer[3+i]);
                } else {
                    LOG_ERROR("Manual switch settings must be 0 or 1");
                }
            }
        break;

        case MSG_CONFIG:
            // Not yet implimented
        break;

    
    default:
        LOG_WARN("Unknown message type 0x%02X\n", current_msg_type);
    }
    
    return;
}


void serial_input() {
    /*
    Parses incoming serial data (UART or USB) of the following protocol:
    [START_BYTE][TYPE][LENGTH_MSB][LENGTH_LSB][DATA...][CHECKSUM][END_BYTE]
    
    Start/End: 0xAA/0xFF
    Types: 0x01 (PWM), 0x02 (Manual Mode), 0x03 (Config)
    */
    static enum { WAIT_START, WAIT_LENGTH_MSB, WAIT_LENGTH_LSB, WAIT_TYPE, WAIT_DATA, WAIT_CHECKSUM, WAIT_END } ser_state = WAIT_START;
    
    // For info logging
    const char* ser_state_names[] = {
    [WAIT_START] = "WAIT_START",
    [WAIT_LENGTH_MSB] = "WAIT_LENGTH_MSB",
    [WAIT_LENGTH_LSB] = "WAIT_LENGTH_LSB",
    [WAIT_TYPE] = "WAIT_TYPE",
    [WAIT_DATA] = "WAIT_DATA",
    [WAIT_CHECKSUM] = "WAIT_CHECKSUM",
    [WAIT_END] = "WAIT_END"
    };

    static uint16_t expected_length = 0;
    static uint8_t checksum = 0;
    static uint16_t data_index = 0;

    int byte;
    while ((byte = read_serial_byte()) != PICO_ERROR_TIMEOUT) {
        LOG_DEBUG("ser_state = %s", ser_state_names[ser_state]);

        switch (ser_state) {
            case WAIT_START:
                if (byte == START_BYTE) {
                    ser_state = WAIT_TYPE;

                    current_state = STATE_RECEIVING; // LED indicator data incoming
                }
                break;

            case WAIT_TYPE:
                if (byte == MSG_PWM || byte == MSG_MANUAL || byte == MSG_CONFIG) {
                    current_msg_type = (MessageType)byte;
                    ser_state = WAIT_LENGTH_MSB;
                } else {
                    ser_state = WAIT_START; // Reset on invalid type TODO: determine if we want to throw an error flag TODO: make sure this doesn't break the LED indicator
                }
                break;

            case WAIT_LENGTH_MSB:
                expected_length = (uint16_t)byte << 8;
                ser_state = WAIT_LENGTH_LSB;
                break;

            
            case WAIT_LENGTH_LSB:
                expected_length |= (uint16_t)byte;

                // TODO: make sure this is a clean exit
                if (expected_length >= MAX_PULSES){
                    LOG_WARN("Length of packet exceeds MAX_PULSES. Resetting.");
                    ser_state = WAIT_START;
                    break;
                }

                data_index = 0;
                ser_state = (expected_length > 0) ? WAIT_DATA : WAIT_CHECKSUM;

                rx_buffer = (uint8_t*)calloc(sizeof(uint8_t), expected_length * sizeof(uint8_t));
                adc_block = (uint8_t*)calloc(sizeof(uint8_t), expected_length * sizeof(uint8_t));
                pid_block = (uint8_t*)calloc(sizeof(uint8_t), expected_length * sizeof(uint8_t));
                if (rx_buffer == NULL) {
                    LOG_ERROR("Memory allocation failed for rx_buffer");
                    ser_state = WAIT_START;
                }
                if (adc_block == NULL) {
                    LOG_ERROR("Memory allocation failed for adc_block");
                }
                if (pid_block == NULL) {
                    LOG_ERROR("Memory allocation failed for pid_block");
                }

                break;

            case WAIT_DATA:
                if (data_index < expected_length) {
                    rx_buffer[data_index++] = byte;
                    checksum ^= byte;
                }
                if (data_index >= expected_length) {
                    ser_state = WAIT_CHECKSUM;
                }
                break;

            case WAIT_CHECKSUM:
                if (checksum == byte) {
                    ser_state = WAIT_END;
                } else {
                    LOG_WARN("Checksum failed. Resetting.");
                    ser_state = WAIT_START; // Reset on checksum error TODO: make sure that everything clears properly on reset
                }
                break;

            case WAIT_END:
                if (byte == END_BYTE) {
                    num_pulses = expected_length;

                    current_state = STATE_DATA_READY;
                    LOG_INFO("Complete packet of length %u received.", expected_length);

                    handle_message(); // Processes complete packet
                }
                ser_state = WAIT_START; // Reset TODO: make sure this resets cleanly
                return;
        }
    }
}


uint8_t* build_packet(int msg_type, uint8_t data[], size_t data_length) {
    /*
    Constructs a complete return packet

    Protocol Format:
    [START_BYTE][TYPE][LENGTH_MSB][LENGTH_LSB][DATA...][CHECKSUM][END_BYTE]
    */

    if (data_length > MAX_PULSES) {
        LOG_ERROR("Size of return data (%zu bytes) larger than %d", data_length, MAX_PULSES);
        return NULL;
    }

    size_t packet_length = 4 + data_length + 1 + 1;

    LOG_INFO("Building return packet of packet_length= %u and msg_type= 0x%x", packet_length, (uint8_t)msg_type);

    uint8_t* packet = malloc(packet_length);
    if (!packet) {
        LOG_ERROR("Memory allocation failed");
        return NULL;
    }

    packet[0] = START_BYTE;
    packet[1] = msg_type;
    packet[2] = (data_length >> 8) & 0xFF;
    packet[3] = data_length & 0xFF;

    // Data
    if (data_length > 0 && data) {
        memcpy(packet + 4 , data, data_length);
    }
    LOG_INFO("Data copied successfully");

    // Checksum (XOR of all bytes except END_BYTE)
    uint8_t checksum = 0;
    for (size_t i = 4; i < 4 + data_length; i++) {
        checksum ^= packet[i];
    }

    packet[4 + data_length] = checksum;
    LOG_DEBUG("checksum(dec, hex)=%u, %x");

    packet[4 + data_length + 1] = END_BYTE;

    return packet;
}

void serial_output(int msg_type, uint8_t data[], size_t data_length) {
    /*
    Builds and returns packets with data from the shot
    */

    uint8_t* packet = build_packet(msg_type, data, data_length);

    LOG_DEBUG("Return packet start and end bytes=%x, %x, %x, %x, %x, %x, %x, %x, %x", 
        packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7], packet[data_length+5]);

    LOG_INFO("packet built");
    #ifdef USE_USB_CDC
    // Implemented as workaround since fwrite was randomly adding 0x0d bytes in the middle of the return packets.
    // TODO: make sure that the tud method is implemented nicely throughout the protocol
    if (tud_cdc_connected()) {
        LOG_INFO("Writing return packet");
        LOG_INFO("packet[0]=%x, packet_length=%u", packet[0], (data_length+6));
        
        // Using packet chunking
        const size_t chunk_size = 64; // Optimal chunk size for tinyUSB
        size_t remaining = data_length + 6; // data + wrapper
        const uint8_t *data_ptr = packet;

        while (remaining > 0) {
            size_t to_send = (remaining > chunk_size) ? chunk_size : remaining; // Sends full chunk or all remaining bytes if under a full chunk is all that remains
            size_t sent = tud_cdc_write(data_ptr, to_send);

            data_ptr += sent;
            remaining -= sent;

            // Flush after each chunk
            tud_cdc_write_flush();
        }
    }
    // fwrite(packet, 1, data_length+6, stdout);
    // fflush(stdout);
    #elif defined(USE_UART)
    uart_write_blocking(UART_ID, packet, data_length+6);
    #endif

    free(packet);
}


// ==== Shot Controler ====
void __time_critical_func(on_pwm_wrap)() {
    /*
    Runs
    */
   // Clears interrupt flag
   pwm_clear_irq(0);

   // Pushes the stored next pulse state to the FIFO
   //pio_sm_put(pio0, sm, nextState);
   pio0->txf[sm] = nextState; // Same as pio_sm_put without checking

   
   // Calculates the following nextState for next cycle
   // Target values from 0 - 99 are negative pulses. Values from 100 - 199 are positive pulses.

   // Finds nextState from target
   if (target < 100) { // Negative pulses
        nextState = negCycle;
        delay = (100-target) * 5; // Delay in PIO cycles @ 25 MHz
    } else { // Positive pulses
        nextState = possCycle;
        delay = (target-99) * 5; // Delay in PIO cycles @ 25 MHz
    }

    // Sets Lower bound on DCP (1 us + switching time)/20 us ~7.5%
    if (delay < 25) {
        if (delay > 12 && last_state == 0) { 
            // Subdivides the lower bound "dead zone" into two sections and emulates 3.75% power for that zone
            delay = 25;
            last_state = 1;
        } else {
            nextState = freeCycle;
            last_state = 0;
        }
    } else {
        // Sets Upper bound on DCP (18 us + switching time)/20 us ~92.5%
        if (delay > 450) {delay = 450;}     
    }     

    // Sets nextState for the next cycle with the new delay
    nextState = nextState | ( delay << 8);

    cycleCount++;

    sem_release(&pwm_sem);
}


void init_shot() {
    /*
    Runs initialization for shot pulse output:
        Computes output state definitions
        Sets up PIO
        Sets up PWM wrapping
    */
    set_sys_clock_khz(125000, true); //125000


    // Computes state definitions
    stop2free = (S2 | S4) << 4;  // Turn on S2 and S4
    free2stop = 0;  // Turn all off
    freeCycle = ((S2 | S4) << 28) | ((S2 | S4) << 24) | ((S2 | S4) << 4) | (S2 | S4);
 
    free2poss = ((S2 | S3) << 4) | S2;  // S2 only then S2 and S3
    poss2free = ((S2 | S4) << 4) | S2;  // S2 only then S2 and S4
    possCycle = (poss2free << 24) | free2poss;
 
    free2neg = ((S1 | S4) << 4) | S4;  // S4 only then S1 and S4
    neg2free = ((S2 | S4) << 4) | S4;  // S4 only then S2 and S4
    negCycle = (neg2free << 24) | free2neg;


    // Choose PIO instance (0 or 1)
    PIO pio = pio0;

    uint sm = pio_claim_unused_sm(pio, true);

    // Add PIO program to PIO instruction memory. (SDK will find location and return with the memory offset of the program)
    uint offset = pio_add_program(pio, &pinsToggle_program);

    // PIO clock divider
    float div = 5.f; //(float)clock_get_hz(clk_sys) / pio_freq;

    // Initialize the program using the helper function in our .pio file
    pinsToggle_program_init(pio, sm, offset, startPin, div);


    // Set up PWM Wrapping
    pwm_clear_irq(0);
    pwm_set_irq_enabled(0, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, div); // Use system clock frequency (25 MHz)
    pwm_config_set_wrap(&config, 499);   // Wrap every 20 us (0-499)
    pwm_init(0, &config, false);

    
    // Set up ADC
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26 (i.e. pin 31))
    adc_select_input(0);


    // Semaphore for CPU <--> PWM
    sem_init(&pwm_sem, 1, 1);

    return;
}


void run_shot(uint16_t pulseCycles) {
    /*
    Runs shot pulses
    */
   uint32_t setpoint;
   float measurement; // ADC in
   const float conversion_factor = 3.3f / (1 << 12); // Convert ADC signal to voltage
   const float scale_factor = 100.0f; // Scales ADC result to -100 - 100 range (TODO: Confirm this)

    // Loads freewheeling as first PWM pulse
    delay = 250;
    nextState = (stop2free << 24) | (( delay << 8) | free2stop);


    // Set up PID
    // PID Definitions (TODO: Move to header?)
    #define PID_KP 1.0f
    #define PID_KI 1.0f
    #define PID_KD 1.0f

    #define PID_LIM_MIN 0.0f
    #define PID_LIM_MAX 200.0f

    #define PID_LIM_MIN_INT 0.0f
    #define PID_LIM_MAX_INT 100.0f

    #define SAMPLE_TIME_S 0.00002 // 20us

    pid_controller pid = {  PID_KP, PID_KI, PID_KD,
                            PID_LIM_MIN, PID_LIM_MAX,
			                PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                            SAMPLE_TIME_S };
    pid_controller_init(&pid);

    // Start PWM
    pwm_set_enabled(0, true);
    //busy_wait_ms(10);

    // Pulse Loop
    for (uint16_t cycle = 0; cycle < pulseCycles; cycle++) {
        setpoint = pulse_buffer[cycle];

        measurement = (((adc_read() * conversion_factor) * scale_factor)/2); // TODO: switch to rolling ADC implementation !! DO THIS SOON
        adc_block[cycle] = measurement;

        // PID Control
        pid_controller_update(&pid, (float)setpoint, measurement);

        // Loads PID modulated waveform to return array
        pid_block[cycle] = pid.out;

        sem_acquire_blocking(&pwm_sem);
        target = pid.out;
    }
    
    // Shutdown PWM
    pwm_set_enabled(0, false);
    // Return to off state
    pio_sm_put(pio0, sm, stop2free);


    return;
}


void shutdown_shot() {
    /*
    Turns off PWM and PIO
    */

    // Turn off PWM
    pwm_set_enabled(0, false);
    irq_set_enabled(PWM_IRQ_WRAP, false);
     
    // Return to off state
    pio_sm_put(pio0, sm, stop2free);
     
    //Turn off pio
    pio_sm_set_enabled(pio0, sm, false);

    return;
}


void wait_for_pin_low(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);

    LOG_INFO("Awaiting trigger.");
    
    while (gpio_get(pin) == 1) {
        tight_loop_contents();
    }

    LOG_INFO("Triggered.");
}


int main() {
    init_led();
    init_serial();

    // sleep_ms(500); // not needed

    init_shot();

    // Main operations loop
    while (true) {
    
        // Waits for next serial instruction input and loads it
        while (current_state != STATE_DATA_READY) {
            serial_input();
            update_led();
        }

        // Runs shot if one is sent
        if (current_msg_type == MSG_PWM) {
            // TODO: add ack before shot

            wait_for_pin_low(TRIGGER_PIN);

            current_state = STATE_OUTPUT_SHOT; // for update_led()
            run_shot(num_pulses);
            current_state = STATE_IDLE;

            sleep_ms(1000);
            serial_output(MSG_RETURN_RX, rx_buffer, num_pulses);
            sleep_ms(1000);
            serial_output(MSG_RETURN_ADC, adc_block, num_pulses);
            sleep_ms(1000);
            serial_output(MSG_RETURN_PID, pid_block, num_pulses);
        }
        LOG_INFO("Looping in main");
    }

    shutdown_shot();

    // put somwhere else later
    free(rx_buffer);
    rx_buffer = NULL;

    LOG_INFO("end");

    return 0;
}