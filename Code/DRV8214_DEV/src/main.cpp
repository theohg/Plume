#include "DRV8214.h"
#include "TCA9548.h"
#include "Adafruit_NeoPixel.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

#define NUM_DRIVERS 1 // Number of DRV8214 drivers in the system
#define IPROPI_RESISTOR 680 // Value in Ohms of the resistor connected to IPROPI pin
#define NUM_RIPPLES 6 // Number of current ripples per output shaft revolution (= nb of ripples per motor revolution x reduction ratio)
#define MOTOR_INTERNAL_RESISTANCE 20 // Internal resistance of the motor in Ohms
#define MOTOR_REDUCTION_RATIO 26 // Reduction ratio of the motor
#define MAX_MOTOR_RPM 1154 // Maximum speed of the motor in RPM

#define SDA_PIN 2
#define SCL_PIN 3
#define CLEAR_FAULT_PIN 21
#define RESET_COUNTER_PIN 26
#define SPEED_PIN 38
#define CHANNEL_PIN 39
#define INFO_PIN 42
#define DIRECTION_PIN 45
#define FAULT_PIN 46
#define STOP_PIN 47
#define LED_PIN 48
#define POTENTIOMETER_PIN 6
#define NUM_LEDS 1

void printByteAsBinary(uint8_t value);
void print2BytesAsBinary(uint16_t value);
void printRegisters(uint8_t driver_id);
void setupGPIOs();
void checkFaultCondition();                 // Checks and handles motor driver fault conditions
void checkStopCondition();                  // Checks if the stop condition is triggered and stops the motors
void updateSpeedAndVoltage();               // Handles speed and voltage adjustments based on user input
void handleMotorDirection();                // Controls motor direction based on input
void handleFaultClear();                    // Handles fault clearing when triggered
void handleCounterReset();                  // Resets the motor counter when triggered
void handleChannelSelection();              // Handles channel selection based on input
void homeMotor();                           // Homes the motor to the starting position
void stallMotorDown();                      // Stalls the motor in the down direction
void updateMotorPositionPotentiometer();    // Updates the motor position based on the potentiometer reading
void updateMotorPositionSerial();           // Updates the motor position based on the serial input

// Global variables
static uint16_t speed = MAX_MOTOR_RPM;
static uint16_t speed_low = MAX_MOTOR_RPM*0.7;
static float voltage = 2.0;
static float current = 0.2; 
static int speed_step = 5; // speed_step ranges from 1 (lowest) to 5 (full speed)
static float nb_of_steps = 5;
static uint8_t faults = 0;
static uint32_t color = 0;
uint16_t ripple_target = 26000;    // Number of ripples to move when in ripple mode
static const bool stops_after_ripples = true;   // If true, the motor will stop when ripple_target is reached
static const bool stops_at_stall = true;        // If true, the motor will stop when a stall is detected
uint8_t i2c_channel = 1;           // I2C channel to select
bool back_and_forth = 0;       // If true, the motor will move back and forth
bool use_ripple_mode = 0;       // Set to true to use ripple-based motion
bool stopped = true;
bool direction_changed = false;
const unsigned long led_300 = 300; // LED ON in milliseconds for short pules
unsigned long redStartTime = 0;
bool isRedActive = false;
uint32_t savedColor = 0;  // To store the previous LED color
bool stalled = false;
bool channel_mode = false;
bool speed_mode = false;
bool direction = false;
bool inHomingRoutine = false;
float target_position = 0.0;
float module_position = 0.0;
bool last_direction;    

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Create an array of DRV8214 objects
DRV8214 drivers[NUM_DRIVERS] = {
    DRV8214(DRV8214_I2C_ADDR_ZZ, 0, IPROPI_RESISTOR, NUM_RIPPLES, MOTOR_INTERNAL_RESISTANCE, MOTOR_REDUCTION_RATIO, MAX_MOTOR_RPM)
};

// Create an array of configuration structs
DRV8214_Config driver_configs[NUM_DRIVERS];

// Create a TCA9548 object
TCA9548 multiplexer = TCA9548(0x70); 

// We can store a pointer to any class derived from Print or Stream
Stream* console = nullptr;

// Helper function to attempt USBSerial first, then fallback to hardware Serial:
void initializeConsole(unsigned long baudRate = 115200, unsigned long timeoutMs = 2000) {
    // First try initializing USBSerial
    USBSerial.begin(baudRate);

    unsigned long start = millis();
    while (!USBSerial && (millis() - start) < timeoutMs) {
        // waiting briefly to see if the USB CDC becomes ready
    }

    if (USBSerial) {
        // If the USB CDC is up, use USBSerial as the console
        console = &USBSerial;
        console->println("Using USB CDC for console");
    } else {
        // Otherwise, fall back to hardware Serial
        Serial.begin(baudRate);
        console = &Serial;
        console->println("Using hardware Serial for console");
    }
    delay(2000); // Allow time for the Serial port to initialize
}

void setup() {
    initializeConsole(); // Decide which Serial to use

    Wire.begin(SDA_PIN, SCL_PIN);

    strip.begin();
    strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red
    strip.show();

    setupGPIOs();

    console->println("\nInitializing TCA9548 multiplexer...\n");

    if (multiplexer.begin()) {
        console->println("TCA9548 multiplexer initialized successfully!\n");
        multiplexer.selectChannel(i2c_channel);
    } else { console->println("Failed to initialize TCA9548 multiplexer!\n"); }

    console->println("\nInitializing TCA9548 multiplexer...\n");

    console->println("Initializing MCP23017 multiplexer...\n");

    for (int i = 0; i < NUM_DRIVERS; i++) {  // Initialize each driver
        driver_configs[i] = DRV8214_Config();
        if (i == 0) { driver_configs[i].verbose = true; } // Set verbose only for the first driver
        drivers[i].setDebugStream(console); // Let the driver’s library know which stream to use for debug
        drivers[i].init(driver_configs[i]);
        drivers[i].resetFaultFlags();
    }
    console->println("DRV8214 drivers initialized successfully!\n");
}

void loop() {
    checkFaultCondition();
    checkStopCondition();
    updateSpeedAndVoltage();
    //handleMotorDirection();
    updateMotorPositionPotentiometer();
    handleFaultClear();
    handleCounterReset();
    handleChannelSelection();
    printRegisters(0);
}

void printRegisters(uint8_t driver_id) {
    if (digitalRead(INFO_PIN) == LOW) {
        color = strip.getPixelColor(0); // save current led color
        strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red
        strip.show();
        console->print("Speed of motor: ");
        console->print(drivers[driver_id].getMotorSpeedRPM(), DEC);
        console->print(" RPM or ");
        console->print(drivers[driver_id].getMotorSpeedRAD(), DEC);
        console->print(" rad/s | ");
        console->print("Voltage: ");
        console->print(drivers[driver_id].getMotorVoltage(), 4);
        console->print(" V | ");
        console->print("Current: ");
        console->print(drivers[driver_id].getMotorCurrent(), 4);
        console->println(" A");
        console->print("Speed of shaft: ");
        console->print(drivers[driver_id].getMotorSpeedShaftRPM(), DEC);
        console->print(" RPM or ");
        console->print(drivers[driver_id].getMotorSpeedShaftRAD(), DEC);
        console->print(" rad/s | ");
        console->print("Duty Cycle: ");
        console->print(drivers[driver_id].getDutyCycle(), DEC);
        console->print("% | ");
        console->print("Tinrush: ");
        console->print(drivers[driver_id].getInrushDuration());
        console->println(" ms");
        console->print("RC_STATUS1 (SPEED): 0b");
        printByteAsBinary(drivers[driver_id].getMotorSpeedRegister());
        console->print("REG_STATUS1 (VOLTAGE): 0b");
        printByteAsBinary(drivers[driver_id].getMotorVoltageRegister());
        console->print("REG_STATUS2 (CURRENT): 0b");
        printByteAsBinary(drivers[driver_id].getMotorCurrentRegister());
        console->print("Ripple counter: ");
        console->print(drivers[driver_id].getRippleCount(), DEC);
        console->print(" | 0b");
        print2BytesAsBinary(drivers[driver_id].getRippleCount());
        drivers[driver_id].printFaultStatus();
        console->print("CONFIG0: 0b");
        printByteAsBinary(drivers[driver_id].getCONFIG0());
        console->print("CONFIG3: 0b");
        printByteAsBinary(drivers[driver_id].getCONFIG3());
        console->print("CONFIG4: 0b");
        printByteAsBinary(drivers[driver_id].getCONFIG4());
        console->print("REG_CTRL0: 0b");
        printByteAsBinary(drivers[driver_id].getREG_CTRL0());
        console->print("REG_CTRL1 (TARGET SPEED): 0b");
        printByteAsBinary(drivers[driver_id].getREG_CTRL1());
        console->print("REG_CTRL2 (DUTY): 0b");
        printByteAsBinary(drivers[driver_id].getREG_CTRL2());
        console->print("RC_CTRL0: 0b");
        printByteAsBinary(drivers[driver_id].getRC_CTRL0());
        console->print("KMC: ");
        console->print(drivers[driver_id].getKMC());
        console->print(" | Ripple Threshold Scaled: ");
        console->print(drivers[driver_id].getRippleThresholdScaled(), DEC);
        console->print(" | Ripples Threshold: ");
        console->print(drivers[driver_id].getRippleThreshold(), DEC);
        console->print(" or 0b");
        print2BytesAsBinary(drivers[driver_id].getRippleThreshold());
        // console->print("RC_CTRL1: 0b");
        // printByteAsBinary(drivers[driver_id].getRC_CTRL1());
        // console->print("RC_CTRL2: 0b");
        // printByteAsBinary(drivers[driver_id].getRC_CTRL2());
        // console->print("RC_CTRL6: 0b");
        // printByteAsBinary(drivers[driver_id].getRC_CTRL6());
        // console->print("RC_CTRL7: 0b");
        // printByteAsBinary(drivers[driver_id].getRC_CTRL7());
        // console->print("RC_CTRL8: 0b");
        // printByteAsBinary(drivers[driver_id].getRC_CTRL8());
        console->println();
        delay(50);
        strip.setPixelColor(0, color); // restore led color
        strip.show();
    } 
}

void printByteAsBinary(uint8_t value) {
    for (int i = 7; i >= 0; i--) {
        console->print((value >> i) & 1);
    }
    console->println();
}

void print2BytesAsBinary(uint16_t value) {
    for (int i = 15; i >= 0; i--) {
        console->print((value >> i) & 1);
    }
    console->println();
}

void setupGPIOs() {
    pinMode(CLEAR_FAULT_PIN, INPUT_PULLUP);
    pinMode(RESET_COUNTER_PIN, INPUT_PULLUP);
    pinMode(SPEED_PIN, INPUT_PULLUP);
    pinMode(FAULT_PIN, INPUT_PULLUP);
    pinMode(DIRECTION_PIN, INPUT_PULLUP);
    pinMode(STOP_PIN, INPUT_PULLUP);
    pinMode(INFO_PIN, INPUT_PULLUP);
    pinMode(CHANNEL_PIN, INPUT_PULLUP);

    analogReadResolution(12);  // Ensure 12-bit resolution
    analogSetPinAttenuation(POTENTIOMETER_PIN, ADC_11db);  // Allow full 0-3.3V range
}

void checkFaultCondition() {
    // Check for FAULT condition
    if (digitalRead(FAULT_PIN) == LOW) {
        if (stops_at_stall && stops_after_ripples && !back_and_forth) {
            stopped = true;
        }
        for (int i = 0; i < NUM_DRIVERS; i++) {
            // only print fault status if there is a new fault
            if (drivers[i].getFaultStatus() != faults) {
                console->println("FAULT DETECTED");
                faults = drivers[i].getFaultStatus();
                drivers[i].printFaultStatus();
                drivers[i].brakeMotor();
            }
             // If both faults are present, light LED in white first
            if ((faults & 0b00100000) && (faults & 0b00000001)) {
                strip.setPixelColor(0, strip.Color(127, 127, 127)); // White
            } 
            // If 6th bit is set, light LED in purple for stall
            else if (faults & 0b00100000) {
                stalled = true;
                strip.setPixelColor(0, strip.Color(127, 0, 127)); // Purple
            } 
            // If 5th bit is set, light LED in yellow for overcurrent
            else if (faults & 0b00000001) {
                strip.setPixelColor(0, strip.Color(127, 127, 0)); // Yellow
            }
            strip.show();
        }
    }
}

void checkStopCondition() {
    // Check for STOP condition, and use led timing as debounce delay
    if (digitalRead(STOP_PIN) == LOW && !isRedActive) {
        // When STOP is pressed, if not already in red state, set LED red
        if (stopped) {
            stopped = false;
            if (!isRedActive) {
                savedColor = strip.getPixelColor(0);      // Save current LED color
                strip.setPixelColor(0, strip.Color(255, 0, 0)); // Set LED to red
                strip.show();
                redStartTime = millis();                  // Record the time of change
                isRedActive = true;
            }
            console->println("RESUMING MOTORS");
            direction_changed = !direction_changed;
            return;
        } else {
            stopped = true;
            if (!isRedActive) {
                savedColor = strip.getPixelColor(0);      // Save current LED color
                strip.setPixelColor(0, strip.Color(255, 0, 0)); // Set LED to red
                strip.show();
                redStartTime = millis();                  // Record the time of change
                isRedActive = true;
            }
            // Stop motors
            console->println("STOPPING MOTORS");
            for (int i = 0; i < NUM_DRIVERS; i++) {
                drivers[i].brakeMotor();
            }
        }
    }
    // Check if 300ms have elapsed since the LED turned red
    if (isRedActive && (millis() - redStartTime >= 300)) {
        // Restore the original LED color
        strip.setPixelColor(0, savedColor);
        strip.show();
        isRedActive = false;
    }
}

void updateSpeedAndVoltage() {
    // Check if the SPEED_PIN is low and update speed/voltage accordingly.
    if (digitalRead(SPEED_PIN) == LOW and speed_mode) {
        // If at lowest speed, reset to full speed; otherwise, decrease one step.
        if (speed_step <= 1) {
            speed_step = nb_of_steps;
        } else {
            speed_step--;
        }
        speed = (uint16_t)(MAX_MOTOR_RPM * speed_step / nb_of_steps);
        voltage = 2.0 * speed_step / nb_of_steps;
        console->print("New speed: ");
        console->print(speed);
        console->print(" RPM, Voltage: ");
        console->println(voltage);
        // Update the speed and voltage for all drivers.
        for (int i = 0; i < NUM_DRIVERS; i++) {
            drivers[i].setRippleSpeed(speed);
            drivers[i].setVoltageSpeed(voltage);
        }
        // Simple debounce delay for the button press.
        delay(300);
    } else if (digitalRead(SPEED_PIN) == LOW) {
        inHomingRoutine = false;
        homeMotor();
    }
}

void handleMotorDirection() {
    if (back_and_forth) {
        // reverse direction when fault due to counter threshold is reached
        // alternate direction based on the directtion_changed flag
        if (drivers[0].getFaultStatus() & 0b00000001 && !stopped) {
            console->println("REVERSING DIRECTION");
            drivers[0].resetRippleCounter();
            drivers[0].resetFaultFlags();
            direction_changed = !direction_changed;
        }
        if (direction_changed && !direction) {
            drivers[0].turnXRipples(ripple_target, stops_after_ripples, false, speed, voltage, current);
            if (!isRedActive) {  strip.setPixelColor(0, strip.Color(0, 0, 255));} // Blue
            direction = true;
        } else if (!direction_changed && direction) {
            drivers[0].turnXRipples(ripple_target, stops_after_ripples, true, speed, voltage, current);
            if (!isRedActive) { strip.setPixelColor(0, strip.Color(0, 255, 0));} // Green
            direction = false;
        }
        strip.show();
        return;
    }
    // Motor control based on the DIRECTION_PIN, moving a fixed number of ripples.
    if (use_ripple_mode && !stopped) {
        if (digitalRead(DIRECTION_PIN) == LOW) { 
            if (!direction_changed) { // Only send a motion command when there is a change in direction.
                drivers[0].turnXRipples(ripple_target, stops_after_ripples, true, speed, voltage, current);
                direction_changed = true;
                if (!isRedActive) { strip.setPixelColor(0, strip.Color(0, 255, 0));} // Green
            }
        } else {
            if (direction_changed) {
                drivers[0].turnXRipples(ripple_target, stops_after_ripples, false, speed, voltage, current);
                direction_changed = false;
                if (!isRedActive) {  strip.setPixelColor(0, strip.Color(0, 0, 255));} // Blue
            }
        }
        strip.show();
    } else {
        // Standard motor control based on the DIRECTION_PIN.
        if (digitalRead(DIRECTION_PIN) == LOW) {
            if (!direction_changed) { // Only send a motion command when there is a change in direction.
                direction_changed = true;
                if (!stopped) {
                    drivers[0].turnForward(speed_low, voltage, current);
                    //drivers[0].turnXRipples(300, stops_after_ripples, true, speed, voltage, current);
                }
                if (!isRedActive) { strip.setPixelColor(0, strip.Color(0, 255, 0));} // Green
            }
        } else {
            if (direction_changed) {
                direction_changed = false;
                if (!stopped) {
                    // drivers[0].setBridgeBehaviorThresholdReached(false);
                    // drivers[0].resetRippleCounter();
                    drivers[0].turnReverse(speed, voltage, current);
                    //drivers[0].turnXRipples(200, stops_after_ripples, false, speed, voltage, current);
                }
                if (!isRedActive) { strip.setPixelColor(0, strip.Color(0, 0, 255));} // Blue
            }
        }
        strip.show();
    }
}

void handleFaultClear() {
    // Handle fault clear request.
    if (digitalRead(CLEAR_FAULT_PIN) == LOW && !isRedActive) {
        if (!isRedActive) {
            strip.setPixelColor(0, strip.Color(255, 0, 0)); // Set LED to red
            strip.show();
            redStartTime = millis();                  // Record the time of change
            isRedActive = true;
        }
        stalled = false;
        console->println("FAULT CLEARED");
        drivers[0].resetFaultFlags();
        faults = drivers[0].getFaultStatus();
        if (!back_and_forth) { stopped = true;}
    }
    // Check if 300ms have elapsed since the LED turned red
    if (isRedActive && (millis() - redStartTime >= 300)) {
        // Restore the color depending on the direction
        if (digitalRead(DIRECTION_PIN) == LOW) {
            strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
        } else {
            strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue
        }
        strip.show();
        isRedActive = false;
    }
}
void handleCounterReset() {
    // Handle counter reset request.
    if (digitalRead(RESET_COUNTER_PIN) == LOW && !isRedActive) {
        if (!isRedActive) {
            strip.setPixelColor(0, strip.Color(255, 0, 0)); // Set LED to red
            strip.show();
            redStartTime = millis();                  // Record the time of change
            isRedActive = true;
        }
        console->println("COUNTER RESETTED");
        drivers[0].resetRippleCounter();
    }
    // Check if 300ms have elapsed since the LED turned red
    if (isRedActive && (millis() - redStartTime >= 300)) {
        // Restore the color depending on the direction
        if (digitalRead(DIRECTION_PIN) == LOW) {
            strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
        } else {
            strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue
        }
        strip.show();
        isRedActive = false;
    }
}

void handleChannelSelection() {
    // Handle channel selection based on input.
    if (digitalRead(CHANNEL_PIN) == LOW && channel_mode) {
        console->println("CHANNEL SELECTED");
        if (i2c_channel >= 7) { i2c_channel = 0; } else { i2c_channel++; }
        color = strip.getPixelColor(0); // save current led color
        strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red
        strip.show();
        delay(300);
        strip.setPixelColor(0, color); // restore led color
        strip.show();
        multiplexer.selectChannel(i2c_channel);
    } else if (digitalRead(CHANNEL_PIN) == LOW){
        // if (back_and_forth && use_ripple_mode) {
        //     console->println("STANDARD MODE (NO RIPPLES)");
        // } else {
        //     console->println("RIPPLE MODE ENABLED");
        // }
        // back_and_forth = !back_and_forth;
        // use_ripple_mode = !use_ripple_mode;

        inHomingRoutine = false;
        stallMotorDown();
    }
}

void homeMotor() {
    stallMotorDown();
    console->print("HOMING MOTOR...  ");
    drivers[0].resetRippleCounter();
    drivers[0].resetFaultFlags();
    drivers[0].setStallBehavior(false);
    drivers[0].turnXRipples(13000, stops_after_ripples, false, speed, voltage, current);
    while (!(drivers[0].getFaultStatus() & 0b00000001)){}
    drivers[0].setStallBehavior(true);
    drivers[0].resetRippleCounter();
    module_position = 13500;
    drivers[0].brakeMotor();
    console->println("MOTOR HOMED!\n");
}

void stallMotorDown() {
    console->print("STALLING MOTOR DOWN...  ");
    if (!inHomingRoutine) {
        drivers[0].resetRippleCounter();
        drivers[0].resetFaultFlags();
        //drivers[0].turnForward(speed_low, voltage, current);
        drivers[0].turnXRipples(65000, stops_after_ripples, true, speed_low, voltage, current);
        while(!(drivers[0].getFaultStatus() & 0b00100000)) {}
        console->println("STALLED DOWN!");
        inHomingRoutine = true;
    }
    module_position = 0;
}

void updateMotorPositionPotentiometer() {

    if (stalled) {return;}
    // 1. Read the potentiometer to set the desired position (0 to 4 mm)
    float target_mm = analogRead(POTENTIOMETER_PIN) * (4.0 / 4095.0);
    long target_ripples = (long)(target_mm * 6500.0);
    console->print("Target position: ");
    console->print(target_ripples);
    console->print(" ripples or ");
    console->print(target_mm);
    console->print(" mm  | Current position: ");
    console->print(module_position);
    console->print(" ripples or ");
    console->print(module_position / 6500.0);
    console->println(" mm");

    // 2. Compute the error and determine incremental movement.
    long error = target_ripples - module_position;
    if (abs(error) < 150) {
        delay(100);
        return;
    }
    // 3. Update the current module_position based on the previous move.
    // Since calling turnXRipples resets the motor driver's ripple counter,
    // we must update module_position by reading the counter before issuing a new move.
    long ripplesDelta = drivers[0].getRippleCount();
    if (ripplesDelta != 0) {
        if (last_direction == false) { // false means moving UP, so add the ripples.
            module_position += ripplesDelta;
        } else { // true means moving DOWN, so subtract the ripples.
            module_position -= ripplesDelta;
        }
        console->print("Updated module position (ripples): ");
        console->println(module_position);
    }
  
    // Determine the move direction: false = UP (need to add), true = DOWN (need to subtract)
    bool direction = (error < 0);
    
    // Choose a small incremental move to allow for re-adjustment.
    long stepSize = 100;
    long moveAmount = (abs(error) < stepSize) ? abs(error) : stepSize;
  
    console->print("Moving motor ");
    console->print(moveAmount);
    console->print(" ripples ");
    console->println(direction ? "DOWN" : "UP");
    
    // Save the current move direction for updating module_position next time.
    last_direction = direction;
  
    // 4. Command the motor to move.
    drivers[0].turnXRipples(moveAmount, stops_after_ripples, direction, speed, voltage, current);
  
    // 5. Wait for the move to complete, allowing for an interruption if the target changes.
    unsigned long startTime = millis();
    const unsigned long movementTimeout = 50; // Adjust as needed based on motor speed.
    while ((millis() - startTime) < movementTimeout) {
        float newTarget_mm = analogRead(POTENTIOMETER_PIN) * (4.0 / 4095.0);
        // If the user changes the potentiometer significantly, break early.
        if (abs(newTarget_mm - target_mm) > 0.1) {
            drivers[0].brakeMotor();
            break;
        }
        delay(10);
    }
}

void updateMotorPositionSerial(){
    
}
