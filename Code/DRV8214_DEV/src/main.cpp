#include "DRV8214.h"
#include "TCA9548.h"

#define NUM_DRIVERS 1 // Number of DRV8214 drivers in the system
#define IPROPI_RESISTOR 680 // Value in Ohms of the resistor connected to IPROPI pin
#define NUM_RIPPLES 156 // Number of current ripples per output shaft revolution (= nb of ripples per motor revolution x reduction ratio)
#define MOTOR_INTERNAL_RESISTANCE 20 // Internal resistance of the motor in Ohms

#define SDA_PIN 8
#define SCL_PIN 9
#define FAULT_PIN 41
#define BUTTON_PIN 40
#define CLEAR_FAULT_PIN 39

void printByteAsBinary(uint8_t value);
void print2BytesAsBinary(uint16_t value);
void printRegisters(uint8_t driver_id);

bool direction_changed = true;
bool fault_cleared = false;

// Create an array of DRV8214 objects
DRV8214 drivers[NUM_DRIVERS] = {
    DRV8214(DRV8214_I2C_ADDR_ZZ, 0, IPROPI_RESISTOR, NUM_RIPPLES, MOTOR_INTERNAL_RESISTANCE)
};

// Create an array of configuration structs
DRV8214_Config driver_configs[NUM_DRIVERS];

// Create a TCA9548 object
TCA9548 multiplexer = TCA9548(0x70); 

void setup() {
    Serial.begin(115200);
    delay(5000);
    Wire.begin(SDA_PIN, SCL_PIN);

    // Set button pin as input with pull-up resistor
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(FAULT_PIN, INPUT_PULLUP);

    Serial.println("Initializing DRV8214 drivers...");

    for (int i = 0; i < NUM_DRIVERS; i++) {  // Initialize each driver
        driver_configs[i] = DRV8214_Config();
        if (i == 0) { driver_configs[i].verbose = true; } // Enable verbose only for the first driver
        drivers[i].init(driver_configs[i]);
        drivers[i].resetFaultFlags();
    }
    Serial.println("DRV8214 drivers initialized successfully!");
    delay(500);
}

void loop() {

    // Check if the button is pressed
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (!direction_changed) {
            drivers[0].turnForward(100, 3.3, 0.2);
            direction_changed = true;
        }
        printRegisters(0);
        
    } else {
        if (direction_changed) {
            drivers[0].turnReverse(100, 3.3, 0.2);
            direction_changed = false;
        }
        printRegisters(0);
    }
    if (digitalRead(FAULT_PIN) == LOW) {
        if (!fault_cleared) {
            drivers[0].resetFaultFlags();
            fault_cleared = true;
        }
    } else {
        fault_cleared = false;
    }
    delay(500);
}

// Helper function to print a byte as binary
void printByteAsBinary(uint8_t value) {
    for (int i = 7; i >= 0; i--) {
        Serial.print((value >> i) & 1);
    }
    Serial.println(); // Move to next line
}

// Helperfunction to print 2 bytes as binary
void print2BytesAsBinary(uint16_t value) {
    for (int i = 15; i >= 0; i--) {
        Serial.print((value >> i) & 1);
    }
    Serial.println(); // Move to next line
}

void printRegisters(uint8_t driver_id) {

    Serial.print("Speed: ");
    Serial.print(drivers[driver_id].getMotorSpeedRPM(), DEC);
    Serial.print(" RPM or ");
    Serial.print(drivers[driver_id].getMotorSpeedRAD(), DEC);
    Serial.print(" rad/s | ");
    Serial.print("Voltage: ");
    Serial.print(drivers[driver_id].getMotorVoltage(), DEC);
    Serial.print(" V | ");
    Serial.print("Current: ");
    Serial.print(drivers[driver_id].getMotorCurrent(), DEC);
    Serial.print(" A | ");
    Serial.print("Duty Cycle: ");
    Serial.print(drivers[driver_id].getDutyCycle(), DEC);
    Serial.print("% | ");
    Serial.print("Tinrush: ");
    Serial.print(drivers[driver_id].getInrushDuration());
    Serial.println(" ms");
    drivers[driver_id].printFaultStatus();
    Serial.print("CONFIG0: 0b");
    printByteAsBinary(drivers[driver_id].getCONFIG0());
    Serial.print("CONFIG3: 0b");
    printByteAsBinary(drivers[driver_id].getCONFIG3());
    Serial.print("CONFIG4: 0b");
    printByteAsBinary(drivers[driver_id].getCONFIG4());
    Serial.print("REG_CTRL0: 0b");
    printByteAsBinary(drivers[driver_id].getREG_CTRL0());
    Serial.print("REG_CTRL1: 0b");
    printByteAsBinary(drivers[driver_id].getREG_CTRL1());
    Serial.print("REG_CTRL2: 0b");
    printByteAsBinary(drivers[driver_id].getREG_CTRL2());
}