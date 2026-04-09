#include <Arduino.h>
#include <SPI.h>

// Pin Definitions
#define MASTER_SCK  37
#define MASTER_MISO 39
#define MASTER_MOSI 38
#define ENC1_CS     36

// AS5047P Registers
#define AS5047P_NOP         0x0000
#define AS5047P_ERRFL       0x0001  // Error register
#define AS5047P_PROG        0x0003  // Programming register
#define AS5047P_DIAAGC      0x3FFC  // Diagnostics & AGC
#define AS5047P_MAG         0x3FFD  // CORDIC magnitude
#define AS5047P_ANGLEUNC    0x3FFE  // Uncompensated angle
#define AS5047P_ANGLECOM    0x3FFF  // Compensated angle (use this one)

// AS5047P SPI Settings
// Mode 1 (CPOL=0, CPHA=1) or Mode 3 (CPOL=1, CPHA=1)
// Max clock 10MHz, we'll use 5MHz to be safe
#define AS5047P_SPI_SPEED   5000000
#define AS5047P_SPI_MODE    SPI_MODE1

SPIClass hspi(HSPI);

// Parity Helper
// AS5047P uses even parity on the 16-bit frame
bool evenParity(uint16_t data) {
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return data & 0x01;  // returns 1 if odd number of 1s
}

// Build a READ command frame
// Bit 15: Parity (even)
// Bit 14: R/W (1 = read)
// Bits 13:0: Address
uint16_t buildReadCommand(uint16_t address) {
    uint16_t cmd = (1 << 14) | (address & 0x3FFF);  // Set R/W bit
    if (evenParity(cmd)) cmd |= (1 << 15);           // Set parity if needed
    return cmd;
}

// SPI Transaction
uint16_t spiTransfer16(uint16_t data) {
    uint16_t result = 0;
    digitalWrite(ENC1_CS, LOW);
    delayMicroseconds(1);  // tCSn: CS setup time
    result  = hspi.transfer(data >> 8) << 8;
    result |= hspi.transfer(data & 0xFF);
    delayMicroseconds(1);  // tCSH: CS hold time
    digitalWrite(ENC1_CS, HIGH);
    delayMicroseconds(1);  // tSPI: inter-frame gap
    return result;
}

// Read a register (two transactions required per AS5047P protocol)
// First transaction: send the read command
// Second transaction: send NOP (or next command) to clock out the data
uint16_t readRegister(uint16_t address) {
    uint16_t cmd = buildReadCommand(address);

    spiTransfer16(cmd);         // First frame: send read command
    uint16_t response = spiTransfer16(buildReadCommand(AS5047P_NOP)); // Second frame: clock out data

    // Strip parity (bit 15) and error flag (bit 14)
    return response & 0x3FFF;
}

// ---- Read angle in raw counts (0–16383) ----
uint16_t readRawAngle() {
    return readRegister(AS5047P_ANGLECOM);
}

// ---- Convert raw counts to degrees ----
float rawToDegrees(uint16_t raw) {
    return (raw / 16384.0f) * 360.0f;
}

// Read diagnostics register 
// Returns true if magnetic field is OK
bool checkDiagnostics() {
    uint16_t diag = readRegister(AS5047P_DIAAGC);
    bool magl  = diag & (1 << 11);  // Magnetic field too low
    bool magh  = diag & (1 << 10);  // Magnetic field too high
    bool cordic = diag & (1 << 9);  // CORDIC overflow
    bool comp  = diag & (1 << 2);   // Compensation enabled

    if (magl)   Serial.println("[DIAG] WARNING: Magnetic field too LOW");
    if (magh)   Serial.println("[DIAG] WARNING: Magnetic field too HIGH");
    if (cordic) Serial.println("[DIAG] WARNING: CORDIC overflow");
    if (comp)   Serial.println("[DIAG] Compensation enabled");

    return !(magl || magh || cordic);
}

// Read error register 
void checkErrors() {
    uint16_t err = readRegister(AS5047P_ERRFL);
    if (err & 0x01) Serial.println("[ERR] FRERR: Framing error");
    if (err & 0x02) Serial.println("[ERR] INVCOMM: Invalid command");
    if (err & 0x04) Serial.println("[ERR] PARERR: Parity error");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("AS5047P Encoder Test");

    // CS pin
    pinMode(ENC1_CS, OUTPUT);
    digitalWrite(ENC1_CS, HIGH);

    // Init SPI on HSPI with custom pins
    hspi.begin(MASTER_SCK, MASTER_MISO, MASTER_MOSI, ENC1_CS);
    hspi.beginTransaction(SPISettings(AS5047P_SPI_SPEED, MSBFIRST, AS5047P_SPI_MODE));

    delay(100);

    // Check diagnostics on startup
    Serial.println("Checking diagnostics...");
    checkDiagnostics();
    checkErrors();
}

void loop() {
    uint16_t raw = readRawAngle();
    float degrees = rawToDegrees(raw);

    Serial.printf("Raw: %5d  |  Angle: %7.3f deg\n", raw, degrees);

    delay(50);  // ~20Hz update rate
}