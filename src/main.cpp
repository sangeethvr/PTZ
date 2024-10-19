#include <HardwareSerial.h>

// Define UART communication for ESP32 (using UART1)
#define TXD_PIN 17
#define RXD_PIN 16
HardwareSerial serialPort(1);

// Constants for Pelco D protocol
const uint8_t START_BYTE = 0xFF;
const int PACKET_SIZE = 7;  // Pelco D packets are 7 bytes long

// Define stepper motor control pins for TILT and PAN
#define TILT_DIR_PIN 12
#define TILT_STEP_PIN 14
#define PAN_DIR_PIN 27
#define PAN_STEP_PIN 26

// Tilt and pan limits (adjust based on your setup)
#define TILT_MIN_LIMIT 0
#define TILT_MAX_LIMIT 1000
#define PAN_MIN_LIMIT 0
#define PAN_MAX_LIMIT 2000

// Predefined middle positions for startup action
#define TILT_MIDDLE_POSITION (TILT_MAX_LIMIT / 2)
#define PAN_MIDDLE_POSITION (PAN_MAX_LIMIT / 2)

// Global state to track the motor movement
bool panMoving = false;
bool tiltMoving = false;
int panDirection = LOW;
int tiltDirection = LOW;
int tiltPosition = 0;
int panPosition = 0;  // Track the pan motor's position

// Ring buffer for UART reception
const int BUFFER_SIZE = 128;
uint8_t uartBuffer[BUFFER_SIZE];
int uartBufferIndex = 0;

// Function prototypes
void processBuffer();
uint8_t calculateChecksum(uint8_t address, uint8_t command1, uint8_t command2, uint8_t data1, uint8_t data2);
void executeCommand(uint8_t command1, uint8_t command2);
void stepMotor(int stepPin);
void calibrateMotors();  // Self-calibration
void moveMotorToPosition(int stepPin, int dirPin, int targetPosition, int &currentPosition);
void startupAction();  // New startup action

void setup() {
  // Initialize serial communication
  serialPort.begin(9600, SERIAL_8N1, RXD_PIN, TXD_PIN);
  Serial.begin(115200);  // Debugging via USB serial
  Serial.println("Pelco D Receiver Started");

  // Initialize stepper motor control pins for Tilt
  pinMode(TILT_DIR_PIN, OUTPUT);
  pinMode(TILT_STEP_PIN, OUTPUT);

  // Initialize stepper motor control pins for Pan
  pinMode(PAN_DIR_PIN, OUTPUT);
  pinMode(PAN_STEP_PIN, OUTPUT);

  // Perform self-calibration to find home positions
  calibrateMotors();

  // Perform startup action after calibration
  startupAction();
}

void loop() {
  // Read data from serial port
  while (serialPort.available()) {
    if (uartBufferIndex < BUFFER_SIZE) {
      uartBuffer[uartBufferIndex++] = serialPort.read();
    } else {
      // Reset the buffer if overflow occurs
      uartBufferIndex = 0;
      Serial.println("Buffer overflow, resetting buffer");
    }
  }

  // Process buffered data if we have enough bytes
  if (uartBufferIndex >= PACKET_SIZE) {
    processBuffer();
    uartBufferIndex = 0;  // Reset buffer index after processing
  }

  // Continuous movement logic for Pan motor
  if (panMoving) {
    stepMotor(PAN_STEP_PIN);
  }

  // Continuous movement logic for Tilt motor
  if (tiltMoving) {
    if ((tiltDirection == LOW && tiltPosition < TILT_MAX_LIMIT) || (tiltDirection == HIGH && tiltPosition > TILT_MIN_LIMIT)) {
      stepMotor(TILT_STEP_PIN);
      tiltPosition += (tiltDirection == LOW) ? 1 : -1;
    } else {
      Serial.println("Tilt movement stopped due to limit");
      tiltMoving = false;
    }
  }
}

void processBuffer() {
  // Find the first valid start byte
  int startIndex = -1;
  
  for (int i = 0; i < uartBufferIndex - PACKET_SIZE + 1; i++) {
    if (uartBuffer[i] == START_BYTE) {
      startIndex = i;
      break;
    }
  }

  // If no valid start byte is found, discard the buffer
  if (startIndex == -1) {
    Serial.println("No valid start byte found, discarding buffer");
    uartBufferIndex = 0;
    return;
  }

  // If we found a valid start byte but not enough data, wait for more
  if (uartBufferIndex - startIndex < PACKET_SIZE) {
    // Not enough data for a full packet, return and wait for more
    return;
  }

  // Process valid packet starting from startIndex
  uint8_t address = uartBuffer[startIndex + 1];
  uint8_t command1 = uartBuffer[startIndex + 2];
  uint8_t command2 = uartBuffer[startIndex + 3];
  uint8_t data1 = uartBuffer[startIndex + 4];
  uint8_t data2 = uartBuffer[startIndex + 5];
  uint8_t checksum = uartBuffer[startIndex + 6];

  if (checksum == calculateChecksum(address, command1, command2, data1, data2)) {
    executeCommand(command1, command2);
  } else {
    Serial.println("Checksum Error");
  }

  // Shift buffer after processing the valid packet
  uartBufferIndex -= (startIndex + PACKET_SIZE);
  memmove(uartBuffer, uartBuffer + startIndex + PACKET_SIZE, uartBufferIndex);
}

uint8_t calculateChecksum(uint8_t address, uint8_t command1, uint8_t command2, uint8_t data1, uint8_t data2) {
  return (address + command1 + command2 + data1 + data2) % 256;
}

void stepMotor(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(1000);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(1000);
}

// Move motor to a target position
void moveMotorToPosition(int stepPin, int dirPin, int targetPosition, int &currentPosition) {
  int direction = (targetPosition > currentPosition) ? LOW : HIGH;
  digitalWrite(dirPin, direction);

  while (currentPosition != targetPosition) {
    stepMotor(stepPin);
    currentPosition += (direction == LOW) ? 1 : -1;
  }
}

// Self-calibration routine to move motors to their known home position (min limit)
void calibrateMotors() {
  Serial.println("Calibrating Pan and Tilt motors...");
  
  // Move Tilt to minimum position
  moveMotorToPosition(TILT_STEP_PIN, TILT_DIR_PIN, TILT_MIN_LIMIT, tiltPosition);
  Serial.println("Tilt motor calibrated to minimum position");

  // Move Pan to minimum position
  moveMotorToPosition(PAN_STEP_PIN, PAN_DIR_PIN, PAN_MIN_LIMIT, panPosition);
  Serial.println("Pan motor calibrated to minimum position");
}

// Startup action to move motors to predefined positions
void startupAction() {
  Serial.println("Performing startup action...");

  // Move Pan to middle position
  Serial.println("Moving Pan to middle position");
  moveMotorToPosition(PAN_STEP_PIN, PAN_DIR_PIN, PAN_MIDDLE_POSITION, panPosition);

  // Move Tilt to middle position
  Serial.println("Moving Tilt to middle position");
  moveMotorToPosition(TILT_STEP_PIN, TILT_DIR_PIN, TILT_MIDDLE_POSITION, tiltPosition);

  // Move both back to home position
  Serial.println("Returning Pan and Tilt to home position");
  moveMotorToPosition(PAN_STEP_PIN, PAN_DIR_PIN, PAN_MIN_LIMIT, panPosition);
  moveMotorToPosition(TILT_STEP_PIN, TILT_DIR_PIN, TILT_MIN_LIMIT, tiltPosition);

  Serial.println("Startup action complete.");
}

void executeCommand(uint8_t command1, uint8_t command2) {
  if (command1 == 0x00) {
    switch (command2) {
      case 0x04:  // Pan Left
        Serial.println("Pan Left Command Received");
        panMoving = true;
        panDirection = LOW;
        digitalWrite(PAN_DIR_PIN, panDirection);
        break;

      case 0x02:  // Pan Right
        Serial.println("Pan Right Command Received");
        panMoving = true;
        panDirection = HIGH;
        digitalWrite(PAN_DIR_PIN, panDirection);
        break;

      case 0x08:  // Tilt Up
        if (tiltPosition < TILT_MAX_LIMIT) {
          Serial.println("Tilt Up Command Received");
          tiltMoving = true;
          tiltDirection = LOW;
          digitalWrite(TILT_DIR_PIN, tiltDirection);
        } else {
          Serial.println("Tilt Up Limit Reached");
        }
        break;

      case 0x10:  // Tilt Down
        if (tiltPosition > TILT_MIN_LIMIT) {
          Serial.println("Tilt Down Command Received");
          tiltMoving = true;
          tiltDirection = HIGH;
          digitalWrite(TILT_DIR_PIN, tiltDirection);
        } else {
          Serial.println("Tilt Down Limit Reached");
        }
        break;

      case 0x00:  // Stop Command
        Serial.println("Stop Command Received");
        panMoving = false;
        tiltMoving = false;
        break;

      default:
        Serial.println("Unknown Command Received");
        break;
    }
  }
}
