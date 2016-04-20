#include <Arduino.h>

#include <RF24.h>
#include <NewPing.h>
#include <LowPower.h>

// [O] To enable / disable the step-up converter
#define STEP_UP_PIN   5
// [O] Ultrasound sensor 'trigger' pin
#define TRIGGER_PIN  3
// [I] Ultrasound sensor 'echo' pin
#define ECHO_PIN     4
// [I] Used to define the address of the RF module (LOW = 1, HIGH = 2)
#define ADDRESS_SELECTION_PIN 9

//////////

// - Number of measures to do by duty cycle
#define NB_MEASURES 5
// - Number of measures to skip for the average
#define NB_MEASURES_SKIP 1

// - Sleep unit
#define SLEEP_STEP_DURATION SLEEP_250MS
// - Sleep factor
#define SLEEP_STEP_NB 6

// - RF message payload size
#define PAYLOAD_SIZE 3

//////////

// - Message definition
typedef struct {
  // - Address of the device
  uint8_t address;
  // - Value of the sensor in centimeters
  unsigned short value;
} Message;

//////////

// - Gateway address
uint8_t gatewayAddress[] = { 0x00, 0xA1, 0xB2, 0xC3, 0xD4 };
// - Node address (edited during setup)
uint8_t nodeAddress[] = { 0xFF, 0xA1, 0xB2, 0xC3, 0xD4 };

// - RF interface
RF24 radio(7,8);
// - Ultrasound sensor interface
NewPing sonar(TRIGGER_PIN, ECHO_PIN);

void setup() {
  // - Setup serial port
  Serial.begin(115200);

  // - Pin initialization
  pinMode(STEP_UP_PIN, OUTPUT);
  digitalWrite(STEP_UP_PIN, LOW);

  // - Read address selection
  pinMode(ADDRESS_SELECTION_PIN, INPUT_PULLUP);
  if(digitalRead(ADDRESS_SELECTION_PIN) == HIGH) {
    nodeAddress[0] = 0x01;
  } else {
    nodeAddress[0] = 0x02;
  }

  // - Setup and configure radio
  radio.begin();
  //radio.enableDynamicPayloads();
  radio.setPayloadSize(PAYLOAD_SIZE);
  radio.setDataRate(RF24_250KBPS); // - Lower speed
  radio.setPALevel(RF24_PA_HIGH); // - Higher power level

  radio.openWritingPipe(gatewayAddress); // - To send values
  radio.openReadingPipe(1, nodeAddress); // - Not used in reality

  radio.powerDown();
}

void loop() {
  // - Sleep loop
  int i = 0;
  for(i = 0; i < SLEEP_STEP_NB; i++) {
    LowPower.powerDown(SLEEP_STEP_DURATION, ADC_OFF, BOD_OFF);
  }

  Serial.println(F("Awake"));

  // - Wake up the sensor through the step-up converter
  digitalWrite(STEP_UP_PIN, HIGH);
  // - Wait few ms for sensor init (TBD: minimal value)
  delay(100);

  // - Let's measure the distance between the sensor and the surface of the water
  int valuesInCm[NB_MEASURES];
  double avgMeasureCm = 0.0d;
  for(i = 0; i < NB_MEASURES; i++) {
    valuesInCm[i] = sonar.ping_cm();
    Serial.print(F("Distance: "));
    Serial.print(valuesInCm[i]);
    Serial.println(F("cm"));

    // - Skip some values if necessary
    if(i >= NB_MEASURES_SKIP) {
      avgMeasureCm += valuesInCm[i];
    }

    // - Delay for remaining echos (cf. sensor datasheet) before next reading
    delay(70);
  }

  // - Power off the sensor
  digitalWrite(STEP_UP_PIN, LOW);

  // - Computing average value (still in centimeters)
  unsigned short valueToSend = (unsigned short) round(avgMeasureCm / (double) (NB_MEASURES - NB_MEASURES_SKIP));

  Serial.print(F("Now sending: "));
  Serial.println(valueToSend);

  // - Message content
  Message msg;
  msg.address = nodeAddress[0];
  msg.value = valueToSend;

  // - Time to start the radio
  radio.powerUp();

  // - Message emission (with ACK)
  if ( radio.write(&msg, PAYLOAD_SIZE) ) {
      if(!radio.available()) {
          Serial.println(F("Got (blank) ack"));
      }
  } else {
    Serial.println(F("Sending failed"));
  }

  // - Time to stop the radio
  radio.powerDown();

  Serial.println(F("----------"));

  // - Delay necessary to serial buffer flush
  delay(50);
}
