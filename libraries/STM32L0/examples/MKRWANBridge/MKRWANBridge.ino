/*
  MKRWAN Bridge
  This sketch needs to be flashed on a MKR WAN 1300 to allow using the board as an STM32L0 target.
  Once flashed using Arduino SAMD core, select Arduino MKRWAN 1300 target from STM32L0 core.

  This example code is in the public domain.
*/

int baud = 115200;
uint8_t parity = 0;

void setup() {
  Serial.begin(baud);
  pinMode(LED_BUILTIN, OUTPUT);
}

char rx_buf[512];
char tx_buf[512];

int rx = 0;
int tx = 0;

void doStuff() {
  if (Serial.baud() != baud) {
    baud = Serial.baud();
    if (baud == 2400) {
      digitalWrite(LORA_BOOT0, HIGH);
      pinMode(LORA_BOOT0, OUTPUT);
      digitalWrite(LORA_BOOT0, HIGH);
      pinMode(LORA_RESET, OUTPUT);
      digitalWrite(LORA_RESET, HIGH);
      delay(100);
      digitalWrite(LORA_RESET, LOW);
      delay(100);
      digitalWrite(LORA_RESET, HIGH);
      Serial2.begin(115200, SERIAL_8E1);
      while (Serial2.available()) {
        Serial2.read();
      }
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      Serial2.begin(baud, SERIAL_8N1);
      pinMode(LORA_BOOT0, OUTPUT);
      digitalWrite(LORA_BOOT0, LOW);
      pinMode(LORA_RESET, OUTPUT);
      digitalWrite(LORA_RESET, HIGH);
      delay(100);
      digitalWrite(LORA_RESET, LOW);
      delay(100);
      digitalWrite(LORA_RESET, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void loop() {

  doStuff();

  while (Serial.available()) {      // If anything comes in Serial (USB),
    tx_buf[tx++] = Serial.read();   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (tx > 0) {
    Serial2.write(tx_buf, tx);
    tx = 0;
  }

  while (Serial2.available()) {      // If anything comes in Serial (USB),
    rx_buf[rx++] = Serial2.read();   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (rx > 0) {
    Serial.write(rx_buf, rx);
    rx = 0;
  }


}
