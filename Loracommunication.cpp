#include "Loracommunication.h"
#include <LoRa.h>

void initializeLoRa() {
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa initialization failed!");
    while (1);
  }
  Serial.println("LoRa initialized.");
}

void sendLoRaData(String data) {
  LoRa.beginPacket();
  LoRa.print(data);
  LoRa.endPacket();
  Serial.println("Data sent via LoRa: " + data);
}

String receiveLoRaData() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    Serial.println("Data received via LoRa: " + received);
    return received;
  }
  return "";
}
