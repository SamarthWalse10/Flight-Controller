#include <PulsePosition.h>

PulsePositionInput ReceiverInput(RISING);

float receiverValues[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int channelNumber = 0;

float inputThrottle;

void readReceiver() {
  channelNumber = ReceiverInput.available();
  if (channelNumber > 0) {
    for (int i = 0; i < channelNumber; i++) {
      receiverValues[i] = ReceiverInput.read(i + 1);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  ReceiverInput.begin(14);

  analogWriteFrequency(1, 50);
  analogWriteResolution(15);
  delay(250);

  while (receiverValues[2] < 1020 || receiverValues[2] > 1050) {
    readReceiver();
    delay(4);
  }
}

void loop() {
  readReceiver();
  inputThrottle = receiverValues[2];
  // double dutyCycle = mapf(inputThrottle, 999.0, 1999.0, 0.00, 4096);
  analogWrite(1, 1.6379*inputThrottle);
  Serial.println(String(inputThrottle) + "  ,  " + String(1.6379*inputThrottle));
  // Serial.println(2.048*inputThrottle);

  // Serial.print("Number of channels: ");
  // Serial.print(channelNumber);
  // Serial.print("\t Roll[μs]: ");
  // Serial.print(receiverValues[0]);
  // Serial.print("\t Pitch[μs]: ");
  // Serial.print(receiverValues[1]);
  // Serial.print("\t Throttle[μs]: ");
  // Serial.print(receiverValues[2]);
  // Serial.print("\t Yaw[μs]: ");
  // Serial.print(receiverValues[3]);
  // Serial.print("\t Aux1[μs]: ");
  // Serial.print(receiverValues[4]);
  // Serial.print("\t Aux2[μs]: ");
  // Serial.println(receiverValues[5]);

  delay(50);
}
