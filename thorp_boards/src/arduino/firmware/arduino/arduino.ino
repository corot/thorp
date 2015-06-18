/*
  Read all range sensors connected to the Arduino analog ports and send through serial port.
  We let any conversion or postprocessing to the reading PC, so we just pass the 0..1023 raw
  values read. For sonars, we also trigger readings by pulling HIGH their RX PIN for 20 uS.
 */

int IR_COUNT = 4;
int HS_COUNT = 11;
int inputPin[] = {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 15, 14, 13, 12 };
int triggPin[] = { 22, 28, 32, 36, 40, 44, 48, 39, 43, 47, 51 };
byte outBuffer[sizeof(inputPin) + 2];


void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  // initialize all trigger PINs
  for (int i = 0; i < HS_COUNT; i++)
    pinMode(triggPin[i], OUTPUT);

  // set the first two bytes to signal the beginning of a readings set; note that our
  // protocol is really crude... we should avoid writing faster than the PC reads!
  outBuffer[0] = 0xFF;
  outBuffer[1] = 0xFF;
}

void loop() {
  // trigger sonars reading
  for (int i = 0; i < HS_COUNT; i++)
    digitalWrite(triggPin[i], HIGH);
  delayMicroseconds(20);
  for (int i = 0; i < HS_COUNT; i++)
    digitalWrite(triggPin[i], LOW);

  // delay a bit less than the sensors period, to compensate the wasted time on reading
  delay(50);

  // read all analog input PINs and pack the obtained value as two bytes, skipping the
  // first two "protocol" bytes
  for (int i = 0; i < HS_COUNT + IR_COUNT; i++) {
    int ar = analogRead(A0 + inputPin[i]);
    outBuffer[(i+1)*2+1] = (byte)((ar >> 8) & 0xFF);
    outBuffer[(i+1)*2]   = (byte)(ar & 0xFF) ;
  }
  // write buffer to serial port
  Serial.write(outBuffer, sizeof(outBuffer));
  Serial.flush();
}
