/*
  Read all range sensors connected to the Arduino analog ports and send through serial port.
  We let any conversion or postprocessing to the reading PC, so we just pass the 0..1023 raw
  values read. For sonars, we also trigger readings by pulling HIGH their RX PIN for 20 uS.
 */

int IR_COUNT = 4;
int HS_COUNT = 11;
int inputPin[] = {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 15, 14, 13, 12 };
int triggPin[] = { 48, 44, 40, 36, 32, 28, 22, 35, 47, 43, 39 };
byte outBuffer[sizeof(inputPin) + 2];


void readAnalog(int index)
{
  // read and pack the obtained value as two bytes,
  // always skipping the first two "protocol" bytes
  int ar = analogRead(A0 + inputPin[index]);
  outBuffer[(index+1)*2+1] = (byte)((ar >> 8) & 0xFF);
  outBuffer[(index+1)*2]   = (byte)(ar & 0xFF) ;
}


void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  // we use port registers to trigger all sonars at the same time
  // compose the masks (see link below) with the PIN indexes and initialize at LOW
  // http://stackoverflow.com/questions/21417521/arduino-use-all-ports-atmega2560
  DDRA = B01000001;
  DDRC = B00100110;
  DDRG = B00000110;
  DDRL = B01100110;
  
  PORTA = DDRA & 0x00;
  PORTC = DDRC & 0x00;
  PORTG = DDRG & 0x00;
  PORTL = DDRL & 0x00;

  // set the first two bytes to signal the beginning of a readings set; note that our
  // protocol is really crude... we should avoid writing faster than the PC reads!
  outBuffer[0] = 0xFF;
  outBuffer[1] = 0xFF;
}

void loop() {
  // trigger sonars reading
  PORTA = DDRA & 0xFF;
  PORTC = DDRC & 0xFF;
  PORTG = DDRG & 0xFF;
  PORTL = DDRL & 0xFF;
  
  delayMicroseconds(20);

  PORTA = DDRA & 0x00;
  PORTC = DDRC & 0x00;
  PORTG = DDRG & 0x00;
  PORTL = DDRL & 0x00;
  
  // delay a bit less than the sensors period, to compensate the wasted time on reading
  delay(46);

  // read all analog input PINs, both sonars and IR sensors that need not triggering
  for (int i = 0; i < HS_COUNT + IR_COUNT; i++)
    readAnalog(i);

  // write buffer to serial port
  Serial.write(outBuffer, sizeof(outBuffer));
  Serial.flush();
}

