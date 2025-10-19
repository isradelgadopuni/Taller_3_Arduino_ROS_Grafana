#define DHT_PIN 2

void setup() {
  Serial.begin(9600);
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, HIGH);
  delay(1000);
}

void loop() {
  byte data[5] = {0, 0, 0, 0, 0};

  // Señal de inicio al DHT11
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, LOW);
  delay(18);
  digitalWrite(DHT_PIN, HIGH);
  delayMicroseconds(30);
  pinMode(DHT_PIN, INPUT);

  // Esperar respuesta del sensor
  unsigned long start = micros();
  while (digitalRead(DHT_PIN) == HIGH)
    if (micros() - start > 100) return;

  // Leer 40 bits (5 bytes)
  for (int i = 0; i < 40; i++) {
    while (digitalRead(DHT_PIN) == LOW);
    unsigned long t = micros();
    while (digitalRead(DHT_PIN) == HIGH);
    if ((micros() - t) > 40)
      data[i / 8] |= (1 << (7 - (i % 8)));
  }

  // Enviar solo la temperatura entera cruda (byte 3)
  Serial.println(data[2]);   // <-- solo el número
  delay(2000);
}
