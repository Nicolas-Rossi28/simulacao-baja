#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// --- Configurações de Hardware ---
#define SD_CS 5

// --- Variáveis de Controle de Tempo ---
const unsigned long INTERVALO_LEITURA_US = 1000; // 1000us = 1ms (1kHz)
unsigned long tempoProximaLeitura = 0;
int logCounter = 0;

// --- Instâncias e Arquivo ---
Adafruit_ADXL345_Unified accel1D = Adafruit_ADXL345_Unified(0x1D);
Adafruit_ADXL345_Unified accel53 = Adafruit_ADXL345_Unified(0x53);
File dataFile;
char filename[32];

// --- Matrizes de Calibração (Fieis ao seu modelo inicial) ---
const float Ainv1D[3][3] = {{0.952875, 0.006993, 0.003366}, {0.006993, 0.952172, -0.000777}, {0.003366, -0.000777, 0.987901}};
const float b1D[3]       = {0.094440, -0.361731, 0.911032};

const float Ainv53[3][3] = {{0.970592, -0.048166, 0.008696}, {-0.048166, 0.988076, -0.025318}, {0.008696, -0.025318, 0.984936}};
const float b53[3]       = {0.931123, -0.218200, 0.350909};

void setup() {
  Serial.begin(921600);
  Wire.begin();
  Wire.setClock(400000); // I2C Fast Mode

  // 1. Inicialização Sensores
  if(!accel1D.begin() || !accel53.begin()) {
    Serial.println("Erro nos ADXLs (verifique endereços 0x1D e 0x53)!");
    while(1);
  }
  
  accel1D.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel1D.setRange(ADXL345_RANGE_16_G);
  accel53.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel53.setRange(ADXL345_RANGE_16_G);

  // 2. Inicialização SD e Nome Dinâmico
  if (!SD.begin(SD_CS)) {
    Serial.println("Erro no SD!");
    while(1);
  }

  sprintf(filename, "/log_%u.csv", micros());
  dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    Serial.printf("Gravando em: %s\n", filename);
    dataFile.println("us;x1;y1;z1;x2;y2;z2"); // Cabeçalho
  }

  tempoProximaLeitura = micros();
}

void loop() {
  // Controle de tempo por polling (micros)
  if (micros() >= tempoProximaLeitura) {
    tempoProximaLeitura += INTERVALO_LEITURA_US;

    sensors_event_t ev1, ev2;
    accel1D.getEvent(&ev1);
    accel53.getEvent(&ev2);

    // Variáveis auxiliares para calibração (f = h - b)
    float f1[3] = {ev1.acceleration.x - b1D[0], ev1.acceleration.y - b1D[1], ev1.acceleration.z - b1D[2]};
    float f2[3] = {ev2.acceleration.x - b53[0], ev2.acceleration.y - b53[1], ev2.acceleration.z - b53[2]};

    if (dataFile) {
      // Escreve Timestamp
      dataFile.print(micros()); dataFile.print(";");

      // Sensor 1 Calibrado (Ainv * f)
      dataFile.print(Ainv1D[0][0]*f1[0] + Ainv1D[0][1]*f1[1] + Ainv1D[0][2]*f1[2], 3); dataFile.print(";");
      dataFile.print(Ainv1D[1][0]*f1[0] + Ainv1D[1][1]*f1[1] + Ainv1D[1][2]*f1[2], 3); dataFile.print(";");
      dataFile.print(Ainv1D[2][0]*f1[0] + Ainv1D[2][1]*f1[1] + Ainv1D[2][2]*f1[2], 3); dataFile.print(";");

      // Sensor 2 Calibrado (Ainv * f)
      dataFile.print(Ainv53[0][0]*f2[0] + Ainv53[0][1]*f2[1] + Ainv53[0][2]*f2[2], 3); dataFile.print(";");
      dataFile.print(Ainv53[1][0]*f2[0] + Ainv53[1][1]*f2[1] + Ainv53[1][2]*f2[2], 3); dataFile.print(";");
      dataFile.println(Ainv53[2][0]*f2[0] + Ainv53[2][1]*f2[1] + Ainv53[2][2]*f2[2], 3);

      logCounter++;
    }

    // Flush a cada 500 amostras (aprox. 0.5s) para segurança
    if (logCounter >= 500) {
      dataFile.flush();
      logCounter = 0;
    }
  }
}
