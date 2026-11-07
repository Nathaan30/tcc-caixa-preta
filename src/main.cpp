#include <Arduino.h>
#include <Wire.h>

#include "DadosTelemetria.h"
#include "SensorIMU.h"
#include "SensorGPS.h"
#include "AnalisadorSinistro.h"
#include "BufferCircular.h"
#include "ArmazenamentoSD.h"

SensorIMU imu;
SensorGPS gps;
AnalisadorSinistro analisador;
BufferCircular buffer;
ArmazenamentoSD cartaoSD;

// Flag para garantir que o SD só será gravado UMA vez após o acidente
bool sinistroGravado = false; 

const char* textoEstado(EstadoMoto e) {
  switch (e) {
    case NORMAL: return "NORMAL";
    case ALERTA_QUEDA: return "ALERTA_QUEDA";
    default: return "ACIDENTE_CONFIRMADO";
  }
}

void imprimirCabecalhoCSV() {
  Serial.println("Data_Hora,Milissegundos,Acel_X,Acel_Y,Acel_Z,Gyro_X,Gyro_Y,Gyro_Z,G_Total,Giro_Total,Angulo_Lateral,Angulo_Frontal,Movimento_Frontal,Lateral,Frontal,Latitude,Longitude,Velocidade_KMH,GPS_Valido,Estado");
}

void imprimirLinhaCSV(DadosTelemetria d, const char* estado) {
  Serial.print(d.dataHora); Serial.print(",");
  Serial.print(d.milissegundos); Serial.print(",");
  Serial.print(d.ax); Serial.print(",");
  Serial.print(d.ay); Serial.print(",");
  Serial.print(d.az); Serial.print(",");
  Serial.print(d.gx); Serial.print(",");
  Serial.print(d.gy); Serial.print(",");
  Serial.print(d.gz); Serial.print(",");
  Serial.print(d.gTotal); Serial.print(",");
  Serial.print(d.giroTotal); Serial.print(",");
  Serial.print(d.anguloLateral); Serial.print(",");
  Serial.print(d.anguloFrontal); Serial.print(",");
  Serial.print(d.movimentoFrontal); Serial.print(",");
  Serial.print(d.lateralTexto); Serial.print(",");
  Serial.print(d.frontalTexto); Serial.print(",");
  Serial.print(d.latitude, 6); Serial.print(",");
  Serial.print(d.longitude, 6); Serial.print(",");
  Serial.print(d.velocidadeKmh, 1); Serial.print(",");
  Serial.print(d.gpsValido ? "SIM" : "NAO"); Serial.print(",");
  Serial.println(estado);
}

void setup() {
  Serial.begin(115200);

  Serial.println("\n=================================");
  Serial.println("--- CAIXA PRETA: INICIANDO ---");
  Serial.println("=================================");
  
  Wire.begin(21, 22);

  Serial.println("Inicializando Acelerometro...");
  imu.iniciar(0x69);
  Serial.println("Calibrando Acelerometro...");
  imu.calibrar();
  Serial.println("Inicializando GPS...");
  gps.iniciar(16, 17, 9600); // Inicializa UART2 (RX2=16, TX2=17)
  Serial.println("Inicializando Cartão SD...");
  cartaoSD.iniciar(5);
  imprimirCabecalhoCSV();
}

void loop() {
  // 1. Lê a IMU
  DadosTelemetria dados = imu.lerDados();

  // 2. Atualiza e injeta os dados de GPS
  gps.atualizarDados(dados);

  // 2. Se o acidente JÁ FOI GRAVADO, não atualiza o buffer e nem grava mais nada
  if (sinistroGravado) {
    // Opcional: manter apenas um aviso no Serial ou acionar um LED de emergência
    delay(1000); 
    return; // Sai do loop sem alterar a memória
  }

  // 3. Se ainda não houve acidente gravado, atualiza o buffer circular na RAM
  buffer.adicionar(dados);

  // 4. Avalia a física do veículo
  EstadoMoto estado = analisador.verificar(dados, millis());
    imprimirLinhaCSV(dados, textoEstado(estado));

  // 5. Se o acidente for confirmado AGORA, realiza a gravação do buffer
  if (estado == ACIDENTE_CONFIRMADO) {
        Serial.println("--- BUFFER DO SINISTRO ---");
    for (int i = 0; i < buffer.getTamanho(); i++) {
      imprimirLinhaCSV(buffer.obterAmostra(i), "ACIDENTE_CONFIRMADO");
    }
    Serial.println("--- FIM DO BUFFER ---");
    Serial.println("ACIDENTE CONFIRMADO! Gravando histórico no Cartão SD...");
    
    // Grava as 50 amostras dos 10 segundos anteriores
    cartaoSD.salvarLogSinistro(buffer); 
    
    // Marca a flag para bloquear gravações posteriores
    sinistroGravado = true; 
    
    Serial.println("Logs protegidos contra sobrescrita.");
  }

  delay(100); // Taxa de amostragem (5 Hz)
}