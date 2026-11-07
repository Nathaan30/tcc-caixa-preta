#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include "DadosTelemetria.h"

// ---------------------------------------------------------------------------
// MONTAGEM DA PLACA
//
// Informe qual eixo do sensor MPU6050 aponta para a FRENTE da moto.
// Os eixos X e Y estao desenhados no modulo; o Z sai da face dos componentes.
//
//   {0, 0, 1}  -> frente = +Z   (padrao)
//   {0, 0,-1}  -> frente = -Z
//   {0, 1, 0}  -> frente = +Y
//   {1, 0, 0}  -> frente = +X   (e assim por diante)
//
// Se, depois de calibrar, inclinar para a ESQUERDA aparecer como "Direita"
// (ou o contrario), inverta o sinal do eixo (ex.: {0,0,1} -> {0,0,-1}).
// ---------------------------------------------------------------------------
const float EIXO_FRENTE[3] = {0.0f, 0.0f, 1.0f};

class SensorIMU {
  private:
    Adafruit_MPU6050 mpu;

    // Referencia da calibracao, em vetores unitarios nos eixos da PLACA:
    //   cima0    = oposto da gravidade quando calibrou ("Reto")
    //   frente0  = frente da moto, perpendicular a cima0
    //   direita0 = direita da moto  (frente0 x cima0)
    float cima0[3]    = {0.0f, 0.0f, 1.0f};
    float frente0[3]  = {0.0f, 1.0f, 0.0f};
    float direita0[3] = {1.0f, 0.0f, 0.0f};

    float axFiltrado = 0, ayFiltrado = 0, azFiltrado = 0;
    float gxFiltrado = 0, gyFiltrado = 0, gzFiltrado = 0;
    bool filtroInicializado = false;

    const float ALPHA_ACEL = 0.20;
    const float ALPHA_GIRO = 0.20;
    const float RAD_PARA_GRAUS = 57.2957795;
    const float GRAVIDADE = 9.80665;

    // Calibracao: variacao maxima aceita (m/s2) em cada eixo durante a coleta.
    // Acima disso a placa esta sendo mexida e a coleta e repetida.
    const float LIMITE_MOVIMENTO_CALIBRACAO = 0.8;
    const int MAX_TENTATIVAS_CALIBRACAO = 5;

    static float produtoEscalar(const float a[3], const float b[3]) {
      return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    static void produtoVetorial(const float a[3], const float b[3], float saida[3]) {
      saida[0] = a[1] * b[2] - a[2] * b[1];
      saida[1] = a[2] * b[0] - a[0] * b[2];
      saida[2] = a[0] * b[1] - a[1] * b[0];
    }

    static bool normalizar(float v[3]) {
      float n = sqrtf(produtoEscalar(v, v));
      if (n < 1e-6f) return false;
      v[0] /= n; v[1] /= n; v[2] /= n;
      return true;
    }

    // Tira de "eixo" a parte paralela a cima0 e normaliza. Falha se o eixo
    // estiver quase alinhado com a gravidade (nao daria para saber a frente).
    bool montarFrente(const float eixo[3]) {
      float p = produtoEscalar(eixo, cima0);
      float f[3] = {eixo[0] - p * cima0[0], eixo[1] - p * cima0[1], eixo[2] - p * cima0[2]};
      if (sqrtf(produtoEscalar(f, f)) < 0.3f) return false;
      normalizar(f);
      frente0[0] = f[0]; frente0[1] = f[1]; frente0[2] = f[2];
      return true;
    }

  public:
    bool iniciar(uint8_t endereco) {
      if (!mpu.begin(endereco)) return false;
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      return true;
    }

    // Deixe a moto/placa PARADA e na posicao normal de uso (moto em pe, sobre as
    // duas rodas, nao no descanso lateral). Essa posicao passa a ser o "Reto".
    void calibrar() {
      const int AMOSTRAS = 200;
      sensors_event_t a, g, temp;
      float soma[3], minimo[3], maximo[3];
      bool estavel = false;

      for (int tentativa = 1; tentativa <= MAX_TENTATIVAS_CALIBRACAO && !estavel; tentativa++) {
        for (int k = 0; k < 3; k++) { soma[k] = 0; minimo[k] = 1e9f; maximo[k] = -1e9f; }

        for (int i = 0; i < AMOSTRAS; i++) {
          mpu.getEvent(&a, &g, &temp);
          float v[3] = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
          for (int k = 0; k < 3; k++) {
            soma[k] += v[k];
            if (v[k] < minimo[k]) minimo[k] = v[k];
            if (v[k] > maximo[k]) maximo[k] = v[k];
          }
          delay(10);
        }

        float maiorVariacao = 0;
        for (int k = 0; k < 3; k++) {
          float variacao = maximo[k] - minimo[k];
          if (variacao > maiorVariacao) maiorVariacao = variacao;
        }
        estavel = (maiorVariacao <= LIMITE_MOVIMENTO_CALIBRACAO);

        if (!estavel) {
          Serial.print("Placa em movimento na calibracao (variacao ");
          Serial.print(maiorVariacao);
          Serial.print(" m/s2). Segure parada. Tentativa ");
          Serial.print(tentativa);
          Serial.print(" de ");
          Serial.println(MAX_TENTATIVAS_CALIBRACAO);
          delay(500);
        }
      }
      if (!estavel) Serial.println("AVISO: calibracao com a placa instavel. Reinicie e segure parada.");

      float base[3] = {soma[0] / AMOSTRAS, soma[1] / AMOSTRAS, soma[2] / AMOSTRAS};
      cima0[0] = base[0]; cima0[1] = base[1]; cima0[2] = base[2];
      if (!normalizar(cima0)) {
        Serial.println("ERRO: sem leitura de gravidade na calibracao. Confira o sensor.");
        return;
      }

      // Frente: usa EIXO_FRENTE; se estiver alinhado com a gravidade, tenta os outros eixos.
      const float candidatos[3][3] = {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}};
      if (!montarFrente(EIXO_FRENTE)) {
        Serial.println("AVISO: EIXO_FRENTE esta alinhado com a gravidade. Usando outro eixo; confira a montagem.");
        for (int c = 0; c < 3; c++) {
          if (montarFrente(candidatos[c])) break;
        }
      }
      produtoVetorial(frente0, cima0, direita0);

      Serial.print("Calibrado | gravidade=");
      Serial.print(sqrtf(produtoEscalar(base, base)));
      Serial.println(" m/s2");
      Serial.print("  cima=(");    Serial.print(cima0[0]);    Serial.print(", "); Serial.print(cima0[1]);    Serial.print(", "); Serial.print(cima0[2]);    Serial.println(")");
      Serial.print("  frente=(");  Serial.print(frente0[0]);  Serial.print(", "); Serial.print(frente0[1]);  Serial.print(", "); Serial.print(frente0[2]);  Serial.println(")");
      Serial.print("  direita=("); Serial.print(direita0[0]); Serial.print(", "); Serial.print(direita0[1]); Serial.print(", "); Serial.print(direita0[2]); Serial.println(")");

      axFiltrado = base[0];
      ayFiltrado = base[1];
      azFiltrado = base[2];
      filtroInicializado = true;
    }

    DadosTelemetria lerDados() {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      if (!filtroInicializado) {
        axFiltrado = a.acceleration.x; ayFiltrado = a.acceleration.y; azFiltrado = a.acceleration.z;
        gxFiltrado = g.gyro.x; gyFiltrado = g.gyro.y; gzFiltrado = g.gyro.z;
        filtroInicializado = true;
      } else {
        axFiltrado = ALPHA_ACEL * a.acceleration.x + (1.0 - ALPHA_ACEL) * axFiltrado;
        ayFiltrado = ALPHA_ACEL * a.acceleration.y + (1.0 - ALPHA_ACEL) * ayFiltrado;
        azFiltrado = ALPHA_ACEL * a.acceleration.z + (1.0 - ALPHA_ACEL) * azFiltrado;

        gxFiltrado = ALPHA_GIRO * g.gyro.x + (1.0 - ALPHA_GIRO) * gxFiltrado;
        gyFiltrado = ALPHA_GIRO * g.gyro.y + (1.0 - ALPHA_GIRO) * gyFiltrado;
        gzFiltrado = ALPHA_GIRO * g.gyro.z + (1.0 - ALPHA_GIRO) * gzFiltrado;
      }

      DadosTelemetria d;
      d.ax = a.acceleration.x; d.ay = a.acceleration.y; d.az = a.acceleration.z;
      d.gx = g.gyro.x; d.gy = g.gyro.y; d.gz = g.gyro.z;

      float acelTotal = sqrt((d.ax * d.ax) + (d.ay * d.ay) + (d.az * d.az));
      d.gTotal = acelTotal / GRAVIDADE;
      d.giroTotal = sqrt((gxFiltrado * gxFiltrado) + (gyFiltrado * gyFiltrado) + (gzFiltrado * gzFiltrado));

      // Angulos em relacao a posicao calibrada ("Reto"), em graus.
      //   anguloLateral: positivo = inclinando para a ESQUERDA, negativo = DIREITA
      //   anguloFrontal: positivo = frente para cima, negativo = frente para baixo
      // Usa os tres eixos e a referencia 3D: o lateral nao satura em 90 graus
      // (moto deitada = ~90) e a inclinacao de um lado nao "vaza" para o outro.
      float v[3] = {axFiltrado, ayFiltrado, azFiltrado};
      float n = sqrtf(produtoEscalar(v, v));
      if (n < 0.01f) {
        d.anguloLateral = 0;
        d.anguloFrontal = 0;
      } else {
        float xCima = produtoEscalar(v, cima0) / n;
        float xDireita = produtoEscalar(v, direita0) / n;
        float xFrente = produtoEscalar(v, frente0) / n;
        d.anguloLateral = atan2f(xDireita, xCima) * RAD_PARA_GRAUS;   // -180 a +180
        d.anguloFrontal = atan2f(xFrente, sqrtf(xDireita * xDireita + xCima * xCima)) * RAD_PARA_GRAUS;   // -90 a +90
      }
      d.movimentoFrontal = produtoEscalar(v, frente0);

      if (d.anguloLateral > 25.0) d.lateralTexto = "Esquerda forte";
      else if (d.anguloLateral > 10.0) d.lateralTexto = "Esquerda";
      else if (d.anguloLateral < -25.0) d.lateralTexto = "Direita forte";
      else if (d.anguloLateral < -10.0) d.lateralTexto = "Direita";
      else d.lateralTexto = "Reto";

      if (d.movimentoFrontal > 1.2) d.frontalTexto = "Acelerando";
      else if (d.movimentoFrontal < -1.2) d.frontalTexto = "Freando";
      else d.frontalTexto = "Estavel";

      d.milissegundos = millis();
      strcpy(d.dataHora, "SEM_RTC");

      return d;
    }
};

#endif