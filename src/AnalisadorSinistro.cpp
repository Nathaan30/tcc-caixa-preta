#include "AnalisadorSinistro.h"

AnalisadorSinistro::AnalisadorSinistro() {
  estadoAtual = NORMAL;
  tempoInicioQueda = 0;
  tempoInicioRecuperacao = 0;
  impactoDuranteAlerta = false;
}

EstadoMoto AnalisadorSinistro::verificar(DadosTelemetria dados, unsigned long tempoAtual) {
  // 1. Processa a maior inclinação
  float absLateral = fabs(dados.anguloLateral);
  float absFrontal = fabs(dados.anguloFrontal);
  float maiorInclinacao = (absFrontal > absLateral) ? absFrontal : absLateral;

  // 2. Avalia condições físicas
  bool inclinacaoSuspeita = maiorInclinacao > LIMITE_ANGULO_QUEDA_ALERTA;
  bool inclinacaoConfirmada = maiorInclinacao > LIMITE_ANGULO_QUEDA_CONFIRMAR;
  bool recuperouPosicao = maiorInclinacao < LIMITE_ANGULO_RECUPERAR;
  bool impactoForte = dados.gTotal > LIMITE_IMPACTO_G;
  bool giroBaixo = dados.giroTotal < LIMITE_GIRO_PARADO;

  // 3. Máquina de Estados
  switch (estadoAtual) {
    case NORMAL:
      if (inclinacaoSuspeita || impactoForte) {
        estadoAtual = ALERTA_QUEDA;
        tempoInicioQueda = tempoAtual;
        tempoInicioRecuperacao = 0;
        impactoDuranteAlerta = impactoForte;
      }
      break;

    case ALERTA_QUEDA:
     // Serial.println("Estado: ALERTA_QUEDA");
      if (impactoForte) impactoDuranteAlerta = true;

      // Validação de falso positivo (recuperação)
      if (recuperouPosicao) {
        if (tempoInicioRecuperacao == 0) tempoInicioRecuperacao = tempoAtual;
        
        if (tempoAtual - tempoInicioRecuperacao >= TEMPO_RECUPERACAO) {
          estadoAtual = NORMAL;
          impactoDuranteAlerta = false;
        }
      } else {
        tempoInicioRecuperacao = 0; // Zera se voltar a inclinar
      }

      // Validação de confirmação de sinistro
      if (estadoAtual == ALERTA_QUEDA) {
        unsigned long tempoEmAlerta = tempoAtual - tempoInicioQueda;
        bool evidenciaForte = inclinacaoConfirmada || (impactoDuranteAlerta && inclinacaoSuspeita);

        if (tempoEmAlerta >= TEMPO_CONFIRMAR_QUEDA && evidenciaForte && giroBaixo) {
          estadoAtual = ACIDENTE_CONFIRMADO;
        } 
        else if (tempoEmAlerta >= TEMPO_CONFIRMAR_MAXIMO && inclinacaoConfirmada) {
          estadoAtual = ACIDENTE_CONFIRMADO;
        }
      }
      break;

    case ACIDENTE_CONFIRMADO:
      // O estado final é travado. Caberá ao arquivo main.cpp ler isso 
      // e disparar o salvamento definitivo no SD.
      break;
  }

  return estadoAtual;
}

EstadoMoto AnalisadorSinistro::getEstadoAtual() {
  return estadoAtual;
}