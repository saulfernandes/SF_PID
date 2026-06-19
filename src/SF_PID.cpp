#include "Arduino.h"
#include "SF_PID.h"

SF_PID::SF_PID() {}

SF_PID::SF_PID(float* Entrada, float* Saida, float* Setpoint,
               float Kp, float Ki, float Kd, Acao acao_) {
  minhaSaida = Saida;
  minhaEntrada = Entrada;
  meuSetpoint = Setpoint;
  modo = Controle::automatico;
  SF_PID::DefinirLimitesSaida(0, 255);  
  tempoAmostragemUs = 100000;           
  SF_PID::DefinirDirecao(acao_);
  SF_PID::DefinirAjustes(Kp, Ki, Kd);
  primeiraLeitura = true;
  ultimoTempo = micros() - tempoAmostragemUs;
}

SF_PID::SF_PID(float* Entrada, float* Saida, float* Setpoint)
  : SF_PID::SF_PID(Entrada, Saida, Setpoint, 0, 0, 0, Acao::direto) {}

/* Algoritmos de Filtragem e Condicionamento (Ocultados por brevidade, MANTENHA OS SEUS ATUAIS AQUI) */
// [Nota: Mantenha as funções AplicarFiltro, ProcessarEntrada, DefinirCoeficientes e ConfigurarFiltro exatamente como forneci no bloco anterior. Nao foram alteradas.]

float SF_PID::AplicarFiltro(float valor) {
  if (primeiraLeitura) {
    ultimoValorEMA = valor;
    estimativaKalman = valor;
    for (int i = 0; i < 15; i++) bufferMediana[i] = valor;
    primeiraLeitura = false;
  }
  if (modoFiltro == Filtro::nenhum) return valor;
  if (modoFiltro == Filtro::emaAdaptativo) {
    float variacao = abs(valor - ultimoValorEMA);
    float proporcao = variacao / rangeVar; 
    float alpha = alphaMin + (alphaMax - alphaMin) * proporcao;
    alpha = constrain(alpha, alphaMin, alphaMax);
    ultimoValorEMA = (alpha * valor) + ((1.0f - alpha) * ultimoValorEMA);
    return ultimoValorEMA;
  }
  if (modoFiltro == Filtro::mediana) {
    bufferMediana[indiceMediana] = valor;
    indiceMediana = (indiceMediana + 1) % 15;
    float temp[15];
    for (int i = 0; i < 15; i++) temp[i] = bufferMediana[i];
    for (int i = 0; i < 14; i++) {
      for (int j = 0; j < 14 - i; j++) {
        if (temp[j] > temp[j + 1]) {
          float t = temp[j]; temp[j] = temp[j + 1]; temp[j + 1] = t;
        }
      }
    }
    return temp[7];
  }
  if (modoFiltro == Filtro::kalman1D) {
    erroKalman += qProcesso;
    float K = erroKalman / (erroKalman + rMedida);
    estimativaKalman += K * (valor - estimativaKalman);
    erroKalman *= (1.0f - K);
    return estimativaKalman;
  }
  return valor;
}

float SF_PID::ProcessarEntrada(float valorLido) {
  if (modoEntrada == Entrada::filtrada) return valorLido; 
  if (modoEntrada == Entrada::temperatura) return AplicarFiltro(valorLido); 
  if (modoEntrada == Entrada::pura) {
    float sinalLimpo = AplicarFiltro(valorLido);
    return (coefA * sinalLimpo * sinalLimpo) + (coefB * sinalLimpo) + coefC + offsetSinal;
  }
  return valorLido;
}

void SF_PID::DefinirEntrada(DfEntrada tipoEntrada) { modoEntrada = tipoEntrada; }
void SF_PID::DefinirEntrada(uint8_t tipoEntrada) { modoEntrada = (Entrada)tipoEntrada; }
void SF_PID::DefinirCoeficientes(float a, float b, float c, float offset) { coefA = a; coefB = b; coefC = c; offsetSinal = offset; }
void SF_PID::DefinirFiltro(Filtro tipoFiltro) { modoFiltro = tipoFiltro; primeiraLeitura = true; }
void SF_PID::DefinirFiltro(uint8_t tipoFiltro) { modoFiltro = (Filtro)tipoFiltro; primeiraLeitura = true; }
void SF_PID::ConfigurarFiltroEMA(float minAlpha, float maxAlpha, float rangeVariacao) { alphaMin = minAlpha; alphaMax = maxAlpha; rangeVar = rangeVariacao; }
void SF_PID::ConfigurarFiltroKalman(float ruidoMedida, float ruidoProcesso) { rMedida = ruidoMedida; qProcesso = ruidoProcesso; }


/* =================================================================================
   Configuração da SINTONIA (AutoTune)
================================================================================== */
void SF_PID::DefinirModoSintonia(Sintonia modo) { modoSintonia = modo; }
void SF_PID::DefinirModoSintonia(uint8_t modo) { modoSintonia = (Sintonia)modo; }

void SF_PID::LigarSintonia() {
  if (modoSintonia == Sintonia::desligado) return;
  sintoniaLigada = true;
  
  // Reseta todas as memorias de sintonia
  tuneMax = -9999.0f; tuneMin = 9999.0f;
  tuneCiclos = 0; tuneSomaPeriodo = 0;
  tuneTempoReferencia = millis();
  tuneUltimoCruzamento = millis();
  
  if(modoSintonia == Sintonia::heuristica) {
    heurEstado = 0; // Inicia Maquina de Estados
  }
  if(modoSintonia == Sintonia::self || modoSintonia == Sintonia::zn_self || modoSintonia == Sintonia::tl_self) {
    selfAtuacoes = 0;
  }
}

void SF_PID::DesligarSintonia() { sintoniaLigada = false; }
bool SF_PID::SintoniaAtiva() { return sintoniaLigada; }

/* =================================================================================
   O CÉREBRO: Calcular() com Bifurcação de Sintonia
================================================================================== */
bool SF_PID::Calcular() {
  if (modo == Controle::manual) return false;
  uint32_t agora = micros();
  uint32_t variacaoTempo = (agora - ultimoTempo);
  
  if (variacaoTempo >= tempoAmostragemUs) {
    float entrada = ProcessarEntrada(*minhaEntrada);
    
    // ROTA A: SINTONIA RELÉ (Bang-Bang Exclusivo)
    bool isRelay = (modoSintonia == Sintonia::zn || modoSintonia == Sintonia::tl || 
                    modoSintonia == Sintonia::zn_self || modoSintonia == Sintonia::tl_self);
                    
    if (sintoniaLigada && isRelay) {
        ExecutarRelay(entrada);
        *minhaEntrada = entrada;
        ultimoTempo = agora;
        return true; // Pula a matemática PID, o Rele tomou controle da maquina.
    }

    // ROTA B: CONTROLE PID NORMAL (Para Controle Real, Heuristica e Self)
    float dEntrada = entrada - ultimaEntrada;
    if (acao == Acao::reverso) dEntrada = -dEntrada;

    erro = *meuSetpoint - entrada;
    if (acao == Acao::reverso) erro = -erro;
    float dErro = erro - ultimoErro;

    float termoPm = kp * dEntrada;
    termoP = -termoPm; 
    termoI = ki * erro;
    termoD = -kd * dEntrada; 

    bool aw = false;
    float saidaTermoI = (-termoPm) + ki * (termoI + erro);
    if (saidaTermoI > saidaMax && dErro > 0) aw = true;
    else if (saidaTermoI < saidaMin && dErro < 0) aw = true;
    if (aw && ki > 0) termoI = constrain(saidaTermoI, -saidaMax, saidaMax);

    somaSaida += termoI;                                                 
    somaSaida = constrain(somaSaida - termoPm, saidaMin, saidaMax); 
    *minhaSaida = constrain(somaSaida + termoD, saidaMin, saidaMax); 

    ultimoErro = erro; ultimaEntrada = entrada; ultimoTempo = agora;
    *minhaEntrada = entrada;

    // ROTA C: SUPERVISORES (Rondam em background junto com o PID)
    if (sintoniaLigada) {
        if (modoSintonia == Sintonia::heuristica) ExecutarHeuristica(entrada);
        else if (modoSintonia == Sintonia::self) ExecutarSelf(entrada);
    }
    
    return true;
  }
  return false;
}

/* =================================================================================
   MOTORES DE INTELIGÊNCIA
================================================================================== */

void SF_PID::ExecutarRelay(float entradaLida) {
    if (entradaLida > tuneMax) tuneMax = entradaLida;
    if (entradaLida < tuneMin) tuneMin = entradaLida;

    // Histerese para evitar repiques de ruidos no zero
    bool cruzouCima = (entradaLida > *meuSetpoint + 0.1f);
    bool cruzouBaixo = (entradaLida < *meuSetpoint - 0.1f);

    if (cruzouBaixo && !estadoRele) {
        estadoRele = true;
        *minhaSaida = saidaMax;
        
        uint32_t agoraMil = millis();
        tuneSomaPeriodo += (agoraMil - tuneUltimoCruzamento);
        tuneUltimoCruzamento = agoraMil;
        tuneCiclos++;
    } 
    else if (cruzouCima && estadoRele) {
        estadoRele = false;
        *minhaSaida = saidaMin;
    }

    // Atingiu 8 Ciclos Reais (Requisito)
    if (tuneCiclos >= 8) {
        float Tu = (tuneSomaPeriodo / 8.0f) / 1000.0f; // Em segundos
        float amplitude = (tuneMax - tuneMin) / 2.0f;
        float Ku = (4.0f * (saidaMax - saidaMin)) / (3.14159f * amplitude);

        float nKp = 0, nKi = 0, nKd = 0;

        if (modoSintonia == Sintonia::zn || modoSintonia == Sintonia::zn_self) {
            nKp = 0.6f * Ku;
            nKi = (2.0f * nKp) / Tu;
            nKd = (nKp * Tu) / 8.0f;
        } else { // TL
            nKp = 0.31f * Ku;
            nKi = nKp / (2.2f * Tu);
            nKd = (nKp * Tu) / 6.3f;
        }
        
        DefinirAjustes(nKp, nKi, nKd);

        if (modoSintonia == Sintonia::zn_self || modoSintonia == Sintonia::tl_self) {
            modoSintonia = Sintonia::self; // Troca automaticamente
            LigarSintonia(); // Reinicia memorias pro self
        } else {
            DesligarSintonia(); // Finalizou puro ZN/TL
        }
    }
}

void SF_PID::ExecutarHeuristica(float entradaLida) {
    uint32_t agoraMil = millis();
    if (entradaLida > tuneMax) tuneMax = entradaLida;
    if (entradaLida < tuneMin) tuneMin = entradaLida;

    // Estado 0: Inicializacao Manual
    if (heurEstado == 0) {
        DefinirAjustes(50.0f, 0.0f, 0.0f); // Inicia Kp em 50
        passoKp = 20.0f; passoKi = 0.0025f; passoKd = 0.001f;
        heurEstado = 1;
        tuneTempoReferencia = agoraMil;
        return;
    }

    // Estado 1: Kp (Espera 15 min = 900.000 ms)
    if (heurEstado == 1 && (agoraMil - tuneTempoReferencia >= 900000)) {
        float variacao = tuneMax - tuneMin;
        
        if (variacao > 0.5f) { 
            // Oscilou! Recua e diminui o passo.
            DefinirAjustes(dispKp - passoKp, 0, 0);
            passoKp /= 4.0f; // Reduz de 20 para 5.
            if (passoKp < 2.0f) { heurEstado = 2; } // Equilibrio perfeito atingido
        } else {
            // Estavel, continua subindo
            DefinirAjustes(dispKp + passoKp, 0, 0);
        }
        tuneMax = -9999; tuneMin = 9999;
        tuneTempoReferencia = agoraMil;
    }

    // Estado 2: Ki (Espera 30 min = 1.800.000 ms)
    if (heurEstado == 2 && (agoraMil - tuneTempoReferencia >= 1800000)) {
        float erroAtual = abs(*meuSetpoint - entradaLida);
        if (erroAtual <= 0.2f) {
            heurEstado = 3; // Chegou no alvo, vamos pro derivativo
        } else {
            DefinirAjustes(dispKp, dispKi + passoKi, 0);
        }
        tuneMax = -9999; tuneMin = 9999;
        tuneTempoReferencia = agoraMil;
    }

    // Estado 3: Kd (Espera 10 min = 600.000 ms)
    if (heurEstado == 3 && (agoraMil - tuneTempoReferencia >= 600000)) {
        float erroAcima = entradaLida - *meuSetpoint;
        if (erroAcima > 0.5f) { // Teve overshoot grave
            DefinirAjustes(dispKp, dispKi, dispKd + passoKd);
            tuneTempoReferencia = agoraMil;
        } else {
            DesligarSintonia(); // SINTONIA CONCLUIDA COM SUCESSO!
        }
    }
}

void SF_PID::ExecutarSelf(float entradaLida) {
    if (entradaLida > tuneMax) tuneMax = entradaLida;
    if (entradaLida < tuneMin) tuneMin = entradaLida;

    // Detecta Ciclos cruzando o Setpoint
    if (entradaLida > *meuSetpoint && !cruzouSP) { cruzouSP = true; tuneCiclos++; }
    if (entradaLida < *meuSetpoint && cruzouSP)  { cruzouSP = false; }

    // Avalia a cada 5 ciclos confirmados
    if (tuneCiclos >= 5) {
        float variacao = tuneMax - tuneMin;
        float erroAtual = abs(*meuSetpoint - entradaLida);
        
        bool atuou = false;

        if (variacao > 1.0f) { // Amortecendo oscilacao
            DefinirAjustes(dispKp * 0.90f, dispKi, dispKd * 1.10f);
            atuou = true;
        } else if (erroAtual > 1.0f) { // Longe do Alvo
            DefinirAjustes(dispKp * 1.15f, dispKi, dispKd);
            atuou = true;
        } else if (erroAtual > 0.2f && erroAtual <= 1.0f) { // Offset final
            DefinirAjustes(dispKp, dispKi * 1.08f, dispKd);
            atuou = true;
        }

        tuneMax = -9999; tuneMin = 9999;
        tuneCiclos = 0;
        
        if (atuou) {
            selfAtuacoes++;
            if (selfAtuacoes >= 20) DesligarSintonia(); // Criterio de Fim
        } else {
            DesligarSintonia(); // Finalizou por estabilidade perfeita
        }
    }
}

/* =================================================================================
   Funções Padrão de Configuração Restantes (Ajustes, Tempo, Limites)
================================================================================== */
void SF_PID::DefinirAjustes(float Kp, float Ki, float Kd) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;
  if (Ki == 0) somaSaida = 0;
  dispKp = Kp; dispKi = Ki; dispKd = Kd;
  float tempoAmostragemSeg = (float)tempoAmostragemUs / 1000000;
  kp = Kp; ki = Ki * tempoAmostragemSeg; kd = Kd / tempoAmostragemSeg;
}

void SF_PID::DefinirTempoAmostragemUs(uint32_t NovoTempoAmostragemUs) {
  if (NovoTempoAmostragemUs > 0) {
    float razao = (float)NovoTempoAmostragemUs / (float)tempoAmostragemUs;
    ki *= razao; kd /= razao; tempoAmostragemUs = NovoTempoAmostragemUs;
  }
}

void SF_PID::DefinirLimitesSaida(float Min, float Max) {
  if (Min >= Max) return;
  saidaMin = Min; saidaMax = Max;
  if (modo != Controle::manual) {
    *minhaSaida = constrain(*minhaSaida, saidaMin, saidaMax);
    somaSaida = constrain(somaSaida, saidaMin, saidaMax);
  }
}

void SF_PID::DefinirModo(Controle novoModo) {
  if (modo == Controle::manual && novoModo == Controle::automatico) SF_PID::Inicializar();
  modo = novoModo;
}
void SF_PID::DefinirModo(uint8_t novoModo) {
  if (modo == Controle::manual && novoModo == 1) SF_PID::Inicializar();
  modo = (novoModo == 1) ? Controle::automatico : Controle::manual;
}

void SF_PID::Inicializar() {
  somaSaida = *minhaSaida;
  ultimaEntrada = ProcessarEntrada(*minhaEntrada);
  somaSaida = constrain(somaSaida, saidaMin, saidaMax);
}

void SF_PID::DefinirDirecao(Acao acao_) { acao = acao_; }
void SF_PID::DefinirDirecao(uint8_t direcao) { acao = (Acao)direcao; }

void SF_PID::Reiniciar() {
  ultimoTempo = micros() - tempoAmostragemUs;
  ultimaEntrada = 0; somaSaida = 0; termoP = 0; termoI = 0; termoD = 0;
  primeiraLeitura = true;
}

void SF_PID::DefinirSomaSaida(float soma) { somaSaida = soma; }

float SF_PID::ObterKp() { return dispKp; }
float SF_PID::ObterKi() { return dispKi; }
float SF_PID::ObterKd() { return dispKd; }
float SF_PID::ObterTermoP() { return termoP; }
float SF_PID::ObterTermoI() { return termoI; }
float SF_PID::ObterTermoD() { return termoD; }
float SF_PID::ObterSomaSaida() { return somaSaida; }
uint8_t SF_PID::ObterModo() { return static_cast<uint8_t>(modo); }
uint8_t SF_PID::ObterDirecao() { return static_cast<uint8_t>(acao); }