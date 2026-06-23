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

/* Algoritmos de Filtragem e Condicionamento */
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
    alpha = alphaMin + (alphaMax - alphaMin) * proporcao;
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

void SF_PID::DefinirEntrada(Entrada tipoEntrada) { modoEntrada = tipoEntrada; }
void SF_PID::DefinirEntrada(uint8_t tipoEntrada) { modoEntrada = (Entrada)tipoEntrada; }
void SF_PID::DefinirCoeficientes(float a, float b, float c, float offset) { coefA = a; coefB = b; coefC = c; offsetSinal = offset; }
void SF_PID::DefinirFiltro(Filtro tipoFiltro) { modoFiltro = tipoFiltro; primeiraLeitura = true; }
void SF_PID::DefinirFiltro(uint8_t tipoFiltro) { modoFiltro = (Filtro)tipoFiltro; primeiraLeitura = true; }
void SF_PID::ConfigurarFiltroEMA(float minAlpha, float maxAlpha, float rangeVariacao) { alphaMin = minAlpha; alphaMax = maxAlpha; rangeVar = rangeVariacao; }
void SF_PID::ConfigurarFiltroKalman(float ruidoMedida, float ruidoProcesso) { rMedida = ruidoMedida; qProcesso = ruidoProcesso; }

/* Configuração da SINTONIA (AutoTune) */
void SF_PID::DefinirModoSintonia(Sintonia modo) { modoSintonia = modo; }
void SF_PID::DefinirModoSintonia(uint8_t modo) { modoSintonia = (Sintonia)modo; }

void SF_PID::LigarSintonia() {
  if (modoSintonia == Sintonia::desligado) return;
  sintoniaLigada = true;
  tuneMax = -9999.0f; tuneMin = 9999.0f;
  tuneCiclos = 0; tuneSomaPeriodo = 0;
  tuneTempoReferencia = millis();
  tuneUltimoCruzamento = millis();
  tunePassos = 1;
  
  if(modoSintonia == Sintonia::heuristica) heurEstado = 0; 
  if(modoSintonia == Sintonia::self || modoSintonia == Sintonia::zn_self || modoSintonia == Sintonia::tl_self) selfAtuacoes = 0;
}

void SF_PID::DesligarSintonia() { sintoniaLigada = false; }
bool SF_PID::SintoniaAtiva() { return sintoniaLigada; }

String SF_PID::ObterStatusSintonia() {
    if (!sintoniaLigada || modoSintonia == Sintonia::desligado) {
        return "Sintonia Inativa";
    }

    if (modoSintonia == Sintonia::zn || modoSintonia == Sintonia::tl || 
        modoSintonia == Sintonia::zn_self || modoSintonia == Sintonia::tl_self) {
        return "Rele: Buscando oscilacao sustentada (Ciclo " + String(tuneCiclos) + "/8)";
    }

    if (modoSintonia == Sintonia::heuristica) {
        if (heurEstado == 1 || heurEstado == 11 || heurEstado == 12) {
            return "Heuristica: Ajustando Kp (Calculo " + String(tunePassos) + ")";
        } else if (heurEstado == 2 || heurEstado == 21) {
            return "Heuristica: Ajustando Ki (Calculo " + String(tunePassos) + ")";
        } else if (heurEstado == 3) {
            return "Heuristica: Ajustando Kd (Calculo " + String(tunePassos) + ")";
        }
        return "Heuristica: Inicializando...";
    }

    if (modoSintonia == Sintonia::self) {
        return "Self: Avaliando onda (Ciclo " + String(tuneCiclos) + "/5) | Atuacoes: " + String(selfAtuacoes) + "/20";
    }

    return "Status Desconhecido";
}

/* =================================================================================
   O CÉREBRO: Calcular() com Condicionamento Desvinculado
================================================================================== */
bool SF_PID::Calcular() {
  if (modo == Controle::manual) return false;
  
  // 1. CONDICIONAMENTO CONTÍNUO
  float entrada = ProcessarEntrada(*minhaEntrada);
  *minhaEntrada = entrada; 

  // ROTA A: SINTONIA RELÉ (Bang-Bang Contínuo)
  bool isRelay = (modoSintonia == Sintonia::zn || modoSintonia == Sintonia::tl || 
                  modoSintonia == Sintonia::zn_self || modoSintonia == Sintonia::tl_self);
                  
  if (sintoniaLigada && isRelay) {
      ExecutarRelay(entrada);
      return true; 
  }

  uint32_t agora = micros();
  uint32_t variacaoTempo = (agora - ultimoTempo);
  
  // 2. MATEMÁTICA PID CLÁSSICA
  if (variacaoTempo >= tempoAmostragemUs) {
    float dEntrada = entrada - ultimaEntrada;
    erro = *meuSetpoint - entrada;

    if (acao == Acao::reverso) {
        dEntrada = -dEntrada;
        erro = -erro;
    }

    // O Coração do PID Clássico (PonE - Proporcional no Erro)
    termoP = kp * erro;          // Proporcional no Erro (Empurra pro Alvo)
    termoI = ki * erro;          // Integral (Mata o erro de regime)
    termoD = -kd * dEntrada;     // Derivativo na Medição (Freia a velocidade, não chuta no SP)

    // Acumulador Integral com Anti-Windup
    somaSaida += termoI;
    somaSaida = constrain(somaSaida, saidaMin, saidaMax);

    // Soma Final
    *minhaSaida = constrain(termoP + somaSaida + termoD, saidaMin, saidaMax);

    ultimoErro = erro; ultimaEntrada = entrada; ultimoTempo = agora;

    // ROTA C: SUPERVISORES
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
    // 1. Ignora os picos nos dois primeiros ciclos para limpar o susto inicial e a resposta pendular
    if (tuneCiclos >= 2) {
        if (entradaLida > tuneMax) tuneMax = entradaLida;
        if (entradaLida < tuneMin) tuneMin = entradaLida;
    }

    bool cruzouCima = (entradaLida > *meuSetpoint + 0.1f);
    bool cruzouBaixo = (entradaLida < *meuSetpoint - 0.1f);

    if (cruzouBaixo && !estadoRele) {
        estadoRele = true;
        *minhaSaida = saidaMax;
        
        uint32_t agoraMil = millis();

        // Só acumula o tempo de período a partir do ciclo 2
        if (tuneCiclos >= 2) {
            tuneSomaPeriodo += (agoraMil - tuneUltimoCruzamento);
        }

        tuneUltimoCruzamento = agoraMil;
        tuneCiclos++;

        // A MÁGICA: No exato momento em que encerra o ciclo 2, zeramos o lixo térmico.
        if (tuneCiclos == 2) {
            tuneMax = -9999.0f;
            tuneMin = 9999.0f;
            tuneSomaPeriodo = 0; 
        }
    } 
    else if (cruzouCima && estadoRele) {
        estadoRele = false;
        *minhaSaida = saidaMin;
    }

    // 8 ciclos estabilizados + 2 descartados = 10 ciclos totais exigidos
    if (tuneCiclos >= 10) {
        float Tu = (tuneSomaPeriodo / 8.0f) / 1000.0f; // Média limpa em segundos
        float amplitude = (tuneMax - tuneMin) / 2.0f;
        float Ku = (4.0f * (saidaMax - saidaMin)) / (3.14159f * amplitude);

        float nKp = 0, nKi = 0, nKd = 0;

        if (modoSintonia == Sintonia::zn || modoSintonia == Sintonia::zn_self) {
            nKp = 0.6f * Ku; 
            nKi = (2.0f * nKp) / Tu; 
            nKd = (nKp * Tu) / 8.0f;
        } else {
            nKp = 0.31f * Ku; 
            nKi = nKp / (2.2f * Tu); 
            nKd = (nKp * Tu) / 6.3f;
        }
        
        // ATENUAÇÃO DERIVATIVA PARA SISTEMAS TÉRMICOS DE ALTA INÉRCIA:
        // Evita que ondas longas (Tu gigante) gerem um Kd astronômico que satura o PWM.
        nKd = nKd * 0.02f; // Reduz o impacto derivativo mantendo apenas a força de micro-amortecimento

        DefinirAjustes(nKp, nKi, nKd);

        if (modoSintonia == Sintonia::zn_self || modoSintonia == Sintonia::tl_self) {
            modoSintonia = Sintonia::self; LigarSintonia(); 
        } else {
            DesligarSintonia(); 
        }
    }
}

void SF_PID::ExecutarHeuristica(float entradaLida) {
    uint32_t agoraMil = millis();

    // =========================================================
    // Estado 0: Inicialização
    // =========================================================
    if (heurEstado == 0) {
        DefinirAjustes(100.0f, 0.0f, 0.0f); // Kp inicial de 100
        passoKp = 20.0f; passoKi = 0.0025f; passoKd = 0.001f;
        heurEstado = 1; 
        tunePassos = 1;
        tuneTempoReferencia = agoraMil;
        return;
    }

    uint32_t tempoNaJanela = agoraMil - tuneTempoReferencia;

    // =========================================================
    // ESTÁGIO 1: AJUSTE DO Kp (Janelas de 15 min = 900.000 ms)
    // =========================================================
    if (heurEstado == 1 || heurEstado == 11 || heurEstado == 12) {
        
        // FASE CEGA: 10 minutos (600.000 ms). Ignora a rampa térmica.
        if (tempoNaJanela < 600000) {
            tuneMax = entradaLida;
            tuneMin = entradaLida;
        } 
        // FASE DE OBSERVAÇÃO: 5 minutos finais (300.000 ms). Captura a oscilação real.
        else if (tempoNaJanela < 900000) {
            if (entradaLida > tuneMax) tuneMax = entradaLida;
            if (entradaLida < tuneMin) tuneMin = entradaLida;
        } 
        // FIM DA JANELA: Executa a Lógica de Tomada de Decisão
        else {
            float variacaoAtual = tuneMax - tuneMin;

            if (heurEstado == 1) { // Primeira Janela (Decide Direção)
                tuneMaxAntigo = tuneMax;
                tuneMinAntigo = tuneMin;
                if (variacaoAtual > 0.5f) {
                    float novoKp = dispKp - passoKp;
                    DefinirAjustes(novoKp < 1.0f ? 1.0f : novoKp, 0, 0); 
                    heurEstado = 12; // Modo de Descida
                } else {
                    DefinirAjustes(dispKp + passoKp, 0, 0); 
                    heurEstado = 11; // Modo de Subida
                }
                tunePassos++;
            } 
            else if (heurEstado == 11) { // Modo de Subida (Procura Instabilidade)
                float variacaoAntiga = tuneMaxAntigo - tuneMinAntigo;
                if (variacaoAtual > variacaoAntiga && variacaoAtual > 0.5f) { 
                    DefinirAjustes(dispKp - passoKp, 0, 0); 
                    passoKp /= 4.0f; 
                    if (passoKp < 2.0f) { 
                        heurEstado = 2; // Kp perfeito
                        tunePassos = 1; // Zera para fase Ki
                    } else {
                        DefinirAjustes(dispKp + passoKp, 0, 0); 
                        tunePassos++;
                    }
                } else {
                    DefinirAjustes(dispKp + passoKp, 0, 0);
                    tunePassos++;
                }
                tuneMaxAntigo = tuneMax; tuneMinAntigo = tuneMin;
            }
            else if (heurEstado == 12) { // Modo de Descida (Procura Estabilidade)
                if (variacaoAtual <= 0.5f) {
                    passoKp /= 4.0f; 
                    if (passoKp < 2.0f) {
                        heurEstado = 2; // Kp perfeito
                        tunePassos = 1; // Zera para fase Ki
                    } else {
                        DefinirAjustes(dispKp + passoKp, 0, 0);
                        heurEstado = 11; // Inverte para subir
                        tunePassos++;
                    }
                } else {
                    float novoKp = dispKp - passoKp;
                    DefinirAjustes(novoKp < 1.0f ? 1.0f : novoKp, 0, 0);
                    tunePassos++;
                }
                tuneMaxAntigo = tuneMax; tuneMinAntigo = tuneMin;
            }

            tuneTempoReferencia = agoraMil; // Zera cronômetro para próxima janela
        }
        return;
    }

    // =========================================================
    // ESTÁGIO 2: AJUSTE DO Ki (Janelas de 30 min = 1.800.000 ms)
    // =========================================================
    if (heurEstado == 2 || heurEstado == 21) {
        
        // FASE CEGA: 20 minutos (1.200.000 ms)
        if (tempoNaJanela < 1200000) {
            tuneMax = entradaLida;
            tuneMin = entradaLida;
        } 
        // FASE DE OBSERVAÇÃO: 10 minutos finais (600.000 ms)
        else if (tempoNaJanela < 1800000) {
            if (entradaLida > tuneMax) tuneMax = entradaLida;
            if (entradaLida < tuneMin) tuneMin = entradaLida;
        } 
        // FIM DA JANELA: Lógica de Decisão do Ki
        else {
            float erroAtual = abs(*meuSetpoint - entradaLida);
            float variacaoAtual = tuneMax - tuneMin;

            if (heurEstado == 2) {
                tuneMaxAntigo = tuneMax;
                tuneMinAntigo = tuneMin;
                if (erroAtual <= 0.2f) {
                    heurEstado = 3; 
                    tunePassos = 1; // Zera para fase Kd
                } else {
                    DefinirAjustes(dispKp, dispKi + passoKi, 0); 
                    heurEstado = 21; 
                    tunePassos++;
                }
            } 
            else if (heurEstado == 21) {
                float variacaoAntiga = tuneMaxAntigo - tuneMinAntigo;
                if (erroAtual <= 0.2f) {
                    heurEstado = 3; 
                    tunePassos = 1; // Zera para fase Kd
                } else if (variacaoAtual > variacaoAntiga && variacaoAtual > 0.5f) {
                    DefinirAjustes(dispKp, dispKi - passoKi, 0);
                    passoKi /= 2.0f; 
                    tunePassos++;
                } else {
                    DefinirAjustes(dispKp, dispKi + passoKi, 0);
                    tunePassos++;
                }
                tuneMaxAntigo = tuneMax; tuneMinAntigo = tuneMin;
            }
            tuneTempoReferencia = agoraMil;
        }
        return;
    }

    // =========================================================
    // ESTÁGIO 3: AJUSTE DO Kd (Janelas de 10 min = 600.000 ms)
    // =========================================================
    if (heurEstado == 3) {
        // Para o Kd, precisamos capturar picos o tempo TODO (sem fase cega)
        if (entradaLida > tuneMax) tuneMax = entradaLida;
        
        if (tempoNaJanela >= 600000) {
            float overshootAtual = tuneMax - *meuSetpoint;
            
            if (overshootAtual > 0.5f) { 
                DefinirAjustes(dispKp, dispKi, dispKd + passoKd);
                tuneTempoReferencia = agoraMil;
                tunePassos++;
            } else {
                DesligarSintonia(); 
            }
            tuneMax = -9999.0f; // Reinicia apenas o Max
        }
        return;
    }
}

void SF_PID::ExecutarSelf(float entradaLida) {
    if (entradaLida > tuneMax) tuneMax = entradaLida;
    if (entradaLida < tuneMin) tuneMin = entradaLida;

    if (entradaLida > *meuSetpoint && !cruzouSP) { cruzouSP = true; tuneCiclos++; }
    if (entradaLida < *meuSetpoint && cruzouSP)  { cruzouSP = false; }

    if (tuneCiclos >= 5) {
        float variacao = tuneMax - tuneMin;
        float erroAtual = abs(*meuSetpoint - entradaLida);
        bool atuou = false;

        if (variacao > 1.0f) { 
            DefinirAjustes(dispKp * 0.90f, dispKi, dispKd * 1.10f); atuou = true;
        } else if (erroAtual > 1.0f) { 
            DefinirAjustes(dispKp * 1.15f, dispKi, dispKd); atuou = true;
        } else if (erroAtual > 0.2f && erroAtual <= 1.0f) { 
            DefinirAjustes(dispKp, dispKi * 1.08f, dispKd); atuou = true;
        }

        tuneMax = -9999; tuneMin = 9999; tuneCiclos = 0;
        
        if (atuou) {
            selfAtuacoes++;
            if (selfAtuacoes >= 20) DesligarSintonia(); 
        } else {
            DesligarSintonia(); 
        }
    }
}

/* =================================================================================
   Funções Padrão de Configuração
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
float SF_PID::ObterAlpha() { return alpha; }  
uint8_t SF_PID::ObterModo() { return static_cast<uint8_t>(modo); }
uint8_t SF_PID::ObterDirecao() { return static_cast<uint8_t>(acao); }