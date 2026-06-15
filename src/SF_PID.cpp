/**********************************************************************************
  Biblioteca SF_PID para Arduino/ESP32 - Version 1.0.0
  por saulfernandes https://github.com/saulfernandes/SF_PID
  Baseada na biblioteca QuickPID. Licenciada sob a Licença MIT
 **********************************************************************************/

#include "Arduino.h"
#include "SF_PID.h"

SF_PID::SF_PID() {}

/* Construtores ********************************************************************
 **********************************************************************************/
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

  ultimoTempo = micros() - tempoAmostragemUs;
}

SF_PID::SF_PID(float* Entrada, float* Saida, float* Setpoint)
  : SF_PID::SF_PID(Entrada, Saida, Setpoint,
                   dispKp = 0,
                   dispKi = 0,
                   dispKd = 0,
                   acao = Acao::direto) {
}

/* Calcular() **********************************************************************
 **********************************************************************************/
bool SF_PID::Calcular() {
  if (modo == Controle::manual) return false;
  
  uint32_t agora = micros();
  uint32_t variacaoTempo = (agora - ultimoTempo);
  
  if (variacaoTempo >= tempoAmostragemUs) {

    float entrada = *minhaEntrada;
    float dEntrada = entrada - ultimaEntrada;
    if (acao == Acao::reverso) dEntrada = -dEntrada;

    erro = *meuSetpoint - entrada;
    if (acao == Acao::reverso) erro = -erro;
    float dErro = erro - ultimoErro;

    // Logica Fixa: Termo Proporcional em PV (Leitura)
    float termoPm = kp * dEntrada;
    termoP = -termoPm; 
    
    // Termo Integral basico
    termoI = ki * erro;
    
    // Logica Fixa: Termo Derivativo em PV (Leitura)
    termoD = -kd * dEntrada; 

    // Logica Fixa: Anti-windup Condicional
    bool aw = false;
    float saidaTermoI = (-termoPm) + ki * (termoI + erro);
    
    if (saidaTermoI > saidaMax && dErro > 0) aw = true;
    else if (saidaTermoI < saidaMin && dErro < 0) aw = true;
    
    if (aw && ki > 0) termoI = constrain(saidaTermoI, -saidaMax, saidaMax);

    // Calculo e limitacao da Saida Final
    somaSaida += termoI;                                                 
    somaSaida = constrain(somaSaida - termoPm, saidaMin, saidaMax); 
    *minhaSaida = constrain(somaSaida + termoD, saidaMin, saidaMax); 

    ultimoErro = erro;
    ultimaEntrada = entrada;
    ultimoTempo = agora;
    return true;
  }
  return false;
}

/* DefinirAjustes(...)**********************************************************
******************************************************************************/
void SF_PID::DefinirAjustes(float Kp, float Ki, float Kd) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;
  if (Ki == 0) somaSaida = 0;
  
  dispKp = Kp; dispKi = Ki; dispKd = Kd;
  
  float tempoAmostragemSeg = (float)tempoAmostragemUs / 1000000;
  kp = Kp;
  ki = Ki * tempoAmostragemSeg;
  kd = Kd / tempoAmostragemSeg;
}

/* DefinirTempoAmostragemUs(.)**************************************************
******************************************************************************/
void SF_PID::DefinirTempoAmostragemUs(uint32_t NovoTempoAmostragemUs) {
  if (NovoTempoAmostragemUs > 0) {
    float razao = (float)NovoTempoAmostragemUs / (float)tempoAmostragemUs;
    ki *= razao;
    kd /= razao;
    tempoAmostragemUs = NovoTempoAmostragemUs;
  }
}

/* DefinirLimitesSaida(..)******************************************************
******************************************************************************/
void SF_PID::DefinirLimitesSaida(float Min, float Max) {
  if (Min >= Max) return;
  saidaMin = Min;
  saidaMax = Max;

  if (modo != Controle::manual) {
    *minhaSaida = constrain(*minhaSaida, saidaMin, saidaMax);
    somaSaida = constrain(somaSaida, saidaMin, saidaMax);
  }
}

/* DefinirModo(.)***************************************************************
******************************************************************************/
void SF_PID::DefinirModo(Controle novoModo) {
  if (modo == Controle::manual && novoModo == Controle::automatico) {
    SF_PID::Inicializar();
  }
  modo = novoModo;
}

void SF_PID::DefinirModo(uint8_t novoModo) {
  if (modo == Controle::manual && novoModo == 1) {
    SF_PID::Inicializar();
  }
  modo = (novoModo == 1) ? Controle::automatico : Controle::manual;
}

/* Inicializar()****************************************************************
******************************************************************************/
void SF_PID::Inicializar() {
  somaSaida = *minhaSaida;
  ultimaEntrada = *minhaEntrada;
  somaSaida = constrain(somaSaida, saidaMin, saidaMax);
}

/* DefinirDirecao(.)************************************************************
******************************************************************************/
void SF_PID::DefinirDirecao(Acao acao_) {
  acao = acao_;
}
void SF_PID::DefinirDirecao(uint8_t direcao) {
  acao = (Acao)direcao;
}

/* Reiniciar()******************************************************************
******************************************************************************/
void SF_PID::Reiniciar() {
  ultimoTempo = micros() - tempoAmostragemUs;
  ultimaEntrada = 0;
  somaSaida = 0;
  termoP = 0;
  termoI = 0;
  termoD = 0;
}

/* DefinirSomaSaida()***********************************************************
******************************************************************************/
void SF_PID::DefinirSomaSaida(float soma) {
  somaSaida = soma;
}

/* Funcoes de Status (Query)****************************************************
******************************************************************************/
float SF_PID::ObterKp() { return dispKp; }
float SF_PID::ObterKi() { return dispKi; }
float SF_PID::ObterKd() { return dispKd; }
float SF_PID::ObterTermoP() { return termoP; }
float SF_PID::ObterTermoI() { return termoI; }
float SF_PID::ObterTermoD() { return termoD; }
float SF_PID::ObterSomaSaida() { return somaSaida; }
uint8_t SF_PID::ObterModo() { return static_cast<uint8_t>(modo); }
uint8_t SF_PID::ObterDirecao() { return static_cast<uint8_t>(acao); }