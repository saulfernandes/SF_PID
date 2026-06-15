#pragma once
#include <stdint.h>
#ifndef SF_PID_h
#define SF_PID_h

class SF_PID {

  public:

    enum class Controle : uint8_t {manual, automatico};      // modo de controle principal
    enum class Acao : uint8_t {direto, reverso};             // acao do controlador

    // Construtor padrao
    SF_PID();

    // Construtor completo. Vincula o PID a Entrada, Saida, Setpoint, parametros iniciais e acao.
    SF_PID(float *Entrada, float *Saida, float *Setpoint, float Kp, float Ki, float Kd, Acao acao);

    // Construtor simplificado que usa acao direta como padrao.
    SF_PID(float *Entrada, float *Saida, float *Setpoint);

    // Define o modo do PID: manual (0), automatico (1).
    void DefinirModo(Controle modo);
    void DefinirModo(uint8_t modo);

    // Executa o calculo do PID. Deve ser chamado toda vez que o loop() rodar.
    bool Calcular();

    // Define e limita a saida para um intervalo especifico (0-255 por padrao).
    void DefinirLimitesSaida(float Min, float Max);

    // Permite mudar os ajustes durante a execucao (Controle Adaptativo).
    void DefinirAjustes(float Kp, float Ki, float Kd);

    // Define a direcao do controlador (Direto ou Reverso).
    void DefinirDirecao(Acao acao);
    void DefinirDirecao(uint8_t direcao);

    // Define o tempo de amostragem em microssegundos. Padrao eh 100000 us (0.1s).
    void DefinirTempoAmostragemUs(uint32_t NovoTempoAmostragemUs);

    // Define o valor de soma da saida (integral).
    void DefinirSomaSaida(float soma);

    void Inicializar();       // Garante transferencia suave do manual para automatico
    void Reiniciar();         // Limpa os termos P, I, D e a soma da saida

    // Funcoes de Consulta (Query) 
    float ObterKp();            
    float ObterKi();            
    float ObterKd();            
    float ObterTermoP();        
    float ObterTermoI();        
    float ObterTermoD();        
    float ObterSomaSaida();     
    uint8_t ObterModo();        
    uint8_t ObterDirecao();     

  private:

    float dispKp = 0;   
    float dispKi = 0;
    float dispKd = 0;
    float termoP;
    float termoI;
    float termoD;

    float kp;           
    float ki;           
    float kd;           

    float *minhaEntrada;     
    float *minhaSaida;    
    float *meuSetpoint;  
    
    float somaSaida;

    Controle modo = Controle::automatico;
    Acao acao = Acao::direto;

    uint32_t tempoAmostragemUs, ultimoTempo;
    float saidaMin, saidaMax, erro, ultimoErro, ultimaEntrada;

};
#endif