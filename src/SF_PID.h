#pragma once
#include <stdint.h>
#ifndef SF_PID_h
#define SF_PID_h

class SF_PID {

  public:

    enum class Controle : uint8_t {manual, automatico};      
    enum class Acao : uint8_t {direto, reverso};             
    enum class Entrada : uint8_t {filtrada, temperatura, pura}; 
    enum class Filtro : uint8_t {nenhum, emaAdaptativo, mediana, kalman1D}; 
    
    // Novas Opcoes de Sintonia
    enum class Sintonia : uint8_t {desligado, zn, tl, heuristica, self, zn_self, tl_self};

    // Construtores
    SF_PID();
    SF_PID(float *Entrada, float *Saida, float *Setpoint, float Kp, float Ki, float Kd, Acao acao);
    SF_PID(float *Entrada, float *Saida, float *Setpoint);

    // Operacao Base
    void DefinirModo(Controle modo);
    void DefinirModo(uint8_t modo);
    bool Calcular();

    // Ajustes de Controle
    void DefinirLimitesSaida(float Min, float Max);
    void DefinirAjustes(float Kp, float Ki, float Kd);
    void DefinirDirecao(Acao acao);
    void DefinirDirecao(uint8_t direcao);
    void DefinirTempoAmostragemUs(uint32_t NovoTempoAmostragemUs);
    void DefinirSomaSaida(float soma);
    void Inicializar();       
    void Reiniciar();         

    // Condicionamento de Sinal e Filtros
    void DefinirEntrada(Entrada tipoEntrada);
    void DefinirEntrada(uint8_t tipoEntrada);
    void DefinirCoeficientes(float a, float b, float c, float offset);
    
    void DefinirFiltro(Filtro tipoFiltro);
    void DefinirFiltro(uint8_t tipoFiltro);
    void ConfigurarFiltroEMA(float minAlpha, float maxAlpha, float rangeVariacao);
    void ConfigurarFiltroKalman(float ruidoMedida, float ruidoProcesso);

    // ===============================================================
    // COMANDOS DO AUTOTUNE / SINTONIA
    // ===============================================================
    void DefinirModoSintonia(Sintonia modo);
    void DefinirModoSintonia(uint8_t modo);
    void LigarSintonia();
    void DesligarSintonia();
    bool SintoniaAtiva(); // Retorna true se estiver sintonizando, false se acabou

    // Funcoes de Consulta (Query) 
    float ObterKp();            
    float ObterKi();            
    float ObterKd();            
    float ObterTermoP();        
    float ObterTermoI();        
    float ObterTermoD();        
    float ObterSomaSaida();     
    float ObterAlpha(); 
    uint8_t ObterModo();        
    uint8_t ObterDirecao();     

  private:

    float ProcessarEntrada(float valorLido);
    float AplicarFiltro(float valor);
    
    // Motores de Sintonia
    void ExecutarRelay(float entradaLida);
    void ExecutarHeuristica(float entradaLida);
    void ExecutarSelf(float entradaLida);

    float dispKp = 0, dispKi = 0, dispKd = 0;
    float termoP, termoI, termoD;
    float kp, ki, kd;           

    float *minhaEntrada;     
    float *minhaSaida;    
    float *meuSetpoint;  
    
    float somaSaida;

    Controle modo = Controle::automatico;
    Acao acao = Acao::direto;
    Entrada modoEntrada = Entrada::filtrada;
    Filtro modoFiltro = Filtro::nenhum;
    Sintonia modoSintonia = Sintonia::desligado;
    
    bool sintoniaLigada = false;

    uint32_t tempoAmostragemUs, ultimoTempo;
    float saidaMin, saidaMax, erro, ultimoErro, ultimaEntrada;

    // Variaveis de Condicionamento
    bool primeiraLeitura = true;
    float coefA = -0.0000004f, coefB = 0.02745f, coefC = -16.547f, offsetSinal = 0.0f;
    
    // Variaveis dos Filtros
    float alphaMin = 0.05f, alphaMax = 0.8f, rangeVar = 20.0f;
    float ultimoValorEMA = 0.0f;
    float bufferMediana[15] = {0};
    uint8_t indiceMediana = 0;
    float estimativaKalman = 0.0f, erroKalman = 1.0f, rMedida = 0.1f, qProcesso = 0.01f; 

    // ===============================================================
    // Variaveis de Memoria do AutoTune
    // ===============================================================
    uint32_t tuneTempoReferencia = 0;
    uint32_t tuneUltimoCruzamento = 0;
    float tuneMax = -9999.0f, tuneMin = 9999.0f;
    uint8_t tuneCiclos = 0;
    float tuneSomaPeriodo = 0;
    bool estadoRele = false;

    // Memorias Heuristica (Manual)
    uint8_t heurEstado = 0;
    float passoKp = 20.0f, passoKi = 0.0025f, passoKd = 0.001f;

    // Memorias Self
    uint8_t selfAtuacoes = 0;
    bool cruzouSP = false;
};
#endif