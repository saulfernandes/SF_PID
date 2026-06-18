# Biblioteca SF_PID

A **SF_PID** é uma biblioteca de controle PID (Proporcional, Integral, Derivativo) enxuta e robusta, otimizada para microcontroladores (ESP32, Arduino). Foi especificamente projetada para controle térmico de equipamentos industriais e laboratoriais.

Esta versão atua com configurações fixas de segurança:
* **Termos P e D na Leitura (PV):** Evita solavancos no atuador (*Kicks*) quando o Setpoint é alterado bruscamente.
* **Anti-Windup Condicional:** Impede que o erro integral se acumule infinitamente quando a máquina atinge o seu limite físico de potência.
* **Condicionamento de Sinal Integrado:** Suporte nativo a filtros digitais (EMA Adaptativo, Mediana, Kalman) e conversão polinomial para sensores industriais (PT100/TDR).

---

## Como Utilizar (Guia Passo a Passo)

### 1. Inicialização (Condicionamento de Sinal)
Configure a entrada, os filtros e a conversão do seu sensor diretamente no `setup()`:

```cpp
void setup() {
  // Define o tipo de entrada: 0 (filtrada), 1 (temperatura), 2 (bruta/ADC)
  ctrl_1.DefinirEntrada(SF_PID::Entrada::pura);
  
  // Define coeficientes para conversão polinomial (A*x² + B*x + C + offset)
  ctrl_1.DefinirCoeficientes(-0.0000004f, 0.02745f, -16.547f, offset);
  
  // Escolha o filtro ideal para seu sinal ruidoso
  ctrl_1.DefinirFiltro(SF_PID::Filtro::emaAdaptativo);
  ctrl_1.ConfigurarFiltroEMA(0.05f, 0.8f, 20.0f); // minAlpha, maxAlpha, RangeVariacao
  
  ctrl_1.DefinirModo(SF_PID::Controle::automatico);
}
```

### 2. Execução Industrial
No `loop()`, entregue o valor bruto do sensor e deixe que a biblioteca trate a limpeza e o controle:

```cpp
void loop() {
  Input = analogRead(TDR); // Lê o sinal sujo
  
  if (ctrl_1.Calcular()) { // A biblioteca limpa, converte e calcula o PID
    analogWrite(PINO_RESISTENCIA, Output); // Aplica saída limpa
  }
}
```

---

## Sintonia Automática (Autotune)

A biblioteca oferece motores de inteligência integrados para sintonizar os ganhos Kp, Ki e Kd automaticamente.

### Modos Disponíveis
* `SF_PID::Sintonia::zn` / `tl`: Método de Relé (Oscilação forçada).
* `SF_PID::Sintonia::heuristica`: Método heurístico manual automatizado.
* `SF_PID::Sintonia::self`: Supervisor adaptativo de fundo.

### Como usar o Autotune
1. Configure o modo: `ctrl_1.DefinirModoSintonia(SF_PID::Sintonia::self);`
2. Ative: `ctrl_1.LigarSintonia();`
3. Monitore o status na sua IHM usando: `ctrl_1.SintoniaAtiva();`
   * Quando o processo terminar, a biblioteca desligará a função automaticamente. Use isso para resetar o botão da sua IHM.

---

## Dicionário de Funções

### Configuração de Sinal
* `DefinirEntrada(tipo)`: Escolhe entre `filtrada`, `temperatura` ou `pura`.
* `DefinirCoeficientes(a, b, c, offset)`: Configura a curva de conversão do sensor.
* `DefinirFiltro(tipo)`: Escolhe entre `nenhum`, `emaAdaptativo`, `mediana` ou `kalman1D`.

### Controle PID
* `DefinirModo(modo)`: Alterna entre `manual` e `automatico`.
* `DefinirAjustes(Kp, Ki, Kd)`: Atualiza os ganhos em tempo real.
* `DefinirLimitesSaida(Min, Max)`: Proteção de hardware.
* `DefinirDirecao(acao)`: `direto` (aquecimento) ou `reverso` (refrigeração).

### Consultas (IHM/Serial)
* `ObterKp()`, `ObterKi()`, `ObterKd()`: Consulta os ganhos atuais.
* `ObterTermoP()`, `ObterTermoI()`, `ObterTermoD()`: Consulta o esforço matemático atual.