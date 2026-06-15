# Biblioteca SF_PID

A **SF_PID** é uma biblioteca de controle PID (Proporcional, Integral, Derivativo) enxuta e robusta, otimizada para microcontroladores (ESP32, Arduino). Foi especificamente projetada para controle térmico de equipamentos industriais e laboratoriais.

Esta versão atua com configurações fixas de segurança:
* **Termos P e D na Leitura (PV):** Evita solavancos no atuador (*Kicks*) quando o Setpoint é alterado bruscamente.
* **Anti-Windup Condicional:** Impede que o erro integral se acumule infinitamente quando a máquina atinge o seu limite físico de potência.

---

## Como Utilizar (Guia Passo a Passo)

Para utilizar a biblioteca no seu programa, siga estes 4 passos fundamentais:

### 1. Criar as Variáveis de Ligação
O PID precisa de três variáveis do tipo `float` para trabalhar: a leitura do sensor (Entrada), o esforço do atuador (Saída) e o alvo (Setpoint).

```cpp
float leitura_sensor = 25.0;
float sinal_atuador = 0.0;
float alvo_setpoint = 100.0;
```

### 2. Inicializar o Controlador (Objeto)
Fora de qualquer função (no escopo global), crie o controlador ligando-o às variáveis criadas no passo anterior. Utilize o "E comercial" (`&`) para enviar os endereços de memória.

```cpp
#include "SF_PID.h"

// Parâmetros de sintonia
float Kp = 2.5, Ki = 1.2, Kd = 0.5;

// Cria o PID para um Aquecedor (Ação Direta)
SF_PID meuPID(&leitura_sensor, &sinal_atuador, &alvo_setpoint, Kp, Ki, Kd, SF_PID::Acao::direto);
```

### 3. Configurar no setup()
Ao iniciar o microcontrolador, defina os limites físicos da sua máquina (por exemplo, um sinal PWM de 0 a 255) e ligue o PID (coloque-o em modo automático).

```cpp
void setup() {
  // A saída do PID nunca será menor que 0 nem maior que 255
  meuPID.DefinirLimitesSaida(0, 255);  
  
  // Liga o controle PID
  meuPID.DefinirModo(SF_PID::Controle::automatico);
}
```

### 4. Executar no loop()
O PID deve estar sempre avaliando o tempo. Leia o seu sensor físico, chame a função `Calcular()` e, se ela retornar verdadeiro, envie o sinal atualizado para a máquina.

```cpp
void loop() {
  // A. Atualiza a variável com a leitura física real
  leitura_sensor = analogRead(PINO_SENSOR); // Exemplo genérico  
  
  // B. Pede ao PID para fazer os cálculos matemáticos
  if (meuPID.Calcular() == true) {
    // C. O PID atualizou a variável 'sinal_atuador'. Envie-a para a máquina.
    analogWrite(PINO_AQUECEDOR, sinal_atuador);
  }
}
```

---

## Dicionário de Funções

Se precisar alterar o comportamento do PID durante a execução da máquina, utilize as funções abaixo:

### Configurações Básicas
* `DefinirModo(SF_PID::Controle::manual ou automatico)`: Liga ou desliga os cálculos do PID. A transição para automático é feita de forma suave. Aceita também `0` (manual) ou `1` (automático).
* `DefinirAjustes(Kp, Ki, Kd)`: Atualiza os ganhos do PID instantaneamente (útil para sistemas com Auto-Tune ou alteração por IHM).
* `DefinirLimitesSaida(Min, Max)`: Limita a saída matemática para proteger o seu equipamento físico.
* `DefinirDirecao(SF_PID::Acao::direto ou reverso)`: `direto` aumenta a saída quando a leitura cai (Aquecimento). `reverso` diminui a saída quando a leitura cai (Refrigeração).

### Controle de Tempo e Memória
* `DefinirTempoAmostragemUs(microssegundos)`: Altera a velocidade de cálculo do PID. O padrão é 100000 us (0,1 segundos).
* `Reiniciar()`: Apaga completamente a memória de erros do passado e os acumuladores da integral.
* `DefinirSomaSaida(valor)`: Injeta manualmente um valor no acumulador integral.

### Consultas (Para visualização em IHM ou Serial)
* `ObterKp()`, `ObterKi()`, `ObterKd()`: Retornam os ganhos atuais em formato legível.
* `ObterTermoP()`, `ObterTermoI()`, `ObterTermoD()`: Retornam o esforço que cada parcela matemática está fazendo neste exato momento.
* `ObterSomaSaida()`: Retorna o estado total do acumulador.