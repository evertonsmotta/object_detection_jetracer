# 🔄 **Fluxo do Projeto MPC - JetRacer Autonomous Control**

## 📋 **Visão Geral do Sistema**

Este documento apresenta o fluxo detalhado do sistema de controle autônomo MPC (Model Predictive Control) do JetRacer, desde a inicialização até a operação em tempo real.

---

## 🎯 **Flowchart Principal do Sistema**

```mermaid
flowchart TD
    A[🚀 INICIALIZAÇÃO DO SISTEMA] --> B[📷 CAPTURA DE IMAGEM]
    B --> C[🤖 YOLOv8-Seg INFERENCE]
    C --> D[🔍 DETECÇÃO DE OBJETOS]
    D --> E[🚦 CLASSIFICAÇÃO E MÁSCARAS]
    E --> F{🚨 VERIFICAÇÃO DE SEGURANÇA}
    
    F -->|EMERGENCY STOP| G[🛑 PARADA DE EMERGÊNCIA]
    F -->|SEGURO| H[🎯 EXTRAÇÃO DE WAYPOINTS]
    
    G --> I[⏸️ AGUARDAR RESET]
    I --> J[🔄 RESET MANUAL]
    J --> B
    
    H --> K[🧮 CÁLCULO MPC]
    K --> L[📐 PLANEJAMENTO DE TRAJETÓRIA]
    L --> M[🎮 APLICAÇÃO DE COMANDOS]
    M --> N[🚗 MOVIMENTO DO JETRACER]
    N --> O{🔄 LOOP CONTINUO?}
    
    O -->|SIM| B
    O -->|NÃO| P[🛑 FINALIZAR SISTEMA]
```

---

## 🔍 **Flowchart Detalhado da Verificação de Segurança**

```mermaid
flowchart TD
    A[🚨 VERIFICAÇÃO DE SEGURANÇA] --> B{🔍 DETECÇÃO DE OBSTÁCULOS}
    A --> C{🚧 FALHA NA DETECÇÃO DE LANES}
    
    B --> D[📏 CALCULAR ÁREA DO OBSTÁCULO]
    D --> E[📍 VERIFICAR POSIÇÃO NA ÁREA CRÍTICA]
    E --> F{🎯 OBSTÁCULO CRÍTICO?}
    
    F -->|SIM| G[🚨 ATIVAR EMERGENCY STOP]
    F -->|NÃO| H[✅ CONTINUAR OPERAÇÃO]
    
    C --> I[🔢 CONTAR LANES DETECTADAS]
    I --> J{🚧 LANES INSUFICIENTES?}
    
    J -->|SIM| K[🚨 ATIVAR EMERGENCY STOP]
    J -->|NÃO| L[✅ CONTINUAR OPERAÇÃO]
    
    G --> M[🛑 PARAR TODOS OS SISTEMAS]
    K --> M
    M --> N[⏸️ AGUARDAR RESET MANUAL]
    
    H --> O[🎯 PRÓXIMO PASSO: MPC]
    L --> O
```

---

## 🎨 **Flowchart da Visualização da Área Crítica**

```mermaid
flowchart TD
    A[🎨 VISUALIZAÇÃO DA ÁREA CRÍTICA] --> B[📷 CLONAR FRAME ORIGINAL]
    B --> C[🔍 DETECTAR LANES NAS IMAGENS]
    C --> D[📐 EXTRAIR CONTORNOS DAS MÁSCARAS]
    D --> E[🚦 SEPARAR LANE ESQUERDA E DIREITA]
    
    E --> F{✅ LANES DETECTADAS?}
    F -->|SIM| G[📏 AMOSTRAR PONTOS EM ALTURAS ESPECÍFICAS]
    F -->|NÃO| H[📦 USAR ÁREA CRÍTICA PADRÃO]
    
    G --> I[🔗 CONSTRUIR CONTORNO TRAPEZOIDAL]
    I --> J[🎨 APLICAR TRANSPARÊNCIA VERDE]
    J --> K[📝 ADICIONAR LABELS E LEGENDA]
    
    H --> L[📐 DESENHAR RETÂNGULO PADRÃO]
    L --> K
    
    K --> M[💾 SALVAR VISUALIZAÇÃO]
    M --> N[🖥️ MOSTRAR EM TEMPO REAL]
```

---

## 🧮 **Flowchart do Sistema MPC**

```mermaid
flowchart TD
    A[🧮 SISTEMA MPC] --> B[📊 ESTADO ATUAL DO VEÍCULO]
    B --> C[🎯 WAYPOINTS EXTRAÍDOS]
    C --> D[📐 MODELO DINÂMICO DO VEÍCULO]
    
    D --> E[🎯 FUNÇÃO OBJETIVO]
    E --> F[📏 RESTRIÇÕES DE SEGURANÇA]
    F --> G[⚡ OTIMIZAÇÃO EM TEMPO REAL]
    
    G --> H{✅ SOLUÇÃO ENCONTRADA?}
    H -->|SIM| I[🎮 COMANDOS DE CONTROLE]
    H -->|NÃO| J[🔄 AJUSTAR PARÂMETROS]
    
    I --> K[🚗 APLICAR COMANDOS AO JETRACER]
    K --> L[📊 ATUALIZAR ESTADO]
    L --> M[🔄 PRÓXIMO CICLO MPC]
    
    J --> G
    M --> B
```

---

## 🎮 **Flowchart do Controle do JetRacer**

```mermaid
flowchart TD
    A[🎮 CONTROLE DO JETRACER] --> B{🔘 MODO DE OPERAÇÃO}
    
    B -->|MANUAL| C[🎮 CONTROLE TOTAL VIA JOYSTICK]
    B -->|ASSISTÊNCIA| D[🤖 VELOCIDADE MANUAL + DIREÇÃO AUTOMÁTICA]
    B -->|AUTÔNOMO| E[🤖 CONTROLE TOTAL AUTOMÁTICO]
    B -->|EMERGENCY| F[🚨 PARADA COMPLETA]
    
    C --> G[📊 LEITURA DO JOYSTICK]
    G --> H[🚗 APLICAÇÃO DIRETA AOS MOTORES]
    
    D --> I[📊 LEITURA DE VELOCIDADE DO JOYSTICK]
    I --> J[🧮 CÁLCULO MPC PARA DIREÇÃO]
    J --> K[🚗 APLICAÇÃO DE COMANDOS]
    
    E --> L[🧮 CÁLCULO MPC COMPLETO]
    L --> K
    
    F --> M[🛑 PARAR MOTORES E SERVO]
    M --> N[⏸️ AGUARDAR RESET]
    
    H --> O[🔄 PRÓXIMO CICLO]
    K --> O
    N --> P[🔄 VOLTAR AO LOOP PRINCIPAL]
```

---

## 🔄 **Flowchart do Loop Principal**

```mermaid
flowchart TD
    A[🔄 LOOP PRINCIPAL] --> B[📷 CAPTURAR FRAME]
    B --> C[🤖 PROCESSAR YOLOv8-Seg]
    C --> D[🔍 APLICAR NMS]
    D --> E[🚨 VERIFICAÇÃO DE SEGURANÇA]
    
    E --> F{🚨 EMERGENCY STOP?}
    F -->|SIM| G[🛑 PARAR SISTEMA]
    F -->|NÃO| H[🎯 PROCESSAR MPC]
    
    G --> I[⏸️ AGUARDAR RESET]
    I --> J[🔄 RESET MANUAL]
    J --> A
    
    H --> K[🎨 VISUALIZAR ÁREA CRÍTICA]
    K --> L[🧮 CALCULAR COMANDOS MPC]
    L --> M[🎮 APLICAR CONTROLES]
    M --> N[📊 LOGS E MONITORAMENTO]
    N --> O{🔄 CONTINUAR?}
    
    O -->|SIM| A
    O -->|NÃO| P[🛑 FINALIZAR]
```

---

## 📊 **Flowchart de Monitoramento e Logs**

```mermaid
flowchart TD
    A[📊 MONITORAMENTO E LOGS] --> B[📝 LOGS DE SEGURANÇA]
    A --> C[📈 MÉTRICAS DE PERFORMANCE]
    A --> D[🎨 VISUALIZAÇÕES SALVAS]
    
    B --> E[🚨 LOGS DE EMERGENCY STOP]
    B --> F[✅ LOGS DE OPERAÇÃO NORMAL]
    B --> G[⚠️ LOGS DE AVISOS]
    
    C --> H[⏱️ TEMPO DE PROCESSAMENTO]
    C --> I[🎯 PRECISÃO DAS DETECÇÕES]
    C --> J[🚗 ESTADO DO JETRACER]
    
    D --> K[📸 IMAGENS SALVAS]
    D --> L[🎬 VÍDEOS DE DEBUG]
    
    E --> M[💾 ARQUIVO DE LOG]
    F --> M
    G --> M
    H --> N[📊 DASHBOARD DE PERFORMANCE]
    I --> N
    J --> N
    K --> O[📁 PASTA DE VISUALIZAÇÕES]
    L --> O
```

---

## 🔧 **Flowchart de Configuração e Inicialização**

```mermaid
flowchart TD
    A[🔧 INICIALIZAÇÃO] --> B[📁 CARREGAR CONFIGURAÇÕES]
    B --> C[🤖 INICIALIZAR YOLOv8-Seg]
    C --> D[🚗 INICIALIZAR JETRACER]
    D --> E[🎮 INICIALIZAR JOYSTICK]
    
    E --> F{✅ TODOS OS SISTEMAS OK?}
    F -->|SIM| G[🚀 SISTEMA PRONTO]
    F -->|NÃO| H[❌ DIAGNÓSTICO DE ERRO]
    
    G --> I[🔄 ENTRAR NO LOOP PRINCIPAL]
    
    H --> J[📊 VERIFICAR CONEXÕES]
    J --> K{🔧 PROBLEMA RESOLVIDO?}
    K -->|SIM| A
    K -->|NÃO| L[🛑 FINALIZAR COM ERRO]
    
    C --> M[📊 CARREGAR MODELO TENSORRT]
    M --> N{✅ MODELO CARREGADO?}
    N -->|SIM| D
    N -->|NÃO| O[❌ ERRO NO MODELO]
    
    D --> P[🔌 VERIFICAR CONEXÃO I2C]
    P --> Q{✅ HARDWARE RESPONDE?}
    Q -->|SIM| E
    Q -->|NÃO| R[❌ ERRO NO HARDWARE]
```

---

## 📋 **Resumo dos Fluxos Principais**

### **1. Fluxo de Segurança (Prioridade Máxima)**
- **Verificação contínua** de obstáculos e falhas de detecção
- **Emergency Stop imediato** em situações críticas
- **Reset manual** para retomar operação

### **2. Fluxo de Controle MPC**
- **Extração de waypoints** das lanes detectadas
- **Otimização em tempo real** para comandos de controle
- **Aplicação suave** aos motores e servo

### **3. Fluxo de Visualização**
- **Área crítica dinâmica** baseada nas lanes reais
- **Forma trapezoidal** que acompanha as curvas
- **Debug visual** em tempo real

### **4. Fluxo de Monitoramento**
- **Logs detalhados** de todas as operações
- **Métricas de performance** em tempo real
- **Salvamento automático** de visualizações

---

## 🎯 **Pontos de Controle Críticos**

### **🚨 Emergency Stop**
- **Obstáculos críticos** na área de navegação
- **Falha na detecção** de lanes (0 lanes detectadas)
- **Problemas de hardware** ou comunicação

### **⚠️ Avisos do Sistema**
- **Baixa confiança** nas detecções
- **Poucas lanes** detectadas (1 lane)
- **Performance degradada** do modelo

### **✅ Operação Normal**
- **2+ lanes** detectadas com alta confiança
- **Área livre** de obstáculos críticos
- **Sistema MPC** funcionando adequadamente

---

## 🔄 **Ciclos de Operação**

### **Ciclo Principal: 30 FPS**
- **Captura de imagem**: 33ms
- **Inference YOLOv8**: 15ms
- **Processamento MPC**: 10ms
- **Aplicação de controles**: 2ms
- **Total**: ~60ms (16.7 FPS efetivo)

### **Ciclo de Segurança: 10 FPS**
- **Verificação de obstáculos**: 100ms
- **Verificação de lanes**: 50ms
- **Decisão de Emergency Stop**: 10ms
- **Total**: 160ms (6.25 FPS)

### **Ciclo de Visualização: 5 FPS**
- **Processamento de máscaras**: 200ms
- **Construção da área crítica**: 100ms
- **Renderização**: 100ms
- **Total**: 400ms (2.5 FPS)

---

**Documento criado**: Agosto 2024  
**Versão**: MPC 1.0  
**Status**: ✅ Implementado e documentado  
**Próxima atualização**: Após testes no Jetson Nano
