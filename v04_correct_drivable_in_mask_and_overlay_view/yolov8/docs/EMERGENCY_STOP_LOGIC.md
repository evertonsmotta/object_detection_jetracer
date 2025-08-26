# 🚨 Lógica de Emergency Stop - Sistema MPC JetRacer

## 📋 Visão Geral

O sistema de Emergency Stop foi redesenhado para ser **mais preciso e seguro**, ativando apenas em duas situações específicas que representam risco real de colisão ou falha de navegação.

## 🎯 **Condições de Ativação**

### **1. Detecção de Obstáculos Próximos nas Lanes**
- **Quando**: Na área crítica entre as lanes utilizadas para cálculo-previsão do centro da pista, há algo que **NÃO** seja a classe "drivable"
- **Área Crítica**: Do meio da imagem (30cm do carro) até a base da imagem, entre as faixas da pista
- **Critérios**:
  - Objeto detectado com confiança > 70%
  - Classe diferente de "drivable"
  - Posicionado na área crítica (entre as faixas, 30cm até base)
  - Ocupa área significativa (>5% da imagem)

### **2. Falha na Detecção de Lanes**
- **Quando**: O sistema não consegue detectar lanes para estimar o centro da pista
- **Critérios**:
  - Nenhuma lane detectada (0 lanes)
  - Confiança média das detecções < 60%
  - **Nota**: Com 1 lane é possível estimar o centro da pista

## 🔧 **Implementação Técnica**

### **Função Principal de Verificação**
```cpp
bool AutonomousController::shouldActivateEmergencyStop(
    const std::vector<Detection>& detections,
    const std::vector<cv::Mat>& masks,
    const std::unordered_map<int, std::string>& labels_map) {
    
    // 1. Verificar obstáculos nas lanes
    if (hasObstacleInLanes(detections, masks, labels_map)) {
        return true;
    }
    
    // 2. Verificar falha na detecção de lanes
    if (hasLaneDetectionFailure(detections, labels_map)) {
        return true;
    }
    
    return false;
}
```

### **Verificação de Obstáculos na Área Crítica**
```cpp
bool AutonomousController::hasObstacleInLanes(...) {
    for (const auto& det : detections) {
        std::string label = labels_map.at(det.label_id);
        
        // Só considerar obstáculos (não-drivable) com alta confiança
        if (label != "drivable" && det.confidence > 0.7) {
            
            // Calcular área do obstáculo
            double obstacle_area = det.bbox[2] * det.bbox[3]; // width * height
            double obstacle_percentage = (obstacle_area / total_image_area) * 100.0;
            
            // Verificar se está na área crítica (entre as faixas, 30cm até base)
            bool is_in_critical_area = isObstacleInCenterArea(det.bbox);
            
            // ATIVAR EMERGENCY STOP se:
            // 1. Obstáculo com alta confiança (>70%)
            // 2. Ocupa área significativa (>5% da imagem)
            // 3. Está na área crítica (entre as faixas, do meio até a base)
            if (obstacle_percentage > 5.0 && is_in_critical_area) {
                return true;
            }
        }
    }
    return false;
}
```

### **Verificação de Falha de Lanes**
```cpp
bool AutonomousController::hasLaneDetectionFailure(...) {
    int lane_count = 0;
    
    for (const auto& det : detections) {
        std::string label = labels_map.at(det.label_id);
        if (label == "lane") {
            lane_count++;
        }
    }
    
    // Falha se menos de 2 lanes detectadas
    if (lane_count < 2) {
        return true;
    }
    
    // Verificar confiança média
    double avg_confidence = calculateAverageConfidence(detections);
    if (avg_confidence < 0.6) {
        return true;
    }
    
    return false;
}
```

## 🚗 **Integração com o Sistema Principal**

### **Verificação Antes do Controle PID**
```cpp
// Em yolov8_seg.cpp - antes do processamento PID
// ====== VERIFICAÇÃO DE SEGURANÇA ANTES DO CONTROLE ======
bool emergency_stop_required = false;

// 1. Verificar obstáculos nas lanes
for (const auto& det : res) {
    std::string label = labels_map[det.class_id];
    if (label != "drivable" && det.confidence > 0.7) {
        emergency_stop_required = true;
        break;
    }
}

// 2. Verificar falha na detecção de lanes
if (!emergency_stop_required) {
    int lane_count = countLanes(res, labels_map);
    if (lane_count < 2) {
        emergency_stop_required = true;
    }
}

// ATIVAR EMERGENCY STOP SE NECESSÁRIO
if (emergency_stop_required) {
    activateEmergencyStop();
    continue; // Pular para o próximo frame
}
```

### **Verificação no AutonomousController**
```cpp
// Em autonomous_controller.cpp - antes de processar waypoints
void AutonomousController::applySteeringAssist(...) {
    // Verificação de segurança antes de processar
    if (shouldActivateEmergencyStop(detections, masks, labels_map)) {
        std::cout << "[MPC] 🚨 ATIVANDO EMERGENCY STOP por condições de segurança!" << std::endl;
        emergencyStop();
        return;
    }
    
    // Continuar com processamento normal...
}
```

## 📊 **Classes de Detecção e Seu Papel**

### **Classe "drivable" (Área Segura)**
- **Função**: Área onde o veículo pode navegar com segurança
- **Uso**: Base para cálculo de trajetória
- **Não ativa**: Emergency Stop

### **Classe "lane" (Faixa de Rodagem)**
- **Função**: Linhas de referência para navegação
- **Requisito**: Mínimo de 1 lane para estimar o centro da pista
- **Falha**: Ativa Emergency Stop apenas se 0 lanes detectadas

### **Outras Classes (Obstáculos)**
- **stop sign**: Sinal de parada
- **speed 50/80**: Limites de velocidade
- **passadeira**: Área de pedestres
- **jetracer**: Outro veículo
- **gate**: Portão/barreira
- **Função**: Todas ativam Emergency Stop se detectadas com alta confiança

## ⚡ **Sequência de Ativação**

### **1. Detecção do Problema**
```
Frame de vídeo → YOLOv8-Seg → Análise de classes → Verificação de segurança
```

### **2. Avaliação de Risco**
```
Obstáculo detectado OU Falha de lanes → shouldActivateEmergencyStop() → true
```

### **3. Ativação do Emergency Stop**
```
emergencyStop() → Parar veículo → Desabilitar controle → Aguardar reset
```

### **4. Recuperação**
```
Usuário pressiona L1 → reset() → Reabilitar controle → Continuar operação
```

## 🎮 **Controles do Usuário**

### **Botão L2 (Emergency Stop Manual)**
- **Função**: Parada de emergência manual
- **Uso**: Em situações de pânico ou controle perdido
- **Sempre ativo**: Independente da lógica automática

### **Botão L1 (Reset)**
- **Função**: Resetar sistema após Emergency Stop
- **Uso**: Após resolver situação de risco
- **Requer**: Verificação manual de segurança

## 📈 **Métricas de Segurança**

### **Logs de Sistema**
```bash
[SAFETY] 🚨 Emergency Stop: Obstáculo detectado nas lanes: stop sign
[SAFETY] 🚨 Emergency Stop: Falha na detecção de lanes - apenas 1 lane(s) detectada(s)
[MPC] 🚨 ATIVANDO EMERGENCY STOP por condições de segurança!
[SAFETY] Sistema parado. Pressione L1 para reset ou L2 para emergência.
```

### **Monitoramento em Tempo Real**
- **Contador de Emergency Stops** ativados
- **Tipo de violação** (obstáculo vs. falha de lanes)
- **Confiança das detecções** que causaram parada
- **Tempo de recuperação** após reset

## 🎨 **Visualização da Área Crítica**

### **Funcionalidades de Visualização**
- **Área Crítica Dinâmica**: Forma trapezoidal baseada nas lanes detectadas (mais larga na base, mais estreita no topo)
- **Objetos Drivable**: Bounding boxes verdes com máscaras sobrepostas
- **Obstáculos Críticos**: Bounding boxes vermelhos (Emergency Stop)
- **Obstáculos na Área**: Bounding boxes laranjas (na área crítica)
- **Obstáculos Distantes**: Bounding boxes cinzas (ignorados)
- **Máscaras das Lanes**: Sobrepostas com transparência para visualização
- **Contorno das Lanes**: Linhas coloridas mostrando a forma real das faixas

### **Configuração de Visualização**
```cpp
// Em yolov8_seg.cpp
#define ENABLE_CRITICAL_AREA_VISUALIZATION 1  // Habilitar visualização
#define ENABLE_DISPLAY 1                      // Display em tempo real

// Função de visualização dinâmica
cv::Mat critical_area_visualization = autonomous_controller->visualizeCriticalArea(
    img, res, masks, labels_map);

// Salvar visualização
cv::imwrite("critical_area_visualization.jpg", critical_area_visualization);

// Mostrar em tempo real
cv::imshow("AREA CRITICA DINAMICA - Emergency Stop", critical_area_visualization);
```

### **Algoritmo de Área Crítica Dinâmica**
```cpp
// 1. Detectar lanes e extrair contornos
// 2. Separar lanes esquerda e direita por posição X
// 3. Amostrar pontos em alturas específicas (160px a 320px)
// 4. Construir contorno trapezoidal conectando os pontos
// 5. Aplicar transparência verde na área crítica
// 6. Desenhar bordas e labels informativos
```

### **Legenda das Cores**
- 🟢 **VERDE**: Área crítica dinâmica (trapezoidal) + objetos drivable
- 🔴 **VERMELHO**: Obstáculo crítico (Emergency Stop)
- 🟠 **LARANJA**: Obstáculo na área crítica
- ⚫ **CINZA**: Obstáculo distante (ignorado)
- 🔵 **AZUL**: Contornos das lanes detectadas
- 🟡 **AMARELO**: Pontos de amostragem da área crítica

## 🔍 **Exemplos de Cenários**
```
Situação: Sinal de parada detectado na área crítica (entre as lanes, 30cm até base)
Detecção: stop sign com confiança 0.85, área >5%, posição crítica
Ação: Emergency Stop ativado
Log: "[SAFETY] 🚨 Emergency Stop: Obstáculo detectado nas lanes: stop sign"
```

### **Cenário 1b: Obstáculo Distante (Não Crítico)**
```
Situação: Sinal de parada detectado acima do meio da imagem
Detecção: stop sign com confiança 0.85, mas acima de 30cm do carro
Ação: Sistema continua operando (obstáculo muito distante)
Log: "[SAFETY] ✅ Obstáculo distante ignorado - fora da área crítica"
```

### **Cenário 2: Falha de Detecção**
```
Situação: Nenhuma lane detectada
Detecção: lane_count = 0
Ação: Emergency Stop ativado
Log: "[SAFETY] 🚨 Emergency Stop: Falha na detecção de lanes - nenhuma lane detectada!"
```

### **Cenário 2b: Operação com 1 Lane**
```
Situação: Apenas 1 lane detectada
Detecção: lane_count = 1
Ação: Sistema continua operando (é possível estimar o centro)
Log: "[SAFETY] ✅ Lanes detectadas: 1 | É possível estimar o centro da pista"
```

### **Cenário 3: Operação Normal**
```
Situação: 2+ lanes detectadas, apenas área drivable
Detecção: lane_count = 3, drivable areas
Ação: Sistema continua operando normalmente
Log: Nenhum Emergency Stop ativado
```

## ⚠️ **Configurações e Thresholds**

### **Thresholds Configuráveis**
```cpp
// Em include/mpc_integration_config.hpp
#define EMERGENCY_STOP_DISTANCE 1.0        // Distância para parada (metros)
#define MIN_LANES_REQUIRED 1               // Mínimo de lanes para operação (atualizado)
#define MIN_DETECTION_CONFIDENCE 0.6       // Confiança mínima para detecções
#define OBSTACLE_CONFIDENCE_THRESHOLD 0.7  // Confiança para considerar obstáculo
#define OBSTACLE_AREA_THRESHOLD 5.0        // Área mínima do obstáculo (% da imagem)
#define CRITICAL_AREA_X_MARGIN 0.25        // Margem da área crítica X (25% das bordas)
#define CRITICAL_AREA_Y_START 0.5          // Início da área crítica Y (meio da imagem - 30cm)
#define CRITICAL_AREA_Y_END 1.0            // Fim da área crítica Y (base da imagem)

// Configurações de Visualização
#define ENABLE_CRITICAL_AREA_VISUALIZATION 1  // Habilitar visualização da área crítica
#define ENABLE_DISPLAY 1                      // Habilitar display em tempo real
```

### **Ajustes de Performance**
- **Confiança muito alta**: Pode causar falsos positivos
- **Confiança muito baixa**: Pode ignorar obstáculos reais
- **Número de lanes**: Mínimo 1 para estimar o centro da pista
- **Área crítica**: Do meio (30cm) até a base da imagem para foco em obstáculos próximos

## 🚀 **Benefícios da Nova Lógica**

### **1. Precisão**
- ✅ Emergency Stop apenas quando realmente necessário
- ✅ Evita paradas desnecessárias por falsos positivos
- ✅ Foco nas duas situações de risco real

### **2. Segurança**
- ✅ Detecção proativa de obstáculos
- ✅ Prevenção de falhas de navegação
- ✅ Resposta imediata a situações críticas

### **3. Usabilidade**
- ✅ Sistema mais confiável para o usuário
- ✅ Menos interrupções desnecessárias
- ✅ Recuperação rápida após situações de risco

### **4. Visualização e Debug**
- ✅ **Visualização em tempo real** da área crítica dinâmica
- ✅ **Forma trapezoidal realista** baseada nas lanes detectadas
- ✅ **Máscaras das lanes** sobrepostas na imagem
- ✅ **Bounding boxes coloridos** para cada tipo de objeto
- ✅ **Legenda clara** para interpretação visual
- ✅ **Salvamento automático** das visualizações

### **5. Área Crítica Dinâmica**
- ✅ **Forma realista** que acompanha as curvas das lanes
- ✅ **Mais larga na base** (onde o carro está mais próximo)
- ✅ **Mais estreita no topo** (onde o carro está mais distante)
- ✅ **Adaptação automática** às diferentes configurações de pista
- ✅ **Fallback inteligente** para situações sem detecção de lanes

---

**Lógica de Emergency Stop - Versão MPC 1.0**

*Implementação: Agosto 2024*
*Status: ✅ Implementado e testado*
*Segurança: 🚨 Máxima prioridade*
