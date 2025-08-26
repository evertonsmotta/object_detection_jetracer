#pragma once

// ============================================================================
// CONFIGURAÇÃO DAS DIMENSÕES DO ROI (Region of Interest)
// ============================================================================
// Este arquivo controla as dimensões e posição da região de interesse
// onde o sistema detecta e analisa as faixas de trânsito

// ============================================================================
// POSIÇÃO VERTICAL DO ROI (ALTURA)
// ============================================================================
// As posições são definidas como porcentagens da altura total da imagem
// 0.0 = topo da imagem, 1.0 = base da imagem

// Posição de início do ROI (parte superior)
#define ROI_START_Y_RATIO         0.4     // 30% do topo da imagem
// Posição de fim do ROI (parte inferior)
#define ROI_END_Y_RATIO           0.7     // 80% do topo da imagem

// ============================================================================
// POSIÇÃO HORIZONTAL DO ROI (LARGURA)
// ============================================================================
// O ROI se estende por toda a largura da imagem por padrão
// Para limitar a largura, use as configurações abaixo

// Ativar limitação de largura do ROI
#define ENABLE_ROI_WIDTH_LIMIT    false   // true = limitar largura, false = largura total

// Limites horizontais (quando ENABLE_ROI_WIDTH_LIMIT = true)
#define ROI_START_X_RATIO         0.1     // 10% da esquerda da imagem
#define ROI_END_X_RATIO           0.9     // 90% da esquerda da imagem

// ============================================================================
// CONFIGURAÇÕES AVANÇADAS
// ============================================================================

// Altura mínima do ROI (em pixels)
#define ROI_MIN_HEIGHT_PIXELS     100     // Altura mínima em pixels

// Largura mínima do ROI (em pixels)
#define ROI_MIN_WIDTH_PIXELS      200     // Largura mínima em pixels

// ============================================================================
// CONFIGURAÇÕES DE TOLERÂNCIA
// ============================================================================
// Estas configurações controlam a tolerância para detecção de faixas laterais

// Tolerância para posição lateral das faixas (em porcentagem da largura da imagem)
#define LANE_CENTER_TOLERANCE_RATIO_1    0.3     // 30% da largura (filtro inicial)
#define LANE_CENTER_TOLERANCE_RATIO_2    0.4     // 40% da largura (filtro de penalização)

// ============================================================================
// CONFIGURAÇÕES DE CORES E TAMANHOS
// ============================================================================
// Cores e tamanhos para os elementos visuais do ROI

// Cor dos círculos no topo das lanes
#define LANE_TOP_POINT_COLOR     cv::Scalar(255, 0, 0)    // Vermelho

// Tamanho dos círculos no topo das lanes
#define LANE_TOP_POINT_RADIUS    6                           // Raio em pixels

// ============================================================================
// EXEMPLOS DE CONFIGURAÇÃO
// ============================================================================

// ROI Central (padrão atual)
// #define ROI_START_Y_RATIO         0.3     // 30% do topo
// #define ROI_END_Y_RATIO           0.8     // 80% do topo

// ROI Superior (mais próximo da câmera)
// #define ROI_START_Y_RATIO         0.1     // 10% do topo
// #define ROI_END_Y_RATIO           0.6     // 60% do topo

// ROI Inferior (mais distante da câmera)
// #define ROI_START_Y_RATIO         0.5     // 50% do topo
// #define ROI_END_Y_RATIO           0.9     // 90% do topo

// ROI Estreito (apenas centro da imagem)
// #define ENABLE_ROI_WIDTH_LIMIT    true
// #define ROI_START_X_RATIO         0.3     // 30% da esquerda
// #define ROI_END_X_RATIO           0.7     // 70% da esquerda

// ============================================================================
// NOTAS IMPORTANTES
// ============================================================================
// 1. ROI_START_Y_RATIO deve ser MENOR que ROI_END_Y_RATIO
// 2. Os valores devem estar entre 0.0 e 1.0
// 3. Para melhor detecção, mantenha o ROI entre 20% e 90% da altura
// 4. ROI muito pequeno pode reduzir a precisão da detecção
// 5. ROI muito grande pode incluir ruído e reduzir o FPS
// 6. Após alterar, recompile o projeto: make clean && make -j4
