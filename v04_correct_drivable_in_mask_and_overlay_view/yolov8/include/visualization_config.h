#pragma once

// ============================================================================
// CONFIGURAÇÕES DE VISUALIZAÇÃO - CONTROLE INDIVIDUAL DE ELEMENTOS
// ============================================================================
// Para ativar/desativar elementos visuais, altere as constantes abaixo:
// true = elemento VISÍVEL, false = elemento OCULTO

// ROI (Region of Interest)
#define SHOW_ROI_RECTANGLE        false    // Retângulo do ROI
#define SHOW_ROI_TEXT            false    // Texto "ROI", "ROI (Two Lanes)", etc.
#define SHOW_ROI_BASE_CENTER     false   // Círculo e texto "ROI Base Center" - DESATIVADO
#define SHOW_LANE_TOP_POINTS     false    // Círculos no topo de cada lane dentro do ROI (NOVO!)

// Centro da Pista
#define SHOW_CENTER_POINT        false    // Círculo verde do centro calculado
#define SHOW_CENTER_TEXT         false    // Texto "Center"
#define SHOW_CENTER_REFERENCE_LINE false  // Linha de referência vertical do centro

// Centro Estimado (caso de uma faixa)
#define SHOW_ESTIMATED_CENTER    false    // Círculo ciano do centro estimado
#define SHOW_ESTIMATED_TEXT      false    // Texto "Est. Center"
#define SHOW_ESTIMATED_REFERENCE_LINE false // Linha de referência vertical

// Conexões e Linhas
#define SHOW_CENTER_TO_LANES_CONNECTION false  // Linhas conectando centro às faixas
#define SHOW_LANE_BASE_POINTS    false    // Círculos vermelhos na base das faixas
#define SHOW_LANE_LABELS         false    // Textos "Left Lane Used", "Right Lane Used"

// Elementos de Debug
#define SHOW_DEBUG_INFO          false   // Informações de debug (coordenadas, medidas)
#define SHOW_CONFLICT_RESOLUTION false   // Informações de resolução de conflitos

// ============================================================================
// CONFIGURAÇÕES DE PERFORMANCE
// ============================================================================
// Estas configurações afetam o FPS - use com cuidado

// Otimizações de renderização
#define ENABLE_ANTIALIASING      true    // Suavização de linhas (cv::LINE_AA)
#define ENABLE_TEXT_RENDERING    true    // Renderização de texto
#define ENABLE_CIRCLE_RENDERING  true    // Renderização de círculos

// ============================================================================
// CONFIGURAÇÕES DE CORES
// ============================================================================
// Cores personalizáveis para diferentes elementos

// ROI Colors
#define ROI_RECTANGLE_COLOR      cv::Scalar(0, 0, 255)    // Azul
#define ROI_TEXT_COLOR           cv::Scalar(255, 0, 255)    // Magenta
#define ROI_BASE_CENTER_COLOR    cv::Scalar(255, 0, 255)    // Magenta

// Center Colors
#define CENTER_POINT_COLOR       cv::Scalar(0, 255, 0)      // Verde
#define CENTER_TEXT_COLOR        cv::Scalar(0, 255, 0)      // Verde
#define CENTER_REFERENCE_COLOR   cv::Scalar(0, 255, 0)      // Verde

// Estimated Center Colors
#define ESTIMATED_CENTER_COLOR   cv::Scalar(0, 255, 255)    // Ciano
#define ESTIMATED_TEXT_COLOR     cv::Scalar(0, 255, 255)    // Ciano
#define ESTIMATED_REFERENCE_COLOR cv::Scalar(0, 255, 255)   // Ciano

// Lane Colors
#define LANE_BASE_POINT_COLOR    cv::Scalar(0, 0, 255)      // Vermelho
#define LANE_LABEL_COLOR         cv::Scalar(0, 255, 255)    // Ciano

// Connection Lines
#define CONNECTION_LINE_COLOR    cv::Scalar(0, 255, 0)      // Verde
#define CONNECTION_LINE_THICKNESS 1                          // Espessura da linha

// ============================================================================
// CONFIGURAÇÕES DE TAMANHO
// ============================================================================
// Tamanhos personalizáveis para diferentes elementos

// Circle Sizes
#define ROI_BASE_CENTER_RADIUS      10
#define CENTER_POINT_RADIUS         8
#define ESTIMATED_CENTER_RADIUS     8
#define LANE_BASE_POINT_RADIUS     6

// Line Thickness
#define ROI_RECTANGLE_THICKNESS     2
#define CENTER_REFERENCE_THICKNESS  2
#define CONNECTION_LINE_THICKNESS   1

// Text Sizes
#define ROI_TEXT_SCALE              0.6
#define CENTER_TEXT_SCALE           0.6
#define ESTIMATED_TEXT_SCALE        0.5
#define LANE_LABEL_SCALE            0.6
