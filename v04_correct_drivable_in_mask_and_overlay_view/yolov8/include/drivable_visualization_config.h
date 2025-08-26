#ifndef DRIVABLE_VISUALIZATION_CONFIG_H
#define DRIVABLE_VISUALIZATION_CONFIG_H

// ====== CONFIGURAÇÕES DE VISUALIZAÇÃO DAS ÁREAS DRIVABLE ======

// Habilitar/desabilitar visualização das áreas drivable
#define ENABLE_DRIVABLE_VISUALIZATION true

// Habilitar/desabilitar destaque das áreas drivable entre lanes
#define ENABLE_DRIVABLE_HIGHLIGHT true

// ====== MODO DE EXIBIÇÃO DAS ÁREAS DRIVABLE ======
// DRIVABLE_DISPLAY_MODE:
// 0 = Não exibir áreas drivable
// 1 = Exibir apenas áreas drivable entre as lanes do ROI (entre base e topo)
// 2 = Exibir todas as áreas drivable detectadas
#define DRIVABLE_DISPLAY_MODE 1

// ====== CONFIGURAÇÕES DO ROI ======
// Habilitar/desabilitar exibição de bboxs
#define SHOW_BBOX false

// Configurações de área do ROI
#define ROI_BASE_Y_RATIO 0.9        // Posição Y da base do ROI (80% da altura da imagem)
#define ROI_TOP_Y_RATIO 0.2         // Posição Y do topo do ROI (30% da altura da imagem)
#define ROI_LANE_TOLERANCE 0.15     // Tolerância para considerar lanes dentro do ROI (15% da largura)

// Cores para diferentes tipos de áreas drivable
#define DRIVABLE_BETWEEN_LANES_COLOR cv::Scalar(0, 255, 255)    // Ciano brilhante
#define DRIVABLE_OTHER_AREAS_COLOR cv::Scalar(0, 128, 0)        // Verde escuro
#define DRIVABLE_ROI_COLOR cv::Scalar(255, 255, 0)              // Amarelo para áreas no ROI
#define DRIVABLE_BORDER_COLOR cv::Scalar(0, 255, 255)           // Ciano para bordas

// Configurações de transparência
#define DRIVABLE_OPACITY 0.6              // 60% de opacidade para áreas entre lanes
#define DRIVABLE_OTHER_OPACITY 0.3        // 30% de opacidade para outras áreas
#define DRIVABLE_ROI_OPACITY 0.7          // 70% de opacidade para áreas no ROI

// Configurações de área crítica
#define CENTER_TOLERANCE_RATIO 0.3        // 30% da largura da imagem para área central
#define MIN_INTERSECTION_AREA 100         // Área mínima de interseção para destacar

// Configurações de texto
#define SHOW_DRIVABLE_LABELS false         // Mostrar labels "DRIVABLE" nas áreas
#define SHOW_HIGHLIGHT_LABELS false        // Mostrar labels "DRIVABLE ENTRE LANES"
#define SHOW_ROI_LABELS true              // Mostrar labels "DRIVABLE NO ROI"

// Configurações de performance
#define DRIVABLE_UPDATE_INTERVAL 1        // Atualizar visualização a cada N frames




#endif // DRIVABLE_VISUALIZATION_CONFIG_H
