#ifndef PERFORMANCE_CONFIG_HPP
#define PERFORMANCE_CONFIG_HPP

// ====== CONFIGURAÇÃO DE PERFORMANCE PARA YOLOv8-Seg ======
// Ajuste estas flags para otimizar o FPS conforme necessário

namespace PerformanceConfig {

    // ====== FLAGS DE OTIMIZAÇÃO VISUAL ======
    constexpr bool ENABLE_VISUAL_OVERLAY = true;      // Desabilitar para melhorar FPS
    constexpr bool ENABLE_LANE_LINES = true;          // Desabilitar para melhorar FPS
    constexpr bool ENABLE_UDP_STREAMING = true;       // Desabilitar para melhorar FPS
    constexpr bool ENABLE_CONSOLE_LOG = true;         // Desabilitar para melhorar FPS
    constexpr bool ENABLE_FPS_DISPLAY = true;         // Sempre manter para monitoramento

    // ====== CONFIGURAÇÕES DE PROCESSAMENTO ======
    constexpr int FPS_UPDATE_INTERVAL = 30;           // Atualizar FPS a cada N frames
    constexpr int CONSOLE_LOG_INTERVAL = 30;          // Log no console a cada N frames
    constexpr int MAX_MASKS_PREALLOC = 10;            // Máximo de máscaras para pré-alocação

    // ====== MODOS DE PERFORMANCE PRÉ-DEFINIDOS ======

    // Modo BALANCEADO (padrão)
    struct BalancedMode {
        static constexpr bool VISUAL_OVERLAY = true;
        static constexpr bool LANE_LINES = true;
        static constexpr bool UDP_STREAMING = true;
        static constexpr bool CONSOLE_LOG = true;
        static constexpr bool FPS_DISPLAY = true;
    };

    // Modo ALTO FPS (desabilita funcionalidades visuais)
    struct HighFPSMode {
        static constexpr bool VISUAL_OVERLAY = false;
        static constexpr bool LANE_LINES = false;
        static constexpr bool UDP_STREAMING = false;
        static constexpr bool CONSOLE_LOG = false;
        static constexpr bool FPS_DISPLAY = true;
    };

    // Modo VISUAL COMPLETO (todas as funcionalidades)
    struct FullVisualMode {
        static constexpr bool VISUAL_OVERLAY = true;
        static constexpr bool LANE_LINES = true;
        static constexpr bool UDP_STREAMING = true;
        static constexpr bool CONSOLE_LOG = true;
        static constexpr bool FPS_DISPLAY = true;
    };

    // ====== SELEÇÃO DO MODO ATUAL ======
    // Descomente apenas UMA das linhas abaixo para selecionar o modo:

    using CurrentMode = BalancedMode;        // Modo balanceado (padrão)
    // using CurrentMode = HighFPSMode;     // Modo alto FPS (recomendado para testes)
    // using CurrentMode = FullVisualMode;  // Modo visual completo

    // ====== CONFIGURAÇÕES AUTOMÁTICAS ======
    constexpr bool ENABLE_VISUAL_OVERLAY_FINAL = CurrentMode::VISUAL_OVERLAY;
    constexpr bool ENABLE_LANE_LINES_FINAL = CurrentMode::LANE_LINES;
    constexpr bool ENABLE_UDP_STREAMING_FINAL = CurrentMode::UDP_STREAMING;
    constexpr bool ENABLE_CONSOLE_LOG_FINAL = CurrentMode::CONSOLE_LOG;
    constexpr bool ENABLE_FPS_DISPLAY_FINAL = CurrentMode::FPS_DISPLAY;

} // namespace PerformanceConfig

#endif // PERFORMANCE_CONFIG_HPP
