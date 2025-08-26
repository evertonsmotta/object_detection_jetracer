#include "mpc_control_interface.hpp"
#include "autonomous_controller.hpp"
#include <iostream>
#include <memory>
#include <atomic>

// TEMPORARIAMENTE COMENTADO PARA RESOLVER ERRO DE COMPILAÇÃO
// As variáveis globais não estão definidas no projeto principal
/*
// Variáveis globais (definidas em yolov8_seg.cpp)
extern std::unique_ptr<mpc::AutonomousController> g_mpc_controller;
extern std::atomic<bool> g_mpc_enabled;
extern std::atomic<bool> g_emergency_stop;

// Função para alternar modo MPC via joystick
void toggleMPCViaJoystick() {
    if (!g_mpc_controller) {
        std::cout << "[MPC] Controlador não inicializado" << std::endl;
        return;
    }

    // Obtém modo atual
    auto current_mode = g_mpc_controller->getStatus().control_mode;

    // Cicla entre os 3 modos: MANUAL → STEERING_ASSIST → FULL_AUTONOMOUS → MANUAL
    mpc::ControlMode next_mode;
    switch (current_mode) {
        case mpc::ControlMode::MANUAL:
            next_mode = mpc::ControlMode::STEERING_ASSIST;
            g_mpc_enabled = true;
            std::cout << "[MPC] Alternando para modo ASSISTÊNCIA DE DIREÇÃO via joystick" << std::endl;
            break;

        case mpc::ControlMode::STEERING_ASSIST:
            next_mode = mpc::ControlMode::FULL_AUTONOMOUS;
            g_mpc_enabled = true;
            std::cout << "[MPC] Alternando para modo AUTÔNOMO COMPLETO via joystick" << std::endl;
            break;

        case mpc::ControlMode::FULL_AUTONOMOUS:
            next_mode = mpc::ControlMode::MANUAL;
            g_mpc_enabled = false;
            std::cout << "[MPC] Alternando para modo MANUAL via joystick" << std::endl;
            break;
    }

    // Aplica o novo modo
    g_mpc_controller->setControlMode(next_mode);
}

// Função para parada de emergência via joystick
void emergencyStopViaJoystick() {
    g_emergency_stop = true;
    if (g_mpc_controller) {
        g_mpc_controller->emergencyStop();
    }
    std::cout << "[MPC] Parada de emergência via joystick!" << std::endl;
}

// Função para reset via joystick
void resetViaJoystick() {
    g_emergency_stop = false;
    if (g_mpc_controller) {
        g_mpc_controller->reset();
    }
    std::cout << "[MPC] Reset via joystick" << std::endl;
}

// Funções para interface externa (C-style)
extern "C" {
    int isMPCEnabled() {
        return g_mpc_enabled ? 1 : 0;
    }

    int isEmergencyStopActive() {
        return g_emergency_stop ? 1 : 0;
    }
}
*/

// Implementações temporárias vazias para resolver erro de compilação
void toggleMPCViaJoystick() {
    std::cout << "[MPC] Função temporariamente desabilitada" << std::endl;
}

void emergencyStopViaJoystick() {
    std::cout << "[MPC] Função temporariamente desabilitada" << std::endl;
}

void resetViaJoystick() {
    std::cout << "[MPC] Função temporariamente desabilitada" << std::endl;
}

extern "C" {
    int isMPCEnabled() {
        return 0; // Sempre retorna desabilitado temporariamente
    }

    int isEmergencyStopActive() {
        return 0; // Sempre retorna não ativo temporariamente
    }
}
