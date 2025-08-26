#ifndef MPC_INTEGRATION_CONFIG_HPP
#define MPC_INTEGRATION_CONFIG_HPP

// Configuração para integração MPC-JetRacer
#define MPC_CONTROL_AVAILABLE 1  // Habilita integração com sistema MPC

// Mapeamento de botões do joystick (baseado no jstest /dev/input/js0)
#define JOYSTICK_ASSISTANCE_MODE_BUTTON 7   // R1 - Modo assistência (velocidade manual + direção automática)
#define JOYSTICK_AUTONOMOUS_MODE_BUTTON 9   // R2 - Modo autônomo total
#define JOYSTICK_EMERGENCY_BUTTON 8         // L2 - Parada de emergência
#define JOYSTICK_RESET_BUTTON 6             // L1 - Reset do sistema

// Mapeamento de eixos do joystick (baseado no jstest /dev/input/js0)
#define JOYSTICK_ACCELERATOR_AXIS 1         // Eixo Y esquerdo (1) - Acelerador
#define JOYSTICK_STEERING_AXIS 2            // Eixo X direito (2) - Direção

// Configurações de controle
#define MAX_SPEED 100.0f                    // Velocidade máxima em %
#define MAX_ANGLE 140                       // Ângulo máximo de direção em graus (igual ao jetracer.hpp)
#define CONTROL_UPDATE_RATE 20              // Taxa de atualização do controle em Hz

#endif // MPC_INTEGRATION_CONFIG_HPP
