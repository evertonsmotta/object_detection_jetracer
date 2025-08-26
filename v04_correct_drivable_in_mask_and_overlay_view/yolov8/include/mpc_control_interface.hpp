#ifndef MPC_CONTROL_INTERFACE_HPP
#define MPC_CONTROL_INTERFACE_HPP

#ifdef __cplusplus
extern "C" {
#endif

// Funções para controle MPC via joystick ou interface externa
void toggleMPCViaJoystick();
void emergencyStopViaJoystick();
void resetViaJoystick();

// Funções para obter status do MPC
int isMPCEnabled();
int isEmergencyStopActive();

#ifdef __cplusplus
}
#endif

#endif // MPC_CONTROL_INTERFACE_HPP



