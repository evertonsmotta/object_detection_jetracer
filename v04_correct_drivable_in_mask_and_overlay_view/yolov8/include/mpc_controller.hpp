#ifndef MPC_CONTROLLER_HPP
#define MPC_CONTROLLER_HPP

#include "vehicle_model.hpp"
#include <vector>
#include <memory>

namespace mpc {

struct MPCParams {
    int prediction_horizon;     // Horizonte de predição
    int control_horizon;        // Horizonte de controle
    double dt;                  // Intervalo de tempo (segundos)

    // Pesos para a função de custo
    double weight_steering;     // Peso para mudanças de direção
    double weight_speed;        // Peso para mudanças de velocidade
    double weight_cross_track;  // Peso para erro lateral
    double weight_heading;      // Peso para erro de direção
    double weight_velocity;     // Peso para erro de velocidade

    MPCParams() :
        prediction_horizon(10),
        control_horizon(5),
        dt(0.1),
        weight_steering(1.0),      // Reduzido drasticamente de 2.0 para 1.0 - Mínimo peso para estabilidade
        weight_speed(1.0),         // Reduzido de 1.5 para 1.0 - Mínimo peso para estabilidade de velocidade
        weight_cross_track(20.0),  // Aumentado drasticamente de 12.0 para 20.0 - Máximo peso para correção lateral
        weight_heading(15.0),      // Aumentado drasticamente de 6.0 para 15.0 - Máximo peso para correção de direção
        weight_velocity(2.0) {}
};

struct ControlOutput {
    double steering;            // Ângulo de direção (GRAUS em vez de radianos)
    double speed;               // Velocidade (m/s)
    double cross_track_error;   // Erro lateral atual
    double heading_error;       // Erro de direção atual (GRAUS em vez de radianos)

    ControlOutput() : steering(0), speed(0), cross_track_error(0), heading_error(0) {}
};

class MPCController {
public:
    MPCController(const MPCParams& params = MPCParams());

    // Resolve o problema de otimização MPC
    ControlOutput solve(const VehicleState& current_state,
                       const std::vector<cv::Point2f>& waypoints);

    // Atualiza os parâmetros do controlador
    void updateParams(const MPCParams& params);

    // Obtém os parâmetros atuais
    MPCParams getParams() const;

    // Reseta o controlador
    void reset();

    // Habilita/desabilita o controle
    void setEnabled(bool enabled);
    bool isEnabled() const;

private:
    MPCParams params_;
    std::unique_ptr<VehicleModel> vehicle_model_;
    bool enabled_;

    // Função de custo para otimização
    double calculateCost(const std::vector<VehicleState>& predicted_states,
                        const std::vector<double>& steering_sequence,
                        const std::vector<double>& speed_sequence,
                        const std::vector<cv::Point2f>& waypoints);

    // Otimização simples usando gradiente descendente
    std::pair<std::vector<double>, std::vector<double>> optimizeControl(
        const VehicleState& current_state,
        const std::vector<cv::Point2f>& waypoints);

    // Calcula gradientes para otimização
    void calculateGradients(const std::vector<VehicleState>& predicted_states,
                           const std::vector<double>& steering_sequence,
                           const std::vector<double>& speed_sequence,
                           const std::vector<cv::Point2f>& waypoints,
                           std::vector<double>& steering_gradients,
                           std::vector<double>& speed_gradients);

    // Aplica restrições aos comandos de controle
    void applyConstraints(std::vector<double>& steering_sequence,
                         std::vector<double>& speed_sequence);
};

} // namespace mpc

#endif // MPC_CONTROLLER_HPP
