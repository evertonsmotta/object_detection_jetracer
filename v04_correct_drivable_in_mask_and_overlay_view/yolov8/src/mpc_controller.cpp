#include "mpc_controller.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace mpc {

// Função auxiliar para converter radianos para graus
double radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

MPCController::MPCController(const MPCParams& params)
    : params_(params), enabled_(true) {
    vehicle_model_.reset(new VehicleModel());
}

ControlOutput MPCController::solve(const VehicleState& current_state,
                                 const std::vector<cv::Point2f>& waypoints) {
    if (!enabled_ || waypoints.empty()) {
        ControlOutput output;
        output.steering = 0.0;
        output.speed = 0.0;
        return output;
    }

    // Otimiza a sequência de controle
    auto control_sequences = optimizeControl(current_state, waypoints);
    auto& steering_sequence = control_sequences.first;
    auto& speed_sequence = control_sequences.second;

    // Retorna o primeiro comando de controle
    ControlOutput output;
    output.steering = steering_sequence[0];
    output.speed = speed_sequence[0];

    // Calcula erros para debug (já em graus)
    output.cross_track_error = vehicle_model_->calculateCrossTrackError(current_state, waypoints);
    output.heading_error = vehicle_model_->calculateDesiredHeading(current_state, waypoints) - current_state.psi;

    // Converte erro de direção para graus e normaliza
    output.heading_error = radiansToDegrees(output.heading_error);
    while (output.heading_error > 180) output.heading_error -= 360;
    while (output.heading_error < -180) output.heading_error += 360;

    return output;
}

void MPCController::updateParams(const MPCParams& params) {
    params_ = params;
}

MPCParams MPCController::getParams() const {
    return params_;
}

void MPCController::reset() {
    // Reset do controlador se necessário
}

void MPCController::setEnabled(bool enabled) {
    enabled_ = enabled;
}

bool MPCController::isEnabled() const {
    return enabled_;
}

double MPCController::calculateCost(const std::vector<VehicleState>& predicted_states,
                                   const std::vector<double>& steering_sequence,
                                   const std::vector<double>& speed_sequence,
                                   const std::vector<cv::Point2f>& waypoints) {
    double total_cost = 0.0;

    // Custo dos erros de estado
    for (const auto& state : predicted_states) {
        double cross_track_error = vehicle_model_->calculateCrossTrackError(state, waypoints);
        double heading_error = vehicle_model_->calculateDesiredHeading(state, waypoints) - state.psi;

        // Converte erro de direção para graus e normaliza
        heading_error = radiansToDegrees(heading_error);
        while (heading_error > 180) heading_error -= 360;
        while (heading_error < -180) heading_error += 360;

        total_cost += params_.weight_cross_track * cross_track_error * cross_track_error;
        total_cost += params_.weight_heading * heading_error * heading_error;
        total_cost += params_.weight_velocity * (state.v - 2.0) * (state.v - 2.0); // Velocidade desejada = 2 m/s
    }

    // Custo das mudanças de controle
    for (size_t i = 0; i < steering_sequence.size(); ++i) {
        if (i > 0) {
            double steering_change = steering_sequence[i] - steering_sequence[i-1];
            double speed_change = speed_sequence[i] - speed_sequence[i-1];

            total_cost += params_.weight_steering * steering_change * steering_change;
            total_cost += params_.weight_speed * speed_change * speed_change;
        }
    }

    return total_cost;
}

std::pair<std::vector<double>, std::vector<double>> MPCController::optimizeControl(
    const VehicleState& current_state,
    const std::vector<cv::Point2f>& waypoints) {

    // Inicializa sequências de controle
    std::vector<double> steering_sequence(params_.control_horizon, 0.0);
    std::vector<double> speed_sequence(params_.control_horizon, 2.0); // Velocidade desejada

    // Parâmetros de otimização para MÁXIMA RESPONSIVIDADE
    const int max_iterations = 10;        // Reduzido drasticamente de 20 para 10 - Mínimas iterações = máxima responsividade
    const double learning_rate = 0.05;    // Aumentado drasticamente de 0.02 para 0.05 - Passo muito maior = convergência ultra-rápida
    const double tolerance = 1e-3;        // Aumentado drasticamente de 1e-4 para 1e-3 - Tolerância muito maior = menos precisão mas mais responsivo

    double prev_cost = std::numeric_limits<double>::max();

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        // Prediz estados futuros
        auto predicted_states = vehicle_model_->predict(current_state, steering_sequence, speed_sequence, params_.dt);

        // Calcula custo atual
        double current_cost = calculateCost(predicted_states, steering_sequence, speed_sequence, waypoints);

        // Verifica convergência
        if (std::abs(current_cost - prev_cost) < tolerance) {
            break;
        }

        prev_cost = current_cost;

        // Calcula gradientes
        std::vector<double> steering_gradients(params_.control_horizon, 0.0);
        std::vector<double> speed_gradients(params_.control_horizon, 0.0);

        calculateGradients(predicted_states, steering_sequence, speed_sequence, waypoints,
                          steering_gradients, speed_gradients);

        // Atualiza sequências de controle usando gradiente descendente
        for (size_t i = 0; i < params_.control_horizon; ++i) {
            steering_sequence[i] -= learning_rate * steering_gradients[i];
            speed_sequence[i] -= learning_rate * speed_gradients[i];
        }

        // Aplica restrições
        applyConstraints(steering_sequence, speed_sequence);
    }

    return {steering_sequence, speed_sequence};
}

void MPCController::calculateGradients(const std::vector<VehicleState>& predicted_states,
                                      const std::vector<double>& steering_sequence,
                                      const std::vector<double>& speed_sequence,
                                      const std::vector<cv::Point2f>& waypoints,
                                      std::vector<double>& steering_gradients,
                                      std::vector<double>& speed_gradients) {

    const double epsilon = 1e-6;

    for (size_t i = 0; i < params_.control_horizon; ++i) {
        // Gradiente para direção
        std::vector<double> steering_plus = steering_sequence;
        std::vector<double> steering_minus = steering_sequence;

        steering_plus[i] += epsilon;
        steering_minus[i] -= epsilon;

        auto states_plus = vehicle_model_->predict(predicted_states[0], steering_plus, speed_sequence, params_.dt);
        auto states_minus = vehicle_model_->predict(predicted_states[0], steering_minus, speed_sequence, params_.dt);

        double cost_plus = calculateCost(states_plus, steering_plus, speed_sequence, waypoints);
        double cost_minus = calculateCost(states_minus, steering_minus, speed_sequence, waypoints);

        steering_gradients[i] = (cost_plus - cost_minus) / (2.0 * epsilon);

        // Gradiente para velocidade
        std::vector<double> speed_plus = speed_sequence;
        std::vector<double> speed_minus = speed_sequence;

        speed_plus[i] += epsilon;
        speed_minus[i] -= epsilon;

        auto states_speed_plus = vehicle_model_->predict(predicted_states[0], steering_sequence, speed_plus, params_.dt);
        auto states_speed_minus = vehicle_model_->predict(predicted_states[0], steering_sequence, speed_minus, params_.dt);

        double cost_speed_plus = calculateCost(states_speed_plus, steering_sequence, speed_plus, waypoints);
        double cost_speed_minus = calculateCost(states_speed_minus, steering_sequence, speed_minus, waypoints);

        speed_gradients[i] = (cost_speed_plus - cost_speed_minus) / (2.0 * epsilon);
    }
}

void MPCController::applyConstraints(std::vector<double>& steering_sequence,
                                    std::vector<double>& speed_sequence) {
    VehicleParams vehicle_params;

    for (size_t i = 0; i < steering_sequence.size(); ++i) {
                // Limita ângulo de direção
        steering_sequence[i] = std::max(-vehicle_params.max_steering,
                                       std::min(steering_sequence[i], vehicle_params.max_steering));

        // Limita velocidade
        speed_sequence[i] = std::max(vehicle_params.min_speed,
                                    std::min(speed_sequence[i], vehicle_params.max_speed));
    }
}

} // namespace mpc
