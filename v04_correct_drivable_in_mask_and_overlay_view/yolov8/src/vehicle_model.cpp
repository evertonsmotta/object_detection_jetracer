#include "vehicle_model.hpp"
#include <algorithm>
#include <iostream>

namespace mpc {

VehicleModel::VehicleModel(const VehicleParams& params) : params_(params) {}

VehicleState VehicleModel::update(const VehicleState& current_state,
                                 double steering_cmd,
                                 double speed_cmd,
                                 double dt) {
    // Limita os comandos de controle
    steering_cmd = std::max(-params_.max_steering, std::min(steering_cmd, params_.max_steering));
    speed_cmd = std::max(params_.min_speed, std::min(speed_cmd, params_.max_speed));

    return bicycleModel(current_state, steering_cmd, speed_cmd, dt);
}

std::vector<VehicleState> VehicleModel::predict(const VehicleState& current_state,
                                               const std::vector<double>& steering_sequence,
                                               const std::vector<double>& speed_sequence,
                                               double dt) {
    std::vector<VehicleState> predicted_states;
    VehicleState state = current_state;

    int horizon = std::min(steering_sequence.size(), speed_sequence.size());

    for (int i = 0; i < horizon; ++i) {
        state = bicycleModel(state, steering_sequence[i], speed_sequence[i], dt);
        predicted_states.push_back(state);
    }

    return predicted_states;
}

double VehicleModel::calculateCrossTrackError(const VehicleState& state,
                                             const std::vector<cv::Point2f>& waypoints) {
    if (waypoints.empty()) return 0.0;

    // Encontra o waypoint mais próximo
    double min_distance = std::numeric_limits<double>::max();
    cv::Point2f closest_waypoint;

    for (const auto& wp : waypoints) {
        double dx = wp.x - state.x;
        double dy = wp.y - state.y;
        double distance = std::sqrt(dx*dx + dy*dy);

        if (distance < min_distance) {
            min_distance = distance;
            closest_waypoint = wp;
        }
    }

    // Calcula o erro lateral (distância perpendicular à direção do veículo)
    double dx = closest_waypoint.x - state.x;
    double dy = closest_waypoint.y - state.y;

    // Projeta o vetor erro na direção perpendicular ao veículo
    double cross_track_error = -dx * std::sin(state.psi) + dy * std::cos(state.psi);

    return cross_track_error;
}

double VehicleModel::calculateDesiredHeading(const VehicleState& state,
                                            const std::vector<cv::Point2f>& waypoints) {
    if (waypoints.size() < 2) return state.psi;

    // Encontra o waypoint mais próximo
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        double dx = waypoints[i].x - state.x;
        double dy = waypoints[i].y - state.y;
        double distance = std::sqrt(dx*dx + dy*dy);

        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }

    // Calcula o ângulo para o próximo waypoint
    size_t next_idx = std::min(closest_idx + 1, waypoints.size() - 1);
    double dx = waypoints[next_idx].x - waypoints[closest_idx].x;
    double dy = waypoints[next_idx].y - waypoints[closest_idx].y;

    double desired_heading = std::atan2(dy, dx);

    // Normaliza o ângulo para [-π, π]
    while (desired_heading > M_PI) desired_heading -= 2 * M_PI;
    while (desired_heading < -M_PI) desired_heading += 2 * M_PI;

    return desired_heading;
}

cv::Point2f VehicleModel::pixelToMeters(const cv::Point2f& pixel, double scale_factor) {
    return cv::Point2f(pixel.x * scale_factor, pixel.y * scale_factor);
}

cv::Point2f VehicleModel::metersToPixel(const cv::Point2f& meters, double scale_factor) {
    return cv::Point2f(meters.x / scale_factor, meters.y / scale_factor);
}

VehicleState VehicleModel::bicycleModel(const VehicleState& state,
                                       double steering_cmd,
                                       double speed_cmd,
                                       double dt) {
    VehicleState new_state = state;

    // Modelo de bicicleta simplificado
    // dx/dt = v * cos(psi)
    // dy/dt = v * sin(psi)
    // dpsi/dt = (v / L) * tan(delta)

    new_state.x += state.v * std::cos(state.psi) * dt;
    new_state.y += state.v * std::sin(state.psi) * dt;
    new_state.psi += (state.v / params_.L) * std::tan(steering_cmd) * dt;
    new_state.v = speed_cmd;
    new_state.delta = steering_cmd;

    // Normaliza o ângulo psi para [-π, π]
    while (new_state.psi > M_PI) new_state.psi -= 2 * M_PI;
    while (new_state.psi < -M_PI) new_state.psi += 2 * M_PI;

    return new_state;
}

} // namespace mpc
