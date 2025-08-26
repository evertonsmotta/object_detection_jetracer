#ifndef VEHICLE_MODEL_HPP
#define VEHICLE_MODEL_HPP

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace mpc {

struct VehicleState {
    double x;           // Posição X (metros)
    double y;           // Posição Y (metros)
    double psi;         // Ângulo de direção (radianos)
    double v;           // Velocidade (m/s)
    double delta;       // Ângulo do volante atual (radianos)

    VehicleState() : x(0), y(0), psi(0), v(0), delta(0) {}
    VehicleState(double x, double y, double psi, double v, double delta)
        : x(x), y(y), psi(psi), v(v), delta(delta) {}
};

struct VehicleParams {
    double L;           // Distância entre eixos (metros)
    double max_steering; // Ângulo máximo de direção (radianos)
    double max_speed;   // Velocidade máxima (m/s)
    double min_speed;   // Velocidade mínima (m/s)

    VehicleParams() : L(0.2), max_steering(2.44), max_speed(5.0), min_speed(0.1) {} // 2.44 rad = 140°
};

class VehicleModel {
public:
    VehicleModel(const VehicleParams& params = VehicleParams());

    // Atualiza o estado do veículo baseado nos comandos de controle
    VehicleState update(const VehicleState& current_state,
                       double steering_cmd,
                       double speed_cmd,
                       double dt);

    // Prediz o estado futuro para um horizonte de tempo
    std::vector<VehicleState> predict(const VehicleState& current_state,
                                     const std::vector<double>& steering_sequence,
                                     const std::vector<double>& speed_sequence,
                                     double dt);

    // Calcula o erro lateral (cross-track error)
    double calculateCrossTrackError(const VehicleState& state,
                                   const std::vector<cv::Point2f>& waypoints);

    // Calcula o ângulo de direção desejado para seguir os waypoints
    double calculateDesiredHeading(const VehicleState& state,
                                  const std::vector<cv::Point2f>& waypoints);

    // Converte coordenadas de pixel para metros
    cv::Point2f pixelToMeters(const cv::Point2f& pixel,
                               double scale_factor = 0.01); // 1cm por pixel

    // Converte coordenadas de metros para pixel
    cv::Point2f metersToPixel(const cv::Point2f& meters,
                               double scale_factor = 0.01);

private:
    VehicleParams params_;

    // Modelo de bicicleta simplificado
    VehicleState bicycleModel(const VehicleState& state,
                             double steering_cmd,
                             double speed_cmd,
                             double dt);
};

} // namespace mpc

#endif // VEHICLE_MODEL_HPP
