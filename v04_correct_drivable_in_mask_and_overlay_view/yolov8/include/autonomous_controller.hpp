#ifndef AUTONOMOUS_CONTROLLER_HPP
#define AUTONOMOUS_CONTROLLER_HPP

#include "mpc_controller.hpp"
#include "waypoint_extractor.hpp"
#include "vehicle_model.hpp"
#include "jetracer.hpp"
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <algorithm> // para std::min, std::max

namespace mpc {

struct ControlConfig {
    double max_steering_angle;      // Ângulo máximo de direção (graus)
    double max_speed;               // Velocidade máxima (m/s)
    double min_speed;               // Velocidade mínima (m/s)
    double control_frequency;       // Frequência de controle (Hz)
    bool enable_safety_checks;      // Habilita verificações de segurança
    double emergency_stop_distance; // Distância para parada de emergência (metros)

    ControlConfig() :
        max_steering_angle(140.0),  // Igual ao MAX_ANGLE_ do jetracer.hpp
        max_speed(3.0),
        min_speed(0.5),
        control_frequency(20.0),
        enable_safety_checks(true),
        emergency_stop_distance(1.0) {}
};

enum class ControlMode {
    MANUAL,           // Controle manual completo
    STEERING_ASSIST,  // Velocidade manual + direção automática
    FULL_AUTONOMOUS   // Controle autônomo completo
};

struct ControlStatus {
    ControlMode control_mode;       // Modo de controle atual
    bool is_autonomous;             // Modo autônomo ativo (para compatibilidade)
    bool is_emergency_stop;         // Parada de emergência ativa
    double current_speed;           // Velocidade atual
    double current_steering;        // Direção atual
    double cross_track_error;       // Erro lateral
    double heading_error;           // Erro de direção
    int waypoints_count;            // Número de waypoints
    double control_latency_ms;      // Latência do controle

    ControlStatus() :
        control_mode(ControlMode::MANUAL),
        is_autonomous(false),
        is_emergency_stop(false),
        current_speed(0.0),
        current_steering(0.0),
        cross_track_error(0.0),
        heading_error(0.0),
        waypoints_count(0),
        control_latency_ms(0.0) {}
};

class AutonomousController {
public:
    AutonomousController(const ControlConfig& config = ControlConfig());
    ~AutonomousController();

    // Inicializa o controlador
    bool initialize(int servo_addr = 0x40, int motor_addr = 0x60);

    // Inicia o controle autônomo
    void startAutonomousControl();

    // Para o controle autônomo
    void stopAutonomousControl();

    // Atualiza a detecção de lanes e executa controle
    void updateControl(const std::vector<Detection>& detections,
                      const std::vector<cv::Mat>& masks,
                      const std::unordered_map<int, std::string>& labels_map);

    // Habilita/desabilita modo autônomo
    void setAutonomousMode(bool enabled);

    // Define modo de controle
    void setControlMode(ControlMode mode);

    // Aplica controle híbrido (velocidade manual + direção automática)
    void applySteeringAssist(const std::vector<Detection>& detections,
                            const std::vector<cv::Mat>& masks,
                            const std::unordered_map<int, std::string>& labels_map,
                            double manual_speed);

    // Obtém status atual
    ControlStatus getStatus() const;

    // Atualiza configuração
    void updateConfig(const ControlConfig& config);

    // Obtém configuração atual
    ControlConfig getConfig() const;

    // Obtém extrator de waypoints para configuração
    WaypointExtractor* getWaypointExtractor() const;

    // Função de emergência
    void emergencyStop();

    // Reseta o controlador
    void reset();
    
    // Métodos de verificação de segurança
    bool shouldActivateEmergencyStop(const std::vector<Detection>& detections,
                                   const std::vector<cv::Mat>& masks,
                                   const std::unordered_map<int, std::string>& labels_map);
    bool hasObstacleInLanes(const std::vector<Detection>& detections,
                           const std::vector<cv::Mat>& masks,
                           const std::unordered_map<int, std::string>& labels_map);
    bool hasLaneDetectionFailure(const std::vector<Detection>& detections,
                                const std::unordered_map<int, std::string>& labels_map);
    
    // Função auxiliar para verificação de área crítica
    bool isObstacleInCenterArea(const float bbox[4]);
    
    // Função para visualização da área crítica
    cv::Mat visualizeCriticalArea(const cv::Mat& frame, 
                                 const std::vector<Detection>& detections,
                                 const std::vector<cv::Mat>& masks,
                                 const std::unordered_map<int, std::string>& labels_map);

private:
    ControlConfig config_;
    ControlStatus status_;

    // Componentes do sistema
    std::unique_ptr<MPCController> mpc_controller_;
    std::unique_ptr<WaypointExtractor> waypoint_extractor_;
    std::unique_ptr<VehicleModel> vehicle_model_;
    std::unique_ptr<jetracer::control::JetRacer> jetracer_;

    // Thread de controle
    std::thread control_thread_;
    std::atomic<bool> running_;
    mutable std::mutex status_mutex_;

    // Estado do veículo
    VehicleState current_vehicle_state_;

    // Funções internas
    void controlLoop();
    void applyControl(const ControlOutput& control_output);
    void applyHybridControl(double manual_speed, double auto_steering);
    bool safetyCheck(const std::vector<cv::Point2f>& waypoints);
    void updateVehicleState();
    double radiansToDegrees(double radians);
    double degreesToRadians(double degrees);

    // Conversão de coordenadas
    std::vector<cv::Point2f> convertWaypointsToWorld(const std::vector<Waypoint>& waypoints);

    // Logging e debug
    void logControlData(const ControlOutput& control_output,
                       const std::vector<cv::Point2f>& waypoints);
};

} // namespace mpc

#endif // AUTONOMOUS_CONTROLLER_HPP
