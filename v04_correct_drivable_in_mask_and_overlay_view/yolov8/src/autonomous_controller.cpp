#include "autonomous_controller.hpp"
#include <iostream>
#include <chrono>
#include <cmath>

namespace mpc {

AutonomousController::AutonomousController(const ControlConfig& config)
    : config_(config), running_(false) {

    // Inicializa componentes
    mpc_controller_.reset(new MPCController());
    waypoint_extractor_.reset(new WaypointExtractor(320, 320)); // Tamanho da imagem
    vehicle_model_.reset(new VehicleModel());

    // Inicializa estado do veículo
    current_vehicle_state_ = VehicleState();
}

AutonomousController::~AutonomousController() {
    stopAutonomousControl();
}

bool AutonomousController::initialize(int servo_addr, int motor_addr) {
    try {
        jetracer_.reset(new jetracer::control::JetRacer(servo_addr, motor_addr));
        jetracer_->start();

        std::cout << "[MPC] JetRacer inicializado com sucesso" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[MPC] Erro ao inicializar JetRacer: " << e.what() << std::endl;
        return false;
    }
}

void AutonomousController::startAutonomousControl() {
    if (running_.load()) {
        std::cout << "[MPC] Controle autônomo já está rodando" << std::endl;
        return;
    }

    running_ = true;
    control_thread_ = std::thread(&AutonomousController::controlLoop, this);

    std::cout << "[MPC] Controle autônomo iniciado" << std::endl;
}

void AutonomousController::stopAutonomousControl() {
    if (!running_.load()) {
        return;
    }

    running_ = false;

    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    // Para o veículo
    if (jetracer_) {
        jetracer_->set_speed(0, 0);
        jetracer_->set_steering(0);
    }

    std::cout << "[MPC] Controle autônomo parado" << std::endl;
}

void AutonomousController::applySteeringAssist(const std::vector<Detection>& detections,
                                               const std::vector<cv::Mat>& masks,
                                               const std::unordered_map<int, std::string>& labels_map,
                                               double manual_speed) {
    if (!running_.load() || status_.is_emergency_stop) {
        return;
    }

    try {
        // ====== VERIFICAÇÃO DE SEGURANÇA ANTES DE PROCESSAR ======
        if (shouldActivateEmergencyStop(detections, masks, labels_map)) {
            std::cout << "[MPC] 🚨 ATIVANDO EMERGENCY STOP por condições de segurança!" << std::endl;
            emergencyStop();
            return;
        }

        // Extrai waypoints das lanes detectadas
        auto waypoints = waypoint_extractor_->extractWaypoints(detections, masks, labels_map);

        if (waypoints.empty()) {
            std::cout << "[MPC] Nenhum waypoint detectado para assistência" << std::endl;
            return;
        }

        // Converte waypoints para coordenadas do mundo
        auto world_waypoints = convertWaypointsToWorld(waypoints);

        // Atualiza estado do veículo
        updateVehicleState();

        // Executa MPC apenas para calcular direção
        auto control_output = mpc_controller_->solve(current_vehicle_state_, world_waypoints);

        // FILTROS DE ESTABILIZAÇÃO PARA MODO ASSISTÊNCIA
        double stabilized_steering = control_output.steering;

        // 1. Filtro de velocidade mínima: só aplica direção se estiver se movendo
        double min_speed_threshold = 0.1; // 0.1 m/s = 0.36 km/h
        if (std::abs(manual_speed) < min_speed_threshold) {
            // Se parado, mantém direção anterior ou centraliza
            static double last_steering = 0.0;
            stabilized_steering = last_steering * 0.9; // Decaimento mais suave (0.9 em vez de 0.8)
            std::cout << "[MPC] Assistência: Veículo parado, mantendo direção anterior" << std::endl;
        } else {
            // Se em movimento, aplica estabilização temporal MUITO mais responsiva
            static double last_steering = 0.0;
            double max_steering_change = 12.0; // Ajuste AQUI - Máximo 12° por frame (em graus!)

            double steering_change = control_output.steering - last_steering;
            if (std::abs(steering_change) > max_steering_change) {
                // Limita mudança brusca
                if (steering_change > 0) {
                    stabilized_steering = last_steering + max_steering_change;
                } else {
                    stabilized_steering = last_steering - max_steering_change;
                }
                std::cout << "[MPC] Assistência: Limitei mudança de direção de "
                          << steering_change << "° para " << max_steering_change << "°" << std::endl;
            }

            // Filtro de suavização MUITO mais responsivo para direção
            double smoothing_factor = 0.9; // Ajuste AQUI - 0.9 = 90% novo + 10% anterior
            stabilized_steering = (smoothing_factor * stabilized_steering) + ((1.0 - smoothing_factor) * last_steering);

            last_steering = stabilized_steering;
        }

        // Aplica controle híbrido: velocidade manual + direção automática estabilizada
        applyHybridControl(manual_speed, stabilized_steering);

        // Atualiza status
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_.cross_track_error = control_output.cross_track_error;
            status_.heading_error = control_output.heading_error;
            status_.waypoints_count = waypoints.size();
            status_.current_speed = manual_speed;
            status_.current_steering = radiansToDegrees(control_output.steering);
        }

        std::cout << "[MPC] Assistência aplicada - Speed: " << manual_speed
                  << " m/s (manual), Steering: " << control_output.steering
                  << "° (auto)" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[MPC] Erro na assistência de direção: " << e.what() << std::endl;
    }
}

void AutonomousController::updateControl(const std::vector<Detection>& detections,
                                        const std::vector<cv::Mat>& masks,
                                        const std::unordered_map<int, std::string>& labels_map) {

    if (!running_.load() || status_.control_mode != ControlMode::FULL_AUTONOMOUS) {
        return;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // ====== VERIFICAÇÃO DE SEGURANÇA ANTES DE PROCESSAR ======
        if (shouldActivateEmergencyStop(detections, masks, labels_map)) {
            std::cout << "[MPC] 🚨 ATIVANDO EMERGENCY STOP por condições de segurança!" << std::endl;
            emergencyStop();
            return;
        }

        // Extrai waypoints das lanes detectadas
        auto waypoints = waypoint_extractor_->extractWaypoints(detections, masks, labels_map);

        std::cout << "[MPC] Waypoints extraídos: " << waypoints.size() << std::endl;

        // Verificação de segurança adicional (se habilitada)
        if (config_.enable_safety_checks && !safetyCheck(convertWaypointsToWorld(waypoints))) {
            emergencyStop();
            return;
        }

        // Converte waypoints para coordenadas do mundo
        auto world_waypoints = convertWaypointsToWorld(waypoints);

        std::cout << "[MPC] World waypoints: " << world_waypoints.size() << std::endl;

        // Atualiza estado do veículo
        updateVehicleState();

        // Executa MPC
        auto control_output = mpc_controller_->solve(current_vehicle_state_, world_waypoints);

        std::cout << "[MPC] Comando MPC - Steering: " << control_output.steering
                  << "°, Speed: " << control_output.speed << " m/s" << std::endl;

        // FILTRO DE SUAVIZAÇÃO PARA DIREÇÃO - MUITO MAIS RESPONSIVO
        static double last_steering = 0.0;
        double max_steering_change = 15.0; // Ajuste AQUI - Máximo 15° por frame (em graus!)

        double steering_change = control_output.steering - last_steering;
        if (std::abs(steering_change) > max_steering_change) {
            if (steering_change > 0) {
                control_output.steering = last_steering + max_steering_change;
            } else {
                control_output.steering = last_steering - max_steering_change;
            }
            std::cout << "[MPC] Limitei mudança de direção de "
                      << steering_change << "° para " << max_steering_change << "°" << std::endl;
        }

        // Filtro de suavização MUITO mais responsivo
        double smoothing_factor = 0.95; // Ajuste AQUI - 0.95 = 95% novo + 5% anterior
        control_output.steering = (smoothing_factor * control_output.steering) + ((1.0 - smoothing_factor) * last_steering);
        last_steering = control_output.steering;

        // Aplica controle
        applyControl(control_output);

        // Atualiza status
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_.cross_track_error = control_output.cross_track_error;
            status_.heading_error = control_output.heading_error; // Já está em graus
            status_.waypoints_count = waypoints.size();
            status_.current_speed = current_vehicle_state_.v;
            status_.current_steering = control_output.steering; // Já está em graus
        }

        // Log dos dados de controle
        logControlData(control_output, world_waypoints);

        // Calcula latência
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_.control_latency_ms = duration.count() / 1000.0;
        }

    } catch (const std::exception& e) {
        std::cerr << "[MPC] Erro no controle: " << e.what() << std::endl;
        emergencyStop();
    }
}

void AutonomousController::setAutonomousMode(bool enabled) {
    if (enabled) {
        setControlMode(ControlMode::FULL_AUTONOMOUS);
    } else {
        setControlMode(ControlMode::MANUAL);
    }
}

void AutonomousController::setControlMode(ControlMode mode) {
    std::lock_guard<std::mutex> lock(status_mutex_);

    ControlMode old_mode = status_.control_mode;
    status_.control_mode = mode;

    // Atualiza o flag is_autonomous para compatibilidade
    status_.is_autonomous = (mode == ControlMode::FULL_AUTONOMOUS);

    if (old_mode != mode) {
        switch (mode) {
            case ControlMode::MANUAL:
                if (running_.load()) {
                    stopAutonomousControl();
                }
                if (jetracer_) {
                    jetracer_->set_speed(0, 0);
                    jetracer_->set_steering(0);
                }
                std::cout << "[MPC] Modo MANUAL ativado" << std::endl;
                break;

            case ControlMode::STEERING_ASSIST:
                if (!running_.load()) {
                    startAutonomousControl();
                }
                std::cout << "[MPC] Modo ASSISTÊNCIA DE DIREÇÃO ativado" << std::endl;
                break;

            case ControlMode::FULL_AUTONOMOUS:
                if (!running_.load()) {
                    startAutonomousControl();
                }
                std::cout << "[MPC] Modo AUTÔNOMO COMPLETO ativado" << std::endl;
                break;
        }
    }
}

ControlStatus AutonomousController::getStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

void AutonomousController::updateConfig(const ControlConfig& config) {
    config_ = config;

    // Atualiza parâmetros do MPC
    MPCParams mpc_params;
    mpc_params.dt = 1.0 / config_.control_frequency;
    mpc_controller_->updateParams(mpc_params);

    std::cout << "[MPC] Configuração atualizada" << std::endl;
}

ControlConfig AutonomousController::getConfig() const {
    return config_;
}

WaypointExtractor* AutonomousController::getWaypointExtractor() const {
    return waypoint_extractor_.get();
}

void AutonomousController::emergencyStop() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.is_emergency_stop = true;

    if (jetracer_) {
        jetracer_->set_speed(0, 0);
        jetracer_->set_steering(0);
    }

    std::cout << "[MPC] PARADA DE EMERGÊNCIA ATIVADA!" << std::endl;
}

void AutonomousController::reset() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.is_emergency_stop = false;

    // Reseta estado do veículo
    current_vehicle_state_ = VehicleState();

    // Reseta controlador MPC
    mpc_controller_->reset();

    std::cout << "[MPC] Controlador resetado" << std::endl;
}

// ====== IMPLEMENTAÇÃO DAS FUNÇÕES DE VERIFICAÇÃO DE SEGURANÇA ======

bool AutonomousController::shouldActivateEmergencyStop(const std::vector<Detection>& detections,
                                                      const std::vector<cv::Mat>& masks,
                                                      const std::unordered_map<int, std::string>& labels_map) {

    // Verificar as duas condições específicas para Emergency Stop

    // 1. DETECÇÃO DE OBSTÁCULOS PRÓXIMOS NAS LANES
    if (hasObstacleInLanes(detections, masks, labels_map)) {
        std::cout << "[SAFETY] 🚨 Emergency Stop: Obstáculo detectado nas lanes!" << std::endl;
        return true;
    }

    // 2. FALHA NA DETECÇÃO DE LANES
    if (hasLaneDetectionFailure(detections, labels_map)) {
        std::cout << "[SAFETY] 🚨 Emergency Stop: Falha na detecção de lanes!" << std::endl;
        return true;
    }

    // Se chegou até aqui, não há necessidade de Emergency Stop
    return false;
}

bool AutonomousController::hasObstacleInLanes(const std::vector<Detection>& detections,
                                             const std::vector<cv::Mat>& masks,
                                             const std::unordered_map<int, std::string>& labels_map) {

    // Verificar se há obstáculos que "mancham" significativamente a área drivable
    // na área crítica: do meio da imagem (30cm do carro) até a base da imagem
    // entre as faixas da pista onde o carro está localizado

    for (const auto& det : detections) {
        std::string label = labels_map.at(det.class_id);

        // Só considerar obstáculos (não-drivable) com alta confiança
        if (label != "drivable" && det.conf > 0.7) {

            // Calcular área do obstáculo
            double obstacle_area = det.bbox[2] * det.bbox[3]; // width * height

            // Calcular área total da imagem
            double total_image_area = 320.0 * 320.0; // kInputW * kInputH

            // Calcular porcentagem da imagem ocupada pelo obstáculo
            double obstacle_percentage = (obstacle_area / total_image_area) * 100.0;

            // Verificar se o obstáculo está na área crítica (entre as faixas, 30cm até base)
            bool is_in_critical_area = isObstacleInCenterArea(det.bbox);

            // ATIVAR EMERGENCY STOP se:
            // 1. Obstáculo com alta confiança (>70%)
            // 2. Ocupa área significativa (>5% da imagem)
            // 3. Está na área crítica (entre as faixas, do meio até a base)
            if (obstacle_percentage > 5.0 && is_in_critical_area) {
                std::cout << "[SAFETY] 🚨 Obstáculo crítico detectado: " << label
                          << " | Área: " << std::fixed << std::setprecision(1) << obstacle_percentage
                          << "% | Posição: (" << det.bbox[0] << ", " << det.bbox[1] << ")" << std::endl;
                return true;
            }
        }
    }

    return false;
}

// Função auxiliar para verificação de área crítica
// Área crítica: baseada nas lanes detectadas (mais larga na base, mais estreita no topo)
bool AutonomousController::isObstacleInCenterArea(const float bbox[4]) {
    // bbox[0] = x, bbox[1] = y, bbox[2] = width, bbox[3] = height

    // ATUALIZADO: Área crítica dinâmica baseada nas lanes
    // - Y: Do meio da imagem (160px) até a base (320px)
    // - X: Varia conforme a largura das lanes em cada altura

    double critical_y_min = 160.0;  // Meio da altura (320 * 0.5) - 30cm do carro
    double critical_y_max = 320.0;  // Base da imagem (320px) - mais próxima do carro

    // Verificar se o obstáculo está na faixa de altura crítica
    bool y_overlap = (bbox[1] < critical_y_max) && (bbox[1] + bbox[3] / 2.0) > critical_y_min;

    if (!y_overlap) {
        return false; // Obstáculo fora da faixa de altura crítica
    }

    // Calcular largura da área crítica na altura do obstáculo
    double obstacle_center_y = bbox[1] + bbox[3] / 2.0;

    // Aproximação da largura da área crítica baseada na altura
    // Base (320px): largura máxima (80% da imagem)
    // Meio (160px): largura mínima (50% da imagem)
    double height_ratio = (obstacle_center_y - critical_y_min) / (critical_y_max - critical_y_min);
    double critical_width_ratio = 0.5 + (height_ratio * 0.3); // 50% a 80%

    double critical_width = 320.0 * critical_width_ratio;
    double critical_x_min = (320.0 - critical_width) / 2.0;
    double critical_x_max = critical_x_min + critical_width;

    // Verificar se o obstáculo está na largura crítica
    bool x_overlap = (bbox[0] < critical_x_max) && (bbox[0] + bbox[2] > critical_x_min);

    // Log detalhado para debug
    if (x_overlap && y_overlap) {
        std::cout << "[SAFETY] 🎯 Obstáculo na área crítica dinâmica: "
                  << "X(" << bbox[0] << " a " << (bbox[0] + bbox[2]) << ") "
                  << "Y(" << bbox[1] << " a " << (bbox[1] + bbox[3]) << ")" << std::endl;
        std::cout << "[SAFETY] 📍 Área crítica na altura " << obstacle_center_y
                  << ": X(" << critical_x_min << " a " << critical_x_max
                  << ") Largura: " << critical_width << "px" << std::endl;
    }

    return x_overlap && y_overlap;
}

// ====== VISUALIZAÇÃO DA ÁREA CRÍTICA ======

cv::Mat AutonomousController::visualizeCriticalArea(const cv::Mat& frame,
                                                   const std::vector<Detection>& detections,
                                                   const std::vector<cv::Mat>& masks,
                                                   const std::unordered_map<int, std::string>& labels_map) {

    // Criar cópia da imagem para desenhar
    cv::Mat visualization = frame.clone();

    // ====== DETECTAR LANES E DEFINIR ÁREA CRÍTICA DINÂMICA ======
    std::vector<cv::Point> left_lane_points, right_lane_points;
    std::vector<cv::Point> critical_area_contour;

    // Extrair pontos das lanes detectadas
    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& det = detections[i];
        auto it = labels_map.find(det.class_id);
        if (it != labels_map.end() && it->second == "lane") {
            // Extrair contorno da máscara da lane
            if (i < masks.size() && !masks[i].empty()) {
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(masks[i], contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                if (!contours.empty()) {
                    // Encontrar o maior contorno (lane principal)
                    auto largest_contour = std::max_element(contours.begin(), contours.end(),
                        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                            return cv::contourArea(a) < cv::contourArea(b);
                        });

                    // Determinar se é lane esquerda ou direita baseado na posição X
                    cv::Rect bbox = cv::boundingRect(*largest_contour);
                    double center_x = bbox.x + bbox.width / 2.0;

                    if (center_x < 160) { // Lane esquerda
                        left_lane_points.insert(left_lane_points.end(), largest_contour->begin(), largest_contour->end());
                    } else { // Lane direita
                        right_lane_points.insert(right_lane_points.end(), largest_contour->begin(), largest_contour->end());
                    }
                }
            }
        }
    }

    // ====== CONSTRUIR ÁREA CRÍTICA DINÂMICA ======
    if (!left_lane_points.empty() && !right_lane_points.empty()) {
        // Definir alturas para amostragem (do meio até a base)
        std::vector<int> sample_heights = {160, 180, 200, 220, 240, 260, 280, 300, 320};

        for (int height : sample_heights) {
            // Encontrar pontos das lanes na altura específica
            cv::Point left_point, right_point;
            bool left_found = false, right_found = false;

            // Buscar ponto da lane esquerda na altura
            for (const auto& point : left_lane_points) {
                if (std::abs(point.y - height) < 5) { // Tolerância de 5px
                    left_point = point;
                    left_found = true;
                    break;
                }
            }

            // Buscar ponto da lane direita na altura
            for (const auto& point : right_lane_points) {
                if (std::abs(point.y - height) < 5) { // Tolerância de 5px
                    right_point = point;
                    right_found = true;
                    break;
                }
            }

            // Se encontrou ambos os pontos, adicionar à área crítica
            if (left_found && right_found) {
                critical_area_contour.push_back(left_point);
                critical_area_contour.push_back(right_point);
            }
        }

        // Ordenar pontos para formar contorno fechado
        if (critical_area_contour.size() >= 4) {
            // Ordenar pontos da esquerda para direita em cada altura
            std::sort(critical_area_contour.begin(), critical_area_contour.end(),
                [](const cv::Point& a, const cv::Point& b) {
                    if (std::abs(a.y - b.y) < 10) { // Mesma altura
                        return a.x < b.x; // Ordenar por X
                    }
                    return a.y < b.y; // Ordenar por Y
                });

            // Criar contorno fechado
            std::vector<cv::Point> closed_contour;
            closed_contour.insert(closed_contour.end(), critical_area_contour.begin(), critical_area_contour.end());

            // Fechar o contorno
            if (!closed_contour.empty()) {
                closed_contour.push_back(closed_contour.front());
            }

            // Desenhar área crítica dinâmica
            std::vector<std::vector<cv::Point>> contours_to_draw = {closed_contour};
            cv::fillPoly(visualization, contours_to_draw, cv::Scalar(0, 255, 0, 128)); // Verde transparente
            cv::polylines(visualization, contours_to_draw, false, cv::Scalar(0, 255, 0), 2); // Borda verde

            // Adicionar texto explicativo
            cv::putText(visualization, "AREA CRITICA DINAMICA (Lanes)",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
    } else {
        // Fallback: usar área crítica retangular se não detectar lanes
        double critical_x_min = 80.0;   // 25% da largura
        double critical_x_max = 240.0;  // 75% da largura
        double critical_y_min = 160.0;  // Meio da altura - 30cm do carro
        double critical_y_max = 320.0;  // Base da imagem

        cv::Rect critical_area(critical_x_min, critical_y_min,
                              critical_x_max - critical_x_min,
                              critical_y_max - critical_y_min);

        cv::Mat overlay = visualization.clone();
        cv::rectangle(overlay, critical_area, cv::Scalar(0, 255, 0), -1);
        cv::addWeighted(visualization, 0.7, overlay, 0.3, 0, visualization);
        cv::rectangle(visualization, critical_area, cv::Scalar(0, 255, 0), 2);

        cv::putText(visualization, "AREA CRITICA PADRAO (Fallback)",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }

    // Visualizar detecções na área crítica
    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& det = detections[i];
        auto it = labels_map.find(det.class_id);
        if (it != labels_map.end()) {
            std::string label = it->second;

            // Calcular área do objeto
            double obstacle_area = det.bbox[2] * det.bbox[3];
            double total_image_area = 320.0 * 320.0;
            double obstacle_percentage = (obstacle_area / total_image_area) * 100.0;

            // Verificar se está na área crítica
            bool is_in_critical_area = isObstacleInCenterArea(det.bbox);

            // Definir cor baseada no tipo e posição
            cv::Scalar color;
            std::string status_text;

            if (label == "drivable") {
                color = cv::Scalar(0, 255, 0); // Verde para área drivable
                status_text = "DRIVABLE";
            } else if (is_in_critical_area && obstacle_percentage > 5.0) {
                color = cv::Scalar(0, 0, 255); // Vermelho para obstáculo crítico
                status_text = "OBSTACULO CRITICO!";
            } else if (is_in_critical_area) {
                color = cv::Scalar(0, 165, 255); // Laranja para obstáculo na área crítica
                status_text = "OBSTACULO NA AREA";
            } else {
                color = cv::Scalar(128, 128, 128); // Cinza para obstáculo distante
                status_text = "OBSTACULO DISTANTE";
            }

            // Desenhar bounding box
            cv::Rect bbox_rect(det.bbox[0], det.bbox[1], det.bbox[2], det.bbox[3]);
            cv::rectangle(visualization, bbox_rect, color, 2);

            // Adicionar texto com informações
            std::string info_text = label + " (" + std::to_string(int(obstacle_percentage)) + "%)";
            cv::putText(visualization, info_text,
                        cv::Point(det.bbox[0], det.bbox[1] - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);

            // Adicionar status se estiver na área crítica
            if (is_in_critical_area) {
                cv::putText(visualization, status_text,
                            cv::Point(det.bbox[0], det.bbox[1] + det.bbox[3] + 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            }

            // Desenhar máscara se disponível
            if (i < masks.size() && !masks[i].empty()) {
                cv::Mat mask_resized;
                cv::resize(masks[i], mask_resized, cv::Size(det.bbox[2], det.bbox[3]));

                // Aplicar máscara com transparência
                cv::Mat roi = visualization(bbox_rect);
                cv::Mat mask_roi;
                cv::resize(masks[i], mask_roi, roi.size());

                // Converter máscara para 3 canais
                cv::Mat mask_3ch;
                cv::cvtColor(mask_roi, mask_3ch, cv::COLOR_GRAY2BGR);

                // Aplicar máscara com transparência
                cv::addWeighted(roi, 0.7, mask_3ch, 0.3, 0, roi);
            }
        }
    }

    // Adicionar legenda
    cv::putText(visualization, "LEGENDA:", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(visualization, "VERDE: Area Critica + Drivable", cv::Point(10, 55),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    cv::putText(visualization, "VERMELHO: Obstaculo Critico (Emergency Stop)", cv::Point(10, 75),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    cv::putText(visualization, "LARANJA: Obstaculo na Area Critica", cv::Point(10, 95),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 165, 255), 1);
    cv::putText(visualization, "CINZA: Obstaculo Distante (Ignorado)", cv::Point(10, 115),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 128, 128), 1);

    return visualization;
}

bool AutonomousController::hasLaneDetectionFailure(const std::vector<Detection>& detections,
                                                  const std::unordered_map<int, std::string>& labels_map) {

    // Contar lanes detectadas
    int lane_count = 0;
    int drivable_count = 0;

    for (const auto& det : detections) {
        std::string label = labels_map.at(det.class_id);

        if (label == "lane") {
            lane_count++;
        } else if (label == "drivable") {
            drivable_count++;
        }
    }

    // FALHA NA DETECÇÃO DE LANES:
    // 1. Nenhuma lane detectada
    // 2. Apenas 1 lane (insuficiente para estimar centro da pista)
    // 3. Baixa confiança nas detecções

    if (lane_count == 0) {
        std::cout << "[SAFETY] 🚨 Falha: Nenhuma lane detectada!" << std::endl;
        return true;
    }

    // ATUALIZADO: Com 1 lane é possível estimar o centro da pista
    // Só ativar Emergency Stop se não houver lanes
    if (lane_count >= 1) {
        // Verificar confiança das detecções
        double total_confidence = 0.0;
        int valid_detections = 0;

        for (const auto& det : detections) {
            std::string label = labels_map.at(det.class_id);
            if (label == "lane" || label == "drivable") {
                total_confidence += det.conf;
                valid_detections++;
            }
        }

        if (valid_detections > 0) {
            double avg_confidence = total_confidence / valid_detections;
            if (avg_confidence < 0.6) {
                std::cout << "[SAFETY] 🚨 Falha: Baixa confiança média (" << std::fixed << std::setprecision(2) << avg_confidence
                          << ") nas detecções de lanes!" << std::endl;
                return true;
            }
        }

        // Se chegou até aqui, a detecção de lanes está funcionando
        // Mesmo com apenas 1 lane, é possível estimar o centro
        std::cout << "[SAFETY] ✅ Lanes detectadas: " << lane_count
                  << " | Confiança: " << std::fixed << std::setprecision(2)
                  << (valid_detections > 0 ? total_confidence / valid_detections : 0.0) << std::endl;
        return false;
    }

    // Caso extremo: nenhuma lane detectada
    return true;
}

void AutonomousController::controlLoop() {
    const auto control_interval = std::chrono::microseconds(
        static_cast<long>(1000000.0 / config_.control_frequency));

    while (running_.load()) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Loop principal de controle
        // O controle real é executado em updateControl()

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        if (elapsed < control_interval) {
            std::this_thread::sleep_for(control_interval - elapsed);
        }
    }
}

void AutonomousController::applyControl(const ControlOutput& control_output) {
    if (!jetracer_ || status_.is_emergency_stop) {
        std::cout << "[MPC] Não aplicando controle - jetracer: " << (jetracer_ ? "OK" : "NULL")
                  << ", emergency: " << status_.is_emergency_stop << std::endl;
        return;
    }

    try {
        // Direção já está em graus, não precisa converter
        double steering_degrees = control_output.steering;

        // Limita ângulo de direção
        steering_degrees = std::max(-config_.max_steering_angle,
                                   std::min(steering_degrees, config_.max_steering_angle));

        // Converte velocidade de m/s para porcentagem (-100 a 100)
        double speed_percentage = (control_output.speed / config_.max_speed) * 100.0;
        speed_percentage = std::max(-100.0, std::min(speed_percentage, 100.0));

        std::cout << "[MPC] Aplicando controle - Speed: " << speed_percentage
                  << "%, Steering: " << steering_degrees << "°" << std::endl;

        // Aplica controle
        jetracer_->set_speed(speed_percentage, steering_degrees);
        jetracer_->set_steering(static_cast<int>(steering_degrees));

    } catch (const std::exception& e) {
        std::cerr << "[MPC] Erro ao aplicar controle: " << e.what() << std::endl;
        emergencyStop();
    }
}

void AutonomousController::applyHybridControl(double manual_speed, double auto_steering) {
    if (!jetracer_ || status_.is_emergency_stop) {
        return;
    }

    try {
        // Direção já está em graus, não precisa converter
        double steering_degrees = auto_steering;

        // Limita ângulo de direção
        steering_degrees = std::max(-config_.max_steering_angle,
                                   std::min(steering_degrees, config_.max_steering_angle));

        // Converte velocidade de m/s para porcentagem (-100 a 100)
        double speed_percentage = (manual_speed / config_.max_speed) * 100.0;
        speed_percentage = std::max(-100.0, std::min(speed_percentage, 100.0));

        // Aplica controle híbrido
        jetracer_->set_speed(speed_percentage, steering_degrees);
        jetracer_->set_steering(static_cast<int>(steering_degrees));

    } catch (const std::exception& e) {
        std::cerr << "[MPC] Erro ao aplicar controle híbrido: " << e.what() << std::endl;
        emergencyStop();
    }
}

bool AutonomousController::safetyCheck(const std::vector<cv::Point2f>& waypoints) {
    if (waypoints.empty()) {
        std::cout << "[MPC] Aviso: Nenhum waypoint detectado" << std::endl;
        return false;
    }

    // Verifica se há waypoints muito próximos (parada de emergência)
    for (const auto& wp : waypoints) {
        double distance = std::sqrt(wp.x * wp.x + wp.y * wp.y);
        if (distance < config_.emergency_stop_distance) {
            std::cout << "[MPC] Aviso: Waypoint muito próximo (" << distance << "m)" << std::endl;
            // Não retorna false imediatamente, apenas loga o aviso
        }
    }

    // Sempre retorna true para permitir controle (apenas loga avisos)
    return true;
}

void AutonomousController::updateVehicleState() {
    // Atualiza estado baseado no controle anterior
    // Em uma implementação real, isso viria de sensores (GPS, IMU, etc.)

    // Simula atualização do estado
        current_vehicle_state_.v = std::max(config_.min_speed,
                                       std::min(current_vehicle_state_.v, config_.max_speed));
}

double AutonomousController::radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

double AutonomousController::degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

std::vector<cv::Point2f> AutonomousController::convertWaypointsToWorld(
    const std::vector<Waypoint>& waypoints) {

    std::vector<cv::Point2f> world_coords;

    for (const auto& wp : waypoints) {
        world_coords.push_back(wp.position);
    }

    return waypoint_extractor_->convertToWorldCoordinates(world_coords);
}

void AutonomousController::logControlData(const ControlOutput& control_output,
                                         const std::vector<cv::Point2f>& waypoints) {
    static int frame_count = 0;
    frame_count++;

    if (frame_count % 30 == 0) { // Log a cada 30 frames
        std::cout << "[MPC] Frame " << frame_count
                  << " | Steering: " << control_output.steering << "°"
                  << " | Speed: " << control_output.speed << " m/s"
                  << " | CTE: " << control_output.cross_track_error << " m"
                  << " | HE: " << control_output.heading_error << "°"
                  << " | Waypoints: " << waypoints.size()
                  << " | Latency: " << status_.control_latency_ms << " ms" << std::endl;
    }
}

} // namespace mpc
