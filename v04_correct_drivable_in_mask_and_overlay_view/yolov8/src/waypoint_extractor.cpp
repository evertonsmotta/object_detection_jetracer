#include "waypoint_extractor.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>

namespace mpc {

WaypointExtractor::WaypointExtractor(int image_width, int image_height)
    : image_width_(image_width),
      image_height_(image_height),
      num_waypoints_(20),
      min_lane_width_(10.0),
      max_lane_width_(100.0),
      temporal_stabilization_enabled_(true),
      stabilization_history_size_(5),         // Reduzido drasticamente de 15 para 5 frames
      position_stabilization_threshold_(30.0), // Aumentado drasticamente de 12.0 para 30.0 pixels
      angle_stabilization_threshold_(20.0) {   // Aumentado drasticamente de 5.0 para 20.0 graus

    last_stats_ = {0, 0, 0.0, 0.0};
}

std::vector<Waypoint> WaypointExtractor::extractWaypoints(
    const std::vector<Detection>& detections,
    const std::vector<cv::Mat>& masks,
    const std::unordered_map<int, std::string>& labels_map) {

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<Waypoint> all_waypoints;
    std::vector<std::vector<cv::Point2f>> lane_centerlines;

    // Extrai todas as centerlines das lanes
    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& detection = detections[i];

        // Verifica se é uma lane
        auto label_it = labels_map.find(detection.class_id);
        if (label_it == labels_map.end() || label_it->second != "lane") {
            continue;
        }

        // Extrai linha central da máscara
        auto centerline = extractLaneCenterline(masks[i]);
        lane_centerlines.push_back(centerline);

        // Converte para waypoints
        for (size_t j = 0; j < centerline.size(); ++j) {
            if (isValidWaypoint(centerline[j])) {
                double confidence = detection.conf;
                int lane_id = detection.class_id;

                all_waypoints.emplace_back(centerline[j], confidence, lane_id);
            }
        }
    }

    // ESTRATÉGIA PARA FAIXA ÚNICA: Estimativa do centro da pista
    std::vector<cv::Point2f> estimated_center_path;
    if (lane_centerlines.size() == 1 && !lane_centerlines[0].empty()) {
        estimated_center_path = estimateRoadCenterFromSingleLane(lane_centerlines[0]);

        // Adiciona waypoints estimados com confiança reduzida
        for (const auto& center_point : estimated_center_path) {
            if (isValidWaypoint(center_point)) {
                all_waypoints.emplace_back(center_point, 0.7, -1); // Confiança reduzida, ID especial
            }
        }

        std::cout << "[WaypointExtractor] Uma faixa detectada. Estimando centro da pista com "
                  << estimated_center_path.size() << " pontos." << std::endl;
    }

    // Filtra waypoints por qualidade
    auto filtered_waypoints = filterWaypoints(all_waypoints);

    // Suaviza a trajetória
    std::vector<cv::Point2f> positions;
    for (const auto& wp : filtered_waypoints) {
        positions.push_back(wp.position);
    }

    // Aplica estabilização temporal para evitar oscilações entre frames
    auto stabilized_positions = stabilizeWaypointsTemporally(positions);

    // Suaviza a trajetória estabilizada
    auto smoothed_positions = smoothTrajectory(stabilized_positions);

    // Converte de volta para waypoints
    std::vector<Waypoint> final_waypoints;
    for (size_t i = 0; i < smoothed_positions.size() && i < filtered_waypoints.size(); ++i) {
        final_waypoints.emplace_back(smoothed_positions[i],
                                    filtered_waypoints[i].confidence,
                                    filtered_waypoints[i].lane_id);
    }

    // Atualiza estatísticas
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    last_stats_.total_waypoints = all_waypoints.size();
    last_stats_.valid_lanes = detections.size();
    last_stats_.processing_time_ms = duration.count() / 1000.0;

    if (!final_waypoints.empty()) {
        double total_confidence = 0.0;
        for (const auto& wp : final_waypoints) {
            total_confidence += wp.confidence;
        }
        last_stats_.average_confidence = total_confidence / final_waypoints.size();
    }

    return final_waypoints;
}

std::vector<cv::Point2f> WaypointExtractor::extractLaneCenterline(const cv::Mat& lane_mask) {
    std::vector<cv::Point2f> centerline;

    if (lane_mask.empty()) return centerline;

    // Converte para imagem binária
    cv::Mat binary_mask;
    cv::threshold(lane_mask, binary_mask, 0.5, 255, cv::THRESH_BINARY);
    binary_mask.convertTo(binary_mask, CV_8UC1);

    // Extrai pontos ao longo da altura da imagem
    int step = image_height_ / num_waypoints_;

    for (int y = image_height_ - 1; y >= 0; y -= step) {
        cv::Point2f center = findLaneCenterAtHeight(binary_mask, y);

        if (center.x >= 0 && center.y >= 0) {
            centerline.push_back(center);
        }
    }

    // Inverte a ordem para ter pontos do mais próximo ao mais distante
    std::reverse(centerline.begin(), centerline.end());

    return centerline;
}

cv::Point2f WaypointExtractor::findLaneCenterAtHeight(const cv::Mat& lane_mask, int height) {
    if (height < 0 || height >= lane_mask.rows) {
        return cv::Point2f(-1, -1);
    }

    std::vector<int> lane_pixels;

    // Encontra todos os pixels de lane nesta altura
    for (int x = 0; x < lane_mask.cols; ++x) {
        if (lane_mask.at<uchar>(height, x) > 0) {
            lane_pixels.push_back(x);
        }
    }

    if (lane_pixels.empty()) {
        return cv::Point2f(-1, -1);
    }

    // Calcula o centro
    double center_x = 0.0;
    for (int x : lane_pixels) {
        center_x += x;
    }
    center_x /= lane_pixels.size();

    return cv::Point2f(center_x, height);
}

double WaypointExtractor::calculateLaneWidthAtHeight(const cv::Mat& lane_mask, int height) {
    if (height < 0 || height >= lane_mask.rows) {
        return 0.0;
    }

    std::vector<int> lane_pixels;

    for (int x = 0; x < lane_mask.cols; ++x) {
        if (lane_mask.at<uchar>(height, x) > 0) {
            lane_pixels.push_back(x);
        }
    }

    if (lane_pixels.size() < 2) {
        return 0.0;
    }

    int min_x = *std::min_element(lane_pixels.begin(), lane_pixels.end());
    int max_x = *std::max_element(lane_pixels.begin(), lane_pixels.end());

    return static_cast<double>(max_x - min_x);
}

std::vector<Waypoint> WaypointExtractor::filterWaypoints(const std::vector<Waypoint>& waypoints,
                                                         double min_confidence) {
    std::vector<Waypoint> filtered;

    for (const auto& wp : waypoints) {
        if (wp.confidence >= min_confidence && isValidWaypoint(wp.position)) {
            filtered.push_back(wp);
        }
    }

    return filtered;
}

std::vector<cv::Point2f> WaypointExtractor::smoothTrajectory(const std::vector<cv::Point2f>& waypoints,
                                                             int window_size) {
    if (waypoints.size() < window_size) {
        return waypoints;
    }

    std::vector<cv::Point2f> smoothed;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        cv::Point2f smoothed_point = applyMovingAverage(waypoints, i, window_size);
        smoothed.push_back(smoothed_point);
    }

    return smoothed;
}

cv::Point2f WaypointExtractor::applyMovingAverage(const std::vector<cv::Point2f>& points,
                                                  int center_idx,
                                                  int window_size) {
    int half_window = window_size / 2;
    int start_idx = std::max(0, center_idx - half_window);
    int end_idx = std::min(static_cast<int>(points.size() - 1), center_idx + half_window);

    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;

    for (int i = start_idx; i <= end_idx; ++i) {
        sum_x += points[i].x;
        sum_y += points[i].y;
        count++;
    }

    if (count == 0) {
        return points[center_idx];
    }

    return cv::Point2f(sum_x / count, sum_y / count);
}

std::vector<cv::Point2f> WaypointExtractor::convertToWorldCoordinates(
    const std::vector<cv::Point2f>& pixel_waypoints,
    double scale_factor) {

    std::vector<cv::Point2f> world_waypoints;

    for (const auto& pixel_wp : pixel_waypoints) {
        // Converte do sistema de coordenadas da imagem para coordenadas do mundo
        // Origem no centro da imagem, Y cresce para cima
        double world_x = (pixel_wp.x - image_width_ / 2.0) * scale_factor;
        double world_y = (image_height_ / 2.0 - pixel_wp.y) * scale_factor;

        world_waypoints.emplace_back(world_x, world_y);
    }

    return world_waypoints;
}

void WaypointExtractor::setParameters(int num_waypoints, double min_lane_width, double max_lane_width) {
    num_waypoints_ = num_waypoints;
    min_lane_width_ = min_lane_width;
    max_lane_width_ = max_lane_width;
}

WaypointExtractor::ExtractionStats WaypointExtractor::getLastExtractionStats() const {
    return last_stats_;
}

bool WaypointExtractor::isValidWaypoint(const cv::Point2f& waypoint) {
    return waypoint.x >= 0 && waypoint.x < image_width_ &&
           waypoint.y >= 0 && waypoint.y < image_height_;
}

std::vector<cv::Point2f> WaypointExtractor::estimateRoadCenterFromSingleLane(
    const std::vector<cv::Point2f>& lane_centerline) {

    std::vector<cv::Point2f> estimated_center;

    if (lane_centerline.empty()) {
        return estimated_center;
    }

    // Calcula posição média da faixa para determinar se é esquerda ou direita
    double avg_x = 0.0;
    for (const auto& point : lane_centerline) {
        avg_x += point.x;
    }
    avg_x /= lane_centerline.size();

    double image_center_x = image_width_ / 2.0;
    bool is_left_lane = avg_x < image_center_x;

    // Estima largura típica da pista (em pixels)
    double typical_lane_width = image_width_ * 0.4; // 40% da largura da imagem

    std::cout << "[WaypointExtractor] Faixa detectada " << (is_left_lane ? "ESQUERDA" : "DIREITA")
              << " (pos: " << avg_x << ", centro: " << image_center_x << ")" << std::endl;

    for (const auto& lane_point : lane_centerline) {
        cv::Point2f estimated_center_point;

        if (is_left_lane) {
            // Se é faixa esquerda, estima centro da pista à direita
            estimated_center_point.x = lane_point.x + (typical_lane_width / 2.0);

            // CORREÇÃO AGRESSIVA: Se muito perto da borda, força retorno ao centro
            double distance_from_edge = lane_point.x;
            if (distance_from_edge < image_width_ * 0.1) { // Menos de 10% da largura
                // Força trajetória mais agressiva para o centro
                estimated_center_point.x = image_center_x + (typical_lane_width * 0.3); // Aumentado de 0.2 para 0.3
                std::cout << "[WaypointExtractor] CORREÇÃO AGRESSIVA: Muito perto da borda esquerda!" << std::endl;
            }

        } else {
            // Se é faixa direita, estima centro da pista à esquerda
            estimated_center_point.x = lane_point.x - (typical_lane_width / 2.0);

            // CORREÇÃO AGRESSIVA: Se muito perto da borda, força retorno ao centro
            double distance_from_edge = image_width_ - lane_point.x;
            if (distance_from_edge < image_width_ * 0.1) { // Menos de 10% da largura
                // Força trajetória mais agressiva para o centro
                estimated_center_point.x = image_center_x - (typical_lane_width * 0.3); // Aumentado de 0.2 para 0.3
                std::cout << "[WaypointExtractor] CORREÇÃO AGRESSIVA: Muito perto da borda direita!" << std::endl;
            }
        }

        // CORREÇÃO CRÍTICA: Ajusta direção baseado na posição relativa ao centro
        double center_offset = estimated_center_point.x - image_center_x;
        if (std::abs(center_offset) > image_width_ * 0.1) { // Se desviado mais de 10% do centro
            if (center_offset > 0) {
                // Carro está à esquerda do centro, força correção para direita
                estimated_center_point.x = image_center_x + (typical_lane_width * 0.2);
                std::cout << "[WaypointExtractor] CORREÇÃO DIREITA: Carro muito à esquerda, forçando direita!" << std::endl;
            } else {
                // Carro está à direita do centro, força correção para esquerda
                estimated_center_point.x = image_center_x - (typical_lane_width * 0.2);
                std::cout << "[WaypointExtractor] CORREÇÃO ESQUERDA: Carro muito à direita, forçando esquerda!" << std::endl;
            }
        }

        estimated_center_point.y = lane_point.y;

        // Garante que o ponto estimado está dentro dos limites da imagem
        estimated_center_point.x = std::max(0.0f,
            std::min(static_cast<float>(image_width_ - 1), estimated_center_point.x));

        estimated_center.push_back(estimated_center_point);
    }

    return estimated_center;
}

void WaypointExtractor::enableTemporalStabilization(bool enable, int history_size) {
    temporal_stabilization_enabled_ = enable;
    stabilization_history_size_ = history_size;

    if (enable) {
        std::cout << "[WaypointExtractor] Estabilização temporal ativada (histórico: " << history_size << " frames)" << std::endl;
    } else {
        std::cout << "[WaypointExtractor] Estabilização temporal desativada" << std::endl;
    }
}

void WaypointExtractor::setStabilizationParameters(double position_threshold, double angle_threshold) {
    position_stabilization_threshold_ = position_threshold;
    angle_stabilization_threshold_ = angle_threshold;

    std::cout << "[WaypointExtractor] Parâmetros de estabilização atualizados:" << std::endl;
    std::cout << "  - Threshold posição: " << position_threshold << " pixels" << std::endl;
    std::cout << "  - Threshold ângulo: " << angle_threshold << " graus" << std::endl;
}

std::vector<cv::Point2f> WaypointExtractor::stabilizeWaypointsTemporally(
    const std::vector<cv::Point2f>& current_waypoints) {

    if (!temporal_stabilization_enabled_ || current_waypoints.empty()) {
        return current_waypoints;
    }

    // Adiciona waypoints atuais ao histórico
    waypoint_history_.push_back(current_waypoints);

    // Mantém apenas o histórico especificado
    if (waypoint_history_.size() > stabilization_history_size_) {
        waypoint_history_.erase(waypoint_history_.begin());
    }

    // Se não temos histórico suficiente, retorna os atuais
    if (waypoint_history_.size() < 3) {
        return current_waypoints;
    }

    std::vector<cv::Point2f> stabilized_waypoints;

    // Para cada waypoint, calcula a média temporal
    for (size_t i = 0; i < current_waypoints.size(); ++i) {
        cv::Point2f stabilized_point(0.0f, 0.0f);
        int valid_count = 0;

        // Calcula média dos últimos frames
        for (const auto& historical_frame : waypoint_history_) {
            if (i < historical_frame.size()) {
                stabilized_point += historical_frame[i];
                valid_count++;
            }
        }

        if (valid_count > 0) {
            stabilized_point /= static_cast<float>(valid_count);

            // Aplica filtro de mudança máxima para evitar saltos bruscos
            if (!last_stable_waypoints_.empty() && i < last_stable_waypoints_.size()) {
                cv::Point2f last_point = last_stable_waypoints_[i];
                float distance = cv::norm(stabilized_point - last_point);

                if (distance > position_stabilization_threshold_) {
                    // Limita a mudança máxima de forma mais suave
                    cv::Point2f direction = stabilized_point - last_point;
                    direction = direction * (position_stabilization_threshold_ / distance);
                    stabilized_point = last_point + direction;

                    std::cout << "[WaypointExtractor] Estabilização: Limitei mudança de "
                              << distance << " para " << position_stabilization_threshold_ << " pixels" << std::endl;
                }

                // 3. Filtro de suavização MUITO mais responsivo para estabilidade extra
                double smoothing_factor = 0.9; // Aumentado drasticamente de 0.6 para 0.9 - Muito mais responsivo
                stabilized_point = (smoothing_factor * stabilized_point) + ((1.0 - smoothing_factor) * last_point);
            }

            stabilized_waypoints.push_back(stabilized_point);
        } else {
            stabilized_waypoints.push_back(current_waypoints[i]);
        }
    }

    // Atualiza waypoints estáveis para próximo frame
    last_stable_waypoints_ = stabilized_waypoints;

    return stabilized_waypoints;
}

} // namespace mpc
