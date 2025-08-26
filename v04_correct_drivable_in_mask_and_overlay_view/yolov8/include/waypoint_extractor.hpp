#ifndef WAYPOINT_EXTRACTOR_HPP
#define WAYPOINT_EXTRACTOR_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include "postprocess.h"

namespace mpc {

struct Waypoint {
    cv::Point2f position;      // Posição em pixels
    double confidence;          // Confiança da detecção
    int lane_id;               // ID da lane

    Waypoint(const cv::Point2f& pos, double conf, int id)
        : position(pos), confidence(conf), lane_id(id) {}
};

class WaypointExtractor {
public:
    WaypointExtractor(int image_width = 320, int image_height = 320);

    // Extrai waypoints das lanes detectadas
    std::vector<Waypoint> extractWaypoints(const std::vector<Detection>& detections,
                                           const std::vector<cv::Mat>& masks,
                                           const std::unordered_map<int, std::string>& labels_map);

    // Extrai linha central de uma máscara de lane
    std::vector<cv::Point2f> extractLaneCenterline(const cv::Mat& lane_mask);

    // Filtra waypoints por qualidade
    std::vector<Waypoint> filterWaypoints(const std::vector<Waypoint>& waypoints,
                                         double min_confidence = 0.5);

    // Suaviza a trajetória dos waypoints
    std::vector<cv::Point2f> smoothTrajectory(const std::vector<cv::Point2f>& waypoints,
                                              int window_size = 5);

    // Converte waypoints para coordenadas do mundo real
    std::vector<cv::Point2f> convertToWorldCoordinates(const std::vector<cv::Point2f>& pixel_waypoints,
                                                       double scale_factor = 0.01);

    // Configura parâmetros de extração
    void setParameters(int num_waypoints, double min_lane_width, double max_lane_width);

    // Obtém estatísticas dos waypoints extraídos
    struct ExtractionStats {
        int total_waypoints;
        int valid_lanes;
        double average_confidence;
        double processing_time_ms;
    };

    ExtractionStats getLastExtractionStats() const;

    // Estabilização temporal para evitar oscilações
    void enableTemporalStabilization(bool enable, int history_size = 10);
    void setStabilizationParameters(double position_threshold, double angle_threshold);

    // Estabiliza waypoints usando histórico temporal
    std::vector<cv::Point2f> stabilizeWaypointsTemporally(const std::vector<cv::Point2f>& current_waypoints);

private:
    int image_width_;
    int image_height_;
    int num_waypoints_;
    double min_lane_width_;
    double max_lane_width_;
    ExtractionStats last_stats_;

    // Parâmetros de estabilização temporal
    bool temporal_stabilization_enabled_;
    int stabilization_history_size_;
    double position_stabilization_threshold_;
    double angle_stabilization_threshold_;
    std::vector<std::vector<cv::Point2f>> waypoint_history_;
    std::vector<cv::Point2f> last_stable_waypoints_;

    // Encontra o centro de uma lane em uma altura específica
    cv::Point2f findLaneCenterAtHeight(const cv::Mat& lane_mask, int height);

    // Calcula a largura da lane em uma altura específica
    double calculateLaneWidthAtHeight(const cv::Mat& lane_mask, int height);

    // Valida se um waypoint está em uma posição válida
    bool isValidWaypoint(const cv::Point2f& waypoint);

    // Aplica filtro de média móvel para suavização
    cv::Point2f applyMovingAverage(const std::vector<cv::Point2f>& points, int center_idx, int window_size);

        // Estima o centro da pista quando apenas uma faixa é detectada
    std::vector<cv::Point2f> estimateRoadCenterFromSingleLane(const std::vector<cv::Point2f>& lane_centerline);
};

} // namespace mpc

#endif // WAYPOINT_EXTRACTOR_HPP
