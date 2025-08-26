#include "postprocess.h"
#include "drivable_visualization_config.h"
#include <algorithm>
#include <iostream>  // Include this header for printing
#include "utils.h"

cv::Rect get_rect(cv::Mat& img, float bbox[4]) {
    // CORREÇÃO: Melhorar a transformação de coordenadas do modelo (320x320) para a imagem real
    // para resolver problemas de proporção das áreas drivable

    float l, r, t, b;
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);

    if (r_h > r_w) {
        // A altura é o fator limitante - a imagem real é mais larga que o modelo
        l = bbox[0];
        r = bbox[2];
        t = bbox[1] - (kInputH - r_w * img.rows) / 2;
        b = bbox[3] - (kInputH - r_w * img.rows) / 2;

        // Aplicar escala uniforme baseada na largura
        l = l / r_w;
        r = r / r_w;
        t = t / r_w;
        b = b / r_w;
    } else {
        // A largura é o fator limitante - a imagem real é mais alta que o modelo
        l = bbox[0] - (kInputW - r_h * img.cols) / 2;
        r = bbox[2] - (kInputW - r_h * img.cols) / 2;
        t = bbox[1];
        b = bbox[3];

        // Aplicar escala uniforme baseada na altura
        l = l / r_h;
        r = r / r_h;
        t = t / r_h;
        b = b / r_h;
    }

    // Garantir que as coordenadas estejam dentro dos limites da imagem
    l = std::max(0.0f, l);
    t = std::max(0.0f, t);
    r = std::min(static_cast<float>(img.cols), r);
    b = std::min(static_cast<float>(img.rows), b);

    // Calcular dimensões finais
    int width = std::max(0, static_cast<int>(round(r - l)));
    int height = std::max(0, static_cast<int>(round(b - t)));

    // Garantir que o retângulo não exceda os limites da imagem
    width = std::min(width, img.cols - static_cast<int>(round(l)));
    height = std::min(height, img.rows - static_cast<int>(round(t)));

    return cv::Rect(static_cast<int>(round(l)), static_cast<int>(round(t)), width, height);
}

// Versão sobrecarregada para const float*
cv::Rect get_rect(cv::Mat& img, const float bbox[4]) {
    // CORREÇÃO: Melhorar a transformação de coordenadas do modelo (320x320) para a imagem real
    // para resolver problemas de proporção das áreas drivable

    float l, r, t, b;
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);

    if (r_h > r_w) {
        // A altura é o fator limitante - a imagem real é mais larga que o modelo
        l = bbox[0];
        r = bbox[2];
        t = bbox[1] - (kInputH - r_w * img.rows) / 2;
        b = bbox[3] - (kInputH - r_w * img.rows) / 2;

        // Aplicar escala uniforme baseada na largura
        l = l / r_w;
        r = r / r_w;
        t = t / r_w;
        b = b / r_w;
    } else {
        // A largura é o fator limitante - a imagem real é mais alta que o modelo
        l = bbox[0] - (kInputW - r_h * img.cols) / 2;
        r = bbox[2] - (kInputW - r_h * img.cols) / 2;
        t = bbox[1];
        b = bbox[3];

        // Aplicar escala uniforme baseada na altura
        l = l / r_h;
        r = r / r_h;
        t = t / r_h;
        b = b / r_h;
    }

    // Garantir que as coordenadas estejam dentro dos limites da imagem
    l = std::max(0.0f, l);
    t = std::max(0.0f, t);
    r = std::min(static_cast<float>(img.cols), r);
    b = std::min(static_cast<float>(img.rows), b);

    // Calcular dimensões finais
    int width = std::max(0, static_cast<int>(round(r - l)));
    int height = std::max(0, static_cast<int>(round(b - t)));

    // Garantir que o retângulo não exceda os limites da imagem
    width = std::min(width, img.cols - static_cast<int>(round(l)));
    height = std::min(height, img.rows - static_cast<int>(round(t)));

    return cv::Rect(static_cast<int>(round(l)), static_cast<int>(round(t)), width, height);
}

cv::Rect get_rect_adapt_landmark(cv::Mat& img, float bbox[4], float lmk[kNumberOfPoints * 3]) {
    // CORREÇÃO: Melhorar a transformação de coordenadas do modelo (320x320) para a imagem real
    // para resolver problemas de proporção das áreas drivable

    float l, r, t, b;
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);

    if (r_h > r_w) {
        // A altura é o fator limitante - a imagem real é mais larga que o modelo
        l = bbox[0] / r_w;
        r = bbox[2] / r_w;
        t = (bbox[1] - (kInputH - r_w * img.rows) / 2) / r_w;
        b = (bbox[3] - (kInputH - r_w * img.rows) / 2) / r_w;

        // Adaptar landmarks com escala uniforme baseada na largura
        for (int i = 0; i < kNumberOfPoints * 3; i += 3) {
            lmk[i] /= r_w;
            lmk[i + 1] = (lmk[i + 1] - (kInputH - r_w * img.rows) / 2) / r_w;
            // lmk[i + 2] permanece inalterado
        }
    } else {
        // A largura é o fator limitante - a imagem real é mais alta que o modelo
        l = (bbox[0] - (kInputW - r_h * img.cols) / 2) / r_h;
        r = (bbox[2] - (kInputW - r_h * img.cols) / 2) / r_h;
        t = bbox[1] / r_h;
        b = bbox[3] / r_h;

        // Adaptar landmarks com escala uniforme baseada na altura
        for (int i = 0; i < kNumberOfPoints * 3; i += 3) {
            lmk[i] = (lmk[i] - (kInputW - r_h * img.cols) / 2) / r_h;
            lmk[i + 1] /= r_h;
            // lmk[i + 2] permanece inalterado
        }
    }

    // Garantir que as coordenadas estejam dentro dos limites da imagem
    l = std::max(0.0f, l);
    t = std::max(0.0f, t);
    r = std::min(static_cast<float>(img.cols), r);
    b = std::min(static_cast<float>(img.rows), b);

    // Calcular dimensões finais
    int width = std::max(0, static_cast<int>(round(r - l)));
    int height = std::max(0, static_cast<int>(round(b - t)));

    // Garantir que o retângulo não exceda os limites da imagem
    width = std::min(width, img.cols - static_cast<int>(round(l)));
    height = std::min(height, img.rows - static_cast<int>(round(t)));

    return cv::Rect(static_cast<int>(round(l)), static_cast<int>(round(t)), width, height);
}

static float iou(float lbox[4], float rbox[4]) {
    float interBox[] = {
            (std::max)(lbox[0], rbox[0]),
            (std::min)(lbox[2], rbox[2]),
            (std::max)(lbox[1], rbox[1]),
            (std::min)(lbox[3], rbox[3]),
    };

    if (interBox[2] > interBox[3] || interBox[0] > interBox[1])
        return 0.0f;

    float interBoxS = (interBox[1] - interBox[0]) * (interBox[3] - interBox[2]);
    float unionBoxS = (lbox[2] - lbox[0]) * (lbox[3] - lbox[1]) + (rbox[2] - rbox[0]) * (rbox[3] - rbox[1]) - interBoxS;
    return interBoxS / unionBoxS;
}

static bool cmp(const Detection& a, const Detection& b) {
    if (a.conf == b.conf) {
        return a.bbox[0] < b.bbox[0];
    }
    return a.conf > b.conf;
}

void nms(std::vector<Detection>& res, float* output, float conf_thresh, float nms_thresh) {
    int det_size = sizeof(Detection) / sizeof(float);
    std::map<float, std::vector<Detection>> m;

    for (int i = 0; i < output[0]; i++) {
        if (output[1 + det_size * i + 4] <= conf_thresh || isnan(output[1 + det_size * i + 4]))
            continue;
        Detection det;
        memcpy(&det, &output[1 + det_size * i], det_size * sizeof(float));
        if (m.count(det.class_id) == 0)
            m.emplace(det.class_id, std::vector<Detection>());
        m[det.class_id].push_back(det);
    }
    for (auto it = m.begin(); it != m.end(); it++) {
        auto& dets = it->second;
        std::sort(dets.begin(), dets.end(), cmp);
        for (size_t m = 0; m < dets.size(); ++m) {
            auto& item = dets[m];
            res.push_back(item);
            for (size_t n = m + 1; n < dets.size(); ++n) {
                if (iou(item.bbox, dets[n].bbox) > nms_thresh) {
                    dets.erase(dets.begin() + n);
                    --n;
                }
            }
        }
    }
}

void batch_nms(std::vector<std::vector<Detection>>& res_batch, float* output, int batch_size, int output_size,
               float conf_thresh, float nms_thresh) {
    res_batch.resize(batch_size);
    for (int i = 0; i < batch_size; i++) {
        nms(res_batch[i], &output[i * output_size], conf_thresh, nms_thresh);
    }
}

void process_decode_ptr_host(std::vector<Detection>& res, const float* decode_ptr_host, int bbox_element, cv::Mat& img,
                             int count) {
    Detection det;
    for (int i = 0; i < count; i++) {
        int basic_pos = 1 + i * bbox_element;
        int keep_flag = decode_ptr_host[basic_pos + 6];
        if (keep_flag == 1) {
            det.bbox[0] = decode_ptr_host[basic_pos + 0];
            det.bbox[1] = decode_ptr_host[basic_pos + 1];
            det.bbox[2] = decode_ptr_host[basic_pos + 2];
            det.bbox[3] = decode_ptr_host[basic_pos + 3];
            det.conf = decode_ptr_host[basic_pos + 4];
            det.class_id = decode_ptr_host[basic_pos + 5];
            res.push_back(det);
        }
    }
}

void batch_process(std::vector<std::vector<Detection>>& res_batch, const float* decode_ptr_host, int batch_size,
                   int bbox_element, const std::vector<cv::Mat>& img_batch) {
    res_batch.resize(batch_size);
    int count = static_cast<int>(*decode_ptr_host);
    count = std::min(count, kMaxNumOutputBbox);
    for (int i = 0; i < batch_size; i++) {
        auto& img = const_cast<cv::Mat&>(img_batch[i]);
        process_decode_ptr_host(res_batch[i], &decode_ptr_host[i * count], bbox_element, img, count);
    }
}

void draw_bbox(std::vector<cv::Mat>& img_batch, std::vector<std::vector<Detection>>& res_batch) {
    for (size_t i = 0; i < img_batch.size(); i++) {
        auto& res = res_batch[i];
        cv::Mat img = img_batch[i];
        for (size_t j = 0; j < res.size(); j++) {
            cv::Rect r = get_rect(img, res[j].bbox);
            cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                        cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        }
    }
}

void draw_bbox_keypoints_line(std::vector<cv::Mat>& img_batch, std::vector<std::vector<Detection>>& res_batch) {
    const std::vector<std::pair<int, int>> skeleton_pairs = {
            {0, 1}, {0, 2},  {0, 5}, {0, 6},  {1, 2},   {1, 3},   {2, 4},   {5, 6},   {5, 7},  {5, 11},
            {6, 8}, {6, 12}, {7, 9}, {8, 10}, {11, 12}, {11, 13}, {12, 14}, {13, 15}, {14, 16}};

    for (size_t i = 0; i < img_batch.size(); i++) {
        auto& res = res_batch[i];
        cv::Mat img = img_batch[i];
        for (size_t j = 0; j < res.size(); j++) {
            cv::Rect r = get_rect_adapt_landmark(img, res[j].bbox, res[j].keypoints);
            cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                        cv::Scalar(0xFF, 0xFF, 0xFF), 2);

            for (int k = 0; k < kNumberOfPoints * 3; k += 3) {
                if (res[j].keypoints[k + 2] > 0.5) {
                    cv::circle(img, cv::Point((int)res[j].keypoints[k], (int)res[j].keypoints[k + 1]), 3,
                               cv::Scalar(0, 0x27, 0xC1), -1);
                }
            }

            for (const auto& bone : skeleton_pairs) {
                int kp1_idx = bone.first * 3;
                int kp2_idx = bone.second * 3;
                if (res[j].keypoints[kp1_idx + 2] > 0.5 && res[j].keypoints[kp2_idx + 2] > 0.5) {
                    cv::Point p1((int)res[j].keypoints[kp1_idx], (int)res[j].keypoints[kp1_idx + 1]);
                    cv::Point p2((int)res[j].keypoints[kp2_idx], (int)res[j].keypoints[kp2_idx + 1]);
                    cv::line(img, p1, p2, cv::Scalar(0, 0x27, 0xC1), 2);
                }
            }
        }
    }
}

cv::Mat scale_mask(cv::Mat mask, cv::Mat img) {
    // CORREÇÃO: A função anterior tinha problemas de proporção que causavam
    // distorção nas áreas drivable quando o objeto mudava de posição

    // Calcular as razões de escala entre o modelo (320x320) e a imagem real
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);

    // Determinar qual dimensão é o fator limitante
    float scale_factor;
    int x_offset, y_offset;

    if (r_h > r_w) {
        // A altura é o fator limitante - a imagem real é mais larga que o modelo
        scale_factor = r_w;
        x_offset = 0;
        y_offset = (kInputH - r_w * img.rows) / 2;
    } else {
        // A largura é o fator limitante - a imagem real é mais alta que o modelo
        scale_factor = r_h;
        x_offset = (kInputW - r_h * img.cols) / 2;
        y_offset = 0;
    }

    // Calcular a região da máscara que corresponde à imagem real
    int mask_width = static_cast<int>(img.cols * scale_factor);
    int mask_height = static_cast<int>(img.rows * scale_factor);

    // Garantir que as dimensões não excedam os limites da máscara
    mask_width = std::min(mask_width, mask.cols);
    mask_height = std::min(mask_height, mask.rows);

    // Calcular o offset para centralizar a região na máscara
    int mask_x = x_offset;
    int mask_y = y_offset;

    // Garantir que a região está dentro dos limites da máscara
    mask_x = std::max(0, std::min(mask_x, mask.cols - mask_width));
    mask_y = std::max(0, std::min(mask_y, mask.rows - mask_height));

    // Extrair a região relevante da máscara
    cv::Rect mask_roi(mask_x, mask_y, mask_width, mask_height);

    // Verificar se a região é válida
    if (mask_roi.x < 0 || mask_roi.y < 0 ||
        mask_roi.x + mask_roi.width > mask.cols ||
        mask_roi.y + mask_roi.height > mask.rows) {
        // Fallback: usar a máscara inteira se a região calculada for inválida
        cv::Mat res;
        cv::resize(mask, res, img.size());
        return res;
    }

    // Extrair e redimensionar a região da máscara para o tamanho da imagem
    cv::Mat mask_region = mask(mask_roi);
    cv::Mat res;
    cv::resize(mask_region, res, img.size());

    return res;
}

void draw_mask_bbox(cv::Mat& img, std::vector<Detection>& dets, std::vector<cv::Mat>& masks,
                    std::unordered_map<int, std::string>& labels_map) {
    static std::vector<uint32_t> colors = {0xFF3838, 0xFF9D97, 0xFF701F, 0xFFB21D, 0xCFD231, 0x48F90A, 0x92CC17,
                                           0x3DDB86, 0x1A9334, 0x00D4BB, 0x2C99A8, 0x00C2FF, 0x344593, 0x6473FF,
                                           0x0018EC, 0x8438FF, 0x520085, 0xCB38FF, 0xFF95C8, 0xFF37C7};

    for (size_t i = 0; i < dets.size(); i++) {
        cv::Mat img_mask = scale_mask(masks[i], img);
        auto color = colors[(int)dets[i].class_id % colors.size()];
        auto bgr = cv::Scalar(color & 0xFF, color >> 8 & 0xFF, color >> 16 & 0xFF);

        cv::Rect r = get_rect(img, dets[i].bbox);

        // Aplicar máscara
        for (int x = r.x; x < r.x + r.width; x++) {
            for (int y = r.y; y < r.y + r.height; y++) {
                if (x < 0 || x >= img.cols || y < 0 || y >= img.rows) continue;
                float val = img_mask.at<float>(y, x);
                if (val <= 0.5)
                    continue;
                img.at<cv::Vec3b>(y, x)[0] = img.at<cv::Vec3b>(y, x)[0] / 2 + bgr[0] / 2;
                img.at<cv::Vec3b>(y, x)[1] = img.at<cv::Vec3b>(y, x)[1] / 2 + bgr[1] / 2;
                img.at<cv::Vec3b>(y, x)[2] = img.at<cv::Vec3b>(y, x)[2] / 2 + bgr[2] / 2;
            }
        }

        // Só desenhar bbox se a configuração estiver habilitada
        #if SHOW_BBOX
        cv::rectangle(img, r, bgr, 2);

        // Get the size of the text
        cv::Size textSize =
                cv::getTextSize(labels_map[(int)dets[i].class_id] + " " + to_string_with_precision(dets[i].conf),
                                cv::FONT_HERSHEY_PLAIN, 1.2, 2, NULL);
        // Set the top left corner of the rectangle
        cv::Point topLeft(r.x, r.y - textSize.height);

        // Set the bottom right corner of the rectangle
        cv::Point bottomRight(r.x + textSize.width, r.y + textSize.height);

        // Set the thickness of the rectangle lines
        int lineThickness = 2;

        // Draw the rectangle on the image
        cv::rectangle(img, topLeft, bottomRight, bgr, -1);

        cv::putText(img, labels_map[(int)dets[i].class_id] + " " + to_string_with_precision(dets[i].conf),
                    cv::Point(r.x, r.y + 4), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar::all(0xFF), 2);
        #endif
    }
}

void draw_mask_only(cv::Mat& img, std::vector<Detection>& dets, std::vector<cv::Mat>& masks,
                    std::unordered_map<int, std::string>& labels_map) {
    static std::vector<uint32_t> colors = {0xFF3838, 0xFF9D97, 0xFF701F, 0xFFB21D, 0xCFD231, 0x48F90A, 0x92CC17,
                                           0x3DDB86, 0x1A9334, 0x00D4BB, 0x2C99A8, 0x00C2FF, 0x344593, 0x6473FF,
                                           0x0018EC, 0x8438FF, 0x520085, 0xCB38FF, 0xFF95C8, 0xFF37C7};

    // Separar detecções por tipo
    std::vector<Detection> lane_dets, drivable_dets;
    std::vector<cv::Mat> lane_masks, drivable_masks;

    for (size_t i = 0; i < dets.size(); i++) {
        auto it = labels_map.find(dets[i].class_id);
        if (it != labels_map.end()) {
            if (it->second == "lane") {
                lane_dets.push_back(dets[i]);
                lane_masks.push_back(masks[i]);
            } else if (it->second == "drivable") {
                drivable_dets.push_back(dets[i]);
                drivable_masks.push_back(masks[i]);
            }
        }
    }

    // ====== PROCESSAR LANES ======
    // Centro da imagem (assumindo que o carro está no centro)
    int center_x = img.cols / 2;
    int center_y = img.rows;

    // Encontrar as faixas mais próximas do carro
    std::vector<std::pair<float, int>> lane_distances;
    for (size_t i = 0; i < lane_dets.size(); i++) {
        cv::Mat img_mask = scale_mask(lane_masks[i], img);
        cv::Rect r = get_rect(img, lane_dets[i].bbox);
        float lane_center_x = r.x + r.width / 2.0f;
        float lane_center_y = r.y + r.height / 2.0f;
        float distance = std::sqrt(std::pow(lane_center_x - center_x, 2) + std::pow(lane_center_y - center_y, 2));
        lane_distances.push_back({distance, i});
    }
    std::sort(lane_distances.begin(), lane_distances.end());

    // Aplicar filtros similares aos da função draw_lane_lines
    std::vector<int> filtered_lane_indices;
    for (const auto& lane_info : lane_distances) {
        int lane_idx = lane_info.second;
        float distance = lane_info.first;

        // FILTRO 1: Distância máxima
        float max_distance_threshold = img.rows * ROI_END_Y_RATIO;  // Configurável via roi_config.h
        if (distance > max_distance_threshold) continue;

        // FILTRO 2: Posição relativa ao carro
        cv::Rect r = get_rect(img, lane_dets[lane_idx].bbox);
        float lane_center_x = r.x + r.width / 2.0f;
        float center_tolerance = img.cols * LANE_CENTER_TOLERANCE_RATIO_1;  // Configurável via roi_config.h
        if (std::abs(lane_center_x - center_x) > center_tolerance) {
            distance *= 1.5; // Penalizar faixas muito laterais
        }

        filtered_lane_indices.push_back(lane_idx);
    }

    // ====== DESENHAR LANES ======
    for (size_t i = 0; i < lane_dets.size(); i++) {
        cv::Mat img_mask = scale_mask(lane_masks[i], img);

        // Verificar se esta lane está sendo usada ativamente
        bool is_active_lane = false;
        for (int filtered_idx : filtered_lane_indices) {
            if (filtered_idx == i) {
                is_active_lane = true;
                break;
            }
        }

        // Usar cores diferentes baseadas no status
        cv::Scalar bgr;
        if (is_active_lane) {
            bgr = cv::Scalar(0, 255, 255); // Ciano brilhante para lanes ativas
        } else {
            bgr = cv::Scalar(0, 165, 255); // Laranja para lanes não ativas
        }

        cv::Rect r = get_rect(img, lane_dets[i].bbox);
        for (int x = r.x; x < r.x + r.width; x++) {
            for (int y = r.y; y < r.y + r.height; y++) {
                if (x < 0 || x >= img.cols || y < 0 || y >= img.rows) continue;
                float val = img_mask.at<float>(y, x);
                if (val <= 0.5)
                    continue;
                img.at<cv::Vec3b>(y, x)[0] = img.at<cv::Vec3b>(y, x)[0] / 2 + bgr[0] / 2;
                img.at<cv::Vec3b>(y, x)[1] = img.at<cv::Vec3b>(y, x)[1] / 2 + bgr[1] / 2;
                img.at<cv::Vec3b>(y, x)[2] = img.at<cv::Vec3b>(y, x)[2] / 2 + bgr[2] / 2;
            }
        }
    }

    // ====== PROCESSAR ÁREAS DRIVABLE BASEADO NO MODO ======
    if (DRIVABLE_DISPLAY_MODE == 0) {
        return; // Não exibir áreas drivable
    }

    // Calcular limites do ROI
    int roi_base_y = static_cast<int>(img.rows * ROI_BASE_Y_RATIO);
    int roi_top_y = static_cast<int>(img.rows * ROI_TOP_Y_RATIO);
    int roi_center_x = img.cols / 2;
    int roi_width = static_cast<int>(img.cols * ROI_LANE_TOLERANCE * 2);

    // Definir área do ROI
    cv::Rect roi_area(roi_center_x - roi_width/2, roi_top_y, roi_width, roi_base_y - roi_top_y);

    // ====== DESENHAR ÁREAS DRIVABLE ======
    for (size_t i = 0; i < drivable_dets.size(); i++) {
        cv::Mat img_mask = scale_mask(drivable_masks[i], img);
        cv::Rect drivable_rect = get_rect(img, drivable_dets[i].bbox);

        // Verificar se deve exibir esta área drivable
        bool should_display = false;
        cv::Scalar drivable_color;
        float alpha;
        std::string label_text;

        if (DRIVABLE_DISPLAY_MODE == 1) {
            // Modo 1: Apenas áreas drivable entre as lanes do ROI
            cv::Rect intersection = roi_area & drivable_rect;
            if (intersection.area() > MIN_INTERSECTION_AREA) {
                should_display = true;
                drivable_color = DRIVABLE_ROI_COLOR;
                alpha = DRIVABLE_ROI_OPACITY;
                label_text = "DRIVABLE NO ROI";
            }
        } else if (DRIVABLE_DISPLAY_MODE == 2) {
            // Modo 2: Todas as áreas drivable
            should_display = true;

            // Verificar se está no ROI para aplicar cor diferente
            cv::Rect intersection = roi_area & drivable_rect;
            if (intersection.area() > MIN_INTERSECTION_AREA) {
                drivable_color = DRIVABLE_ROI_COLOR;
                alpha = DRIVABLE_ROI_OPACITY;
                label_text = "DRIVABLE NO ROI";
            } else {
                drivable_color = DRIVABLE_OTHER_AREAS_COLOR;
                alpha = DRIVABLE_OTHER_OPACITY;
                label_text = "DRIVABLE";
            }
        }

        if (should_display) {
            // Aplicar máscara drivable com proporções corretas
            // CORREÇÃO: Usar toda a área da imagem para verificar a máscara redimensionada
            for (int y = 0; y < img.rows; y++) {
                for (int x = 0; x < img.cols; x++) {
                    // Verificar se o pixel está na máscara drivable redimensionada
                    float val = img_mask.at<float>(y, x);
                    if (val > 0.5) {
                        // Aplicar cor com transparência
                        img.at<cv::Vec3b>(y, x)[0] = img.at<cv::Vec3b>(y, x)[0] * (1 - alpha) + drivable_color[0] * alpha;
                        img.at<cv::Vec3b>(y, x)[1] = img.at<cv::Vec3b>(y, x)[1] * (1 - alpha) + drivable_color[1] * alpha;
                        img.at<cv::Vec3b>(y, x)[2] = img.at<cv::Vec3b>(y, x)[2] * (1 - alpha) + drivable_color[2] * alpha;
                    }
                }
            }

            // Adicionar label se habilitado
            if (SHOW_DRIVABLE_LABELS && !label_text.empty()) {
                cv::putText(img, label_text, cv::Point(drivable_rect.x, drivable_rect.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, drivable_color, 2);
            }
        }
    }

    // ====== DESENHAR ROI VISUAL (OPCIONAL) ======
    if (DRIVABLE_DISPLAY_MODE > 0) {
        // Desenhar borda do ROI para referência visual
        // cv::rectangle(img, roi_area, cv::Scalar(255, 0, 255), 2);  // DESATIVADO
        // cv::putText(img, "ROI", cv::Point(roi_area.x, roi_area.y - 10),  // DESATIVADO
        //             cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);  // DESATIVADO
    }
}

void draw_lane_lines(cv::Mat& img, std::vector<Detection>& dets, std::vector<cv::Mat>& masks,
                     std::unordered_map<int, std::string>& labels_map) {
    // Filtrar apenas detecções de "lane"
    // IMPORTANTE: A nomenclatura "Left Lane" e "Right Lane" é baseada na INCLINAÇÃO das faixas
    // - Faixas com inclinação positiva (da esquerda para direita) são "Right Lane" (Vermelho)
    // - Faixas com inclinação negativa (da direita para esquerda) são "Left Lane" (Azul)
    // - Para faixas quase verticais (inclinação < 0.1), usa posição X como fallback
    // Esta abordagem é mais robusta que apenas posição X, pois considera a direção da faixa

    std::vector<Detection> lane_dets;
    std::vector<cv::Mat> lane_masks;

    for (size_t i = 0; i < dets.size(); i++) {
        auto it = labels_map.find(dets[i].class_id);
        if (it != labels_map.end() && it->second == "lane") {
            lane_dets.push_back(dets[i]);
            lane_masks.push_back(masks[i]);
        }
    }

    if (lane_dets.empty()) return;

    // Centro da imagem (assumindo que o carro está no centro)
    int center_x = img.cols / 2;
    int center_y = img.rows;

    // Encontrar as faixas mais próximas do carro
    std::vector<std::pair<float, int>> lane_distances;

    for (size_t i = 0; i < lane_dets.size(); i++) {
        cv::Mat img_mask = scale_mask(lane_masks[i], img);
        cv::Rect r = get_rect(img, lane_dets[i].bbox);

        // Calcular a distância do centro da faixa ao centro do carro
        float lane_center_x = r.x + r.width / 2.0f;
        float lane_center_y = r.y + r.height / 2.0f;

        float distance = std::sqrt(std::pow(lane_center_x - center_x, 2) + std::pow(lane_center_y - center_y, 2));
        lane_distances.push_back({distance, i});
    }

    // LÓGICA DE PRIORIZAÇÃO: Proximidade da base da faixa ao centro da base do ROI
    // CRITÉRIO PRINCIPAL: Para faixas da mesma classificação (Left/Right), manter apenas a que tem
    // o círculo da base mais próximo do "ROI Base Center"
    // Definir ROI como o meio do frame (área central)
    int roi_start_y = img.rows * ROI_START_Y_RATIO;  // Configurável via roi_config.h
    int roi_end_y = img.rows * ROI_END_Y_RATIO;      // Configurável via roi_config.h
    int roi_base_center_x = img.cols / 2; // Centro da base do ROI
    int roi_base_y = roi_end_y; // Base do ROI

        // Desenhar retângulo do ROI para visualização
    #if SHOW_ROI_RECTANGLE
    cv::rectangle(img, cv::Point(0, roi_start_y), cv::Point(img.cols, roi_end_y),
                  cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    #endif

    // Adicionar texto "ROI" no topo
    #if SHOW_ROI_TEXT
    cv::putText(img, "ROI", cv::Point(10, roi_start_y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
    #endif

    // Calcular distância da base de cada lane ao centro da base do ROI
    std::vector<std::pair<float, int>> lane_base_distances;
    std::vector<float> lane_slopes;

    for (size_t i = 0; i < lane_dets.size(); i++) {
        cv::Mat img_mask = scale_mask(lane_masks[i], img);
        cv::Rect r = get_rect(img, lane_dets[i].bbox);

        // Calcular pontos da faixa para determinar inclinação e encontrar a base
        std::vector<cv::Point> temp_points;
        cv::Point base_point = cv::Point(r.x + r.width/2, r.y + r.height); // Ponto inicial
        int min_distance_to_base = std::abs(base_point.y - roi_base_y);

        for (int y = r.y; y < r.y + r.height; y += 10) {
            if (y < 0 || y >= img.rows) continue;

            int left_x = -1, right_x = -1;
            for (int x = r.x; x < r.x + r.width; x++) {
                if (x < 0 || x >= img.cols) continue;

                float val = img_mask.at<float>(y, x);
                if (val > 0.5) {
                    if (left_x == -1) left_x = x;
                    right_x = x;
                }
            }

            if (left_x != -1 && right_x != -1) {
                int center_x = (left_x + right_x) / 2;
                temp_points.push_back(cv::Point(center_x, y));

                // Atualizar ponto da base se este estiver mais próximo da base do ROI
                int distance_to_base = std::abs(y - roi_base_y);
                if (distance_to_base < min_distance_to_base) {
                    min_distance_to_base = distance_to_base;
                    base_point = cv::Point(center_x, y);
                }
            }
        }

        // Calcular inclinação da faixa
        float temp_slope = 0.0f;
        if (temp_points.size() >= 2) {
            std::vector<cv::Point> valid_points;
            for (const auto& point : temp_points) {
                if (point.x >= 0 && point.x < img.cols && point.y >= 0 && point.y < img.rows) {
                    valid_points.push_back(point);
                }
            }

            if (valid_points.size() >= 2) {
                float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
                int n = valid_points.size();

                for (const auto& point : valid_points) {
                    sum_x += point.x;
                    sum_y += point.y;
                    sum_xy += point.x * point.y;
                    sum_x2 += point.x * point.x;
                }

                float denominator = n * sum_x2 - sum_x * sum_x;
                if (std::abs(denominator) > 1e-6) {
                    temp_slope = (n * sum_xy - sum_x * sum_y) / denominator;
                }
            }
        }

        lane_slopes.push_back(temp_slope);

        // Calcular distância da base da lane ao centro da base do ROI
        float base_distance = std::sqrt(std::pow(base_point.x - roi_base_center_x, 2) +
                                      std::pow(base_point.y - roi_base_y, 2));
        lane_base_distances.push_back({base_distance, i});
    }

    // APLICAR FILTROS E PENALIZAÇÕES PRIMEIRO, DEPOIS ORDENAR POR PROXIMIDADE CORRIGIDA
    std::vector<std::pair<float, int>> corrected_lane_distances;

    // Aplicar filtros e calcular distâncias corrigidas para cada lane
    for (size_t i = 0; i < lane_base_distances.size(); i++) {
        int lane_idx = lane_base_distances[i].second;
        float base_distance = lane_base_distances[i].first;
        float slope = lane_slopes[lane_idx];

        bool should_include = true;

        // FILTRO 1: Distância máxima (evitar faixas muito distantes)
        float max_distance_threshold = img.rows * ROI_END_Y_RATIO; // Configurável via roi_config.h
        if (base_distance > max_distance_threshold) {
            should_include = false;
        }

        // FILTRO 2: Inclinação compatível (evitar faixas perpendiculares)
        if (std::abs(slope) > 2.0) {
            should_include = false;
        }

        // FILTRO 3: Posição relativa ao carro - aplicar penalização à distância
        cv::Rect r = get_rect(img, lane_dets[lane_idx].bbox);
        float lane_center_x = r.x + r.width / 2.0f;
        float corrected_distance = base_distance;

        float center_tolerance = img.cols * LANE_CENTER_TOLERANCE_RATIO_2;  // Configurável via roi_config.h
        if (std::abs(lane_center_x - center_x) > center_tolerance) {
            // Aplicar penalização à distância para reordenação
            corrected_distance *= 1.2;
        }

        if (should_include) {
            corrected_lane_distances.push_back({corrected_distance, lane_idx});
        }
    }

    // AGORA ORDENAR por proximidade corrigida (após penalizações)
    std::sort(corrected_lane_distances.begin(), corrected_lane_distances.end());

    // Extrair índices das lanes na ordem correta de prioridade
    std::vector<int> filtered_lane_indices;
    for (const auto& lane_info : corrected_lane_distances) {
        filtered_lane_indices.push_back(lane_info.second);
    }

    // FILTRO ADICIONAL: Resolver conflitos de classificação
    // Se temos duas faixas com a mesma classificação, priorizar a mais próxima do centro da base do ROI
    if (filtered_lane_indices.size() >= 2) {
        std::vector<std::pair<int, std::string>> lane_classifications;
        std::vector<float> base_distances;

        // Calcular classificação e distância da base para cada faixa filtrada
        for (int lane_idx : filtered_lane_indices) {
            cv::Mat img_mask = scale_mask(lane_masks[lane_idx], img);
            cv::Rect r = get_rect(img, lane_dets[lane_idx].bbox);

            // Calcular pontos da faixa para determinar inclinação
            std::vector<cv::Point> temp_points;
            for (int y = r.y; y < r.y + r.height; y += 10) {
                if (y < 0 || y >= img.rows) continue;

                int left_x = -1, right_x = -1;
                for (int x = r.x; x < r.x + r.width; x++) {
                    if (x < 0 || x >= img.cols) continue;

                    float val = img_mask.at<float>(y, x);
                    if (val > 0.5) {
                        if (left_x == -1) left_x = x;
                        right_x = x;
                    }
                }

                if (left_x != -1 && right_x != -1) {
                    int center_x = (left_x + right_x) / 2;
                    temp_points.push_back(cv::Point(center_x, y));
                }
            }

            // Calcular inclinação para classificação
            float temp_slope = 0.0f;
            if (temp_points.size() >= 2) {
                std::vector<cv::Point> valid_points;
                for (const auto& point : temp_points) {
                    if (point.x >= 0 && point.x < img.cols && point.y >= 0 && point.y < img.rows) {
                        valid_points.push_back(point);
                    }
                }

                if (valid_points.size() >= 2) {
                    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
                    int n = valid_points.size();

                    for (const auto& point : valid_points) {
                        sum_x += point.x;
                        sum_y += point.y;
                        sum_xy += point.x * point.y;
                        sum_x2 += point.x * point.x;
                    }

                    float denominator = n * sum_x2 - sum_x * sum_x;
                    if (std::abs(denominator) > 1e-6) {
                        temp_slope = (n * sum_xy - sum_x * sum_y) / denominator;
                    }
                }
            }

            // Classificar a faixa
            bool is_left_lane = false;
            if (std::abs(temp_slope) > 0.1) {
                is_left_lane = temp_slope < 0;
            } else {
                is_left_lane = (r.x + r.width / 2.0f) < img.cols / 2;
            }

            std::string classification = is_left_lane ? "Left" : "Right";
            lane_classifications.push_back({lane_idx, classification});

            // Calcular distância da base da faixa ao centro da base do ROI
            // Usar a mesma lógica da primeira parte: analisar a máscara para encontrar o ponto real da base
            cv::Point base_point = cv::Point(r.x + r.width/2, r.y + r.height); // Ponto inicial
            int min_distance_to_base = std::abs(base_point.y - roi_base_y);

            for (int y = r.y; y < r.y + r.height; y += 10) {
                if (y < 0 || y >= img.rows) continue;

                int left_x = -1, right_x = -1;
                for (int x = r.x; x < r.x + r.width; x++) {
                    if (x < 0 || x >= img.cols) continue;

                    float val = img_mask.at<float>(y, x);
                    if (val > 0.5) {
                        if (left_x == -1) left_x = x;
                        right_x = x;
                    }
                }

                if (left_x != -1 && right_x != -1) {
                    int center_x = (left_x + right_x) / 2;
                    // Atualizar ponto da base se este estiver mais próximo da base do ROI
                    int distance_to_base = std::abs(y - roi_base_y);
                    if (distance_to_base < min_distance_to_base) {
                        min_distance_to_base = distance_to_base;
                        base_point = cv::Point(center_x, y);
                    }
                }
            }

            // Calcular distância usando o ponto real da base encontrado na máscara
            float base_distance = std::sqrt(std::pow(base_point.x - roi_base_center_x, 2) +
                                          std::pow(base_point.y - roi_base_y, 2));
            base_distances.push_back(base_distance);
        }

        // Verificar se há conflitos de classificação
        std::map<std::string, std::vector<int>> classification_groups;
        for (size_t i = 0; i < lane_classifications.size(); i++) {
            std::string classification = lane_classifications[i].second;
            int lane_idx = lane_classifications[i].first;
            classification_groups[classification].push_back(lane_idx);
        }

        // Resolver conflitos: para cada classificação, manter apenas a faixa mais próxima da base do ROI
        // PRIORIDADE: Proximidade da base da faixa ao centro da base do ROI
        std::vector<int> final_filtered_indices;
        for (const auto& group : classification_groups) {
            std::string classification = group.first;
            const std::vector<int>& lane_indices = group.second;

            if (lane_indices.size() == 1) {
                // Apenas uma faixa desta classificação, incluir
                final_filtered_indices.push_back(lane_indices[0]);
            } else {
                // MÚLTIPLAS FAIXAS DA MESMA CLASSIFICAÇÃO: Selecionar a mais próxima do centro da base do ROI
                int best_lane_idx = lane_indices[0];
                // Encontrar a distância da base da faixa ao centro da base do ROI
                float best_base_distance = -1;
                for (size_t j = 0; j < lane_classifications.size(); j++) {
                    if (lane_classifications[j].first == best_lane_idx) {
                        best_base_distance = base_distances[j];
                        break;
                    }
                }

                // Comparar com todas as outras faixas da mesma classificação
                for (size_t i = 1; i < lane_indices.size(); i++) {
                    int current_lane_idx = lane_indices[i];
                    // Encontrar a distância da base da faixa atual ao centro da base do ROI
                    float current_base_distance = -1;
                    for (size_t j = 0; j < lane_classifications.size(); j++) {
                        if (lane_classifications[j].first == current_lane_idx) {
                            current_base_distance = base_distances[j];
                            break;
                        }
                    }

                    // Selecionar a faixa com a menor distância da base ao centro da base do ROI
                    if (current_base_distance >= 0 && (best_base_distance < 0 || current_base_distance < best_base_distance)) {
                        best_lane_idx = current_lane_idx;
                        best_base_distance = current_base_distance;
                    }
                }

                final_filtered_indices.push_back(best_lane_idx);

                // REMOVIDO: Informações de conflito para MÁXIMO FPS
                // REMOVIDO: "Conflict resolved: X Lane - kept closest to ROI base center"
                // REMOVIDO: "Kept Lane X (BaseDist=Ypx)"
            }
        }

        // Atualizar filtered_lane_indices com a resolução de conflitos
        filtered_lane_indices = final_filtered_indices;
    }

    // Se não encontrou faixas filtradas, usar as mais próximas (fallback)
    if (filtered_lane_indices.empty()) {
        for (size_t i = 0; i < std::min(2UL, lane_distances.size()); i++) {
            filtered_lane_indices.push_back(lane_distances[i].second);
        }
    }

    // REMOVIDO: Todas as informações pesadas de debug para MÁXIMO FPS
    // REMOVIDO: "Total Lanes: X | Filtered: Y"
    // REMOVIDO: "Lane x: X=... BaseDist=... Priority=... [ACTIVE/FILTERED]"

        // REMOVIDO: Todas as informações pesadas de overlay para MÁXIMO FPS
    // REMOVIDO: Resolução de conflitos
    // REMOVIDO: Critérios de seleção
    // REMOVIDO: Informações de lanes ativas

    // PRIMEIRO: Desenhar labels para TODAS as lanes detectadas
    for (size_t i = 0; i < lane_dets.size(); i++) {
        cv::Mat img_mask = scale_mask(lane_masks[i], img);
        cv::Rect r = get_rect(img, lane_dets[i].bbox);

        // Calcular o centro da faixa
        float lane_center_x = r.x + r.width / 2.0f;

        // Encontrar os pontos da linha da faixa
        std::vector<cv::Point> lane_points;
        for (int y = r.y; y < r.y + r.height; y += 10) {
            if (y < 0 || y >= img.rows) continue;

            int left_x = -1, right_x = -1;
            for (int x = r.x; x < r.x + r.width; x++) {
                if (x < 0 || x >= img.cols) continue;

                float val = img_mask.at<float>(y, x);
                if (val > 0.5) {
                    if (left_x == -1) left_x = x;
                    right_x = x;
                }
            }

            if (left_x != -1 && right_x != -1) {
                int center_x = (left_x + right_x) / 2;
                lane_points.push_back(cv::Point(center_x, y));
            }
        }

        // Calcular inclinação para determinar Left/Right
        float lane_slope = 0.0f;
        if (lane_points.size() >= 2) {
            std::vector<cv::Point> valid_points;
            for (const auto& point : lane_points) {
                if (point.x >= 0 && point.x < img.cols && point.y >= 0 && point.y < img.rows) {
                    valid_points.push_back(point);
                }
            }

            if (valid_points.size() >= 2) {
                float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
                int n = valid_points.size();

                for (const auto& point : valid_points) {
                    sum_x += point.x;
                    sum_y += point.y;
                    sum_xy += point.x * point.y;
                    sum_x2 += point.x * point.x;
                }

                float denominator = n * sum_x2 - sum_x * sum_x;
                if (std::abs(denominator) > 1e-6) {
                    lane_slope = (n * sum_xy - sum_x * sum_y) / denominator;
                }
            }
        }

        // Determinar se é Left ou Right Lane
        bool is_left_lane = false;
        if (std::abs(lane_slope) > 0.1) {
            is_left_lane = lane_slope < 0;
        } else {
            is_left_lane = lane_center_x < img.cols / 2;
        }

        // Verificar se esta lane está ativa (filtrada)
        bool is_active_lane = false;
        for (int filtered_idx : filtered_lane_indices) {
            if (filtered_idx == i) {
                is_active_lane = true;
                break;
            }
        }

        // Desenhar linha da lane se há pontos suficientes
        if (lane_points.size() > 1) {
            // Suavizar a linha
            std::vector<cv::Point> smoothed_points;
            if (lane_points.size() >= 3) {
                for (size_t j = 1; j < lane_points.size() - 1; j++) {
                    int avg_x = (lane_points[j-1].x + lane_points[j].x + lane_points[j+1].x) / 3;
                    int avg_y = (lane_points[j-1].y + lane_points[j].y + lane_points[j+1].y) / 3;
                    smoothed_points.push_back(cv::Point(avg_x, avg_y));
                }
            } else {
                smoothed_points = lane_points;
            }

            // Cor e espessura baseada no status
            cv::Scalar draw_color;
            int line_thickness;
            if (is_active_lane) {
                draw_color = cv::Scalar(0, 255, 255); // Ciano para lanes ativas
                line_thickness = 5;
            } else {
                draw_color = cv::Scalar(0, 165, 255); // Laranja para lanes não ativas
                line_thickness = 3;
            }

            // Desenhar linha
            for (size_t j = 1; j < smoothed_points.size(); j++) {
                cv::line(img, smoothed_points[j-1], smoothed_points[j], draw_color, line_thickness, cv::LINE_AA);
            }

            // Retângulos de destaque removidos para evitar poluição visual

            // EXIBIR LABELS DAS LANES USADAS (APENAS ESSENCIAIS)
            #if SHOW_LANE_LABELS
            if (is_active_lane) {
                std::string lane_label = is_left_lane ? "Left Lane Used" : "Right Lane Used";
                cv::Scalar label_color = cv::Scalar(0, 255, 255); // Ciano para ativas

                // Posição do label no meio da linha
                if (!smoothed_points.empty()) {
                    cv::Point label_pos = smoothed_points[smoothed_points.size()/2];
                    cv::putText(img, lane_label, label_pos, cv::FONT_HERSHEY_SIMPLEX,
                               0.6, label_color, 2);
                }
            }
            #endif
        }
    }

        // Desenhar o centro da base do ROI
    #if SHOW_ROI_BASE_CENTER
    cv::Point roi_base_center(roi_base_center_x, roi_base_y);
    cv::circle(img, roi_base_center, 10, cv::Scalar(255, 0, 255), -1, cv::LINE_AA); // Magenta sólido
    cv::circle(img, roi_base_center, 12, cv::Scalar(255, 255, 255), 2, cv::LINE_AA); // Contorno branco

    // Adicionar texto "ROI Base Center"
    cv::putText(img, "ROI Base Center", cv::Point(roi_base_center.x + 15, roi_base_center.y - 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2);
    #endif



    // Círculos vermelhos na base de cada lane detectada dentro do ROI
    for (size_t i = 0; i < lane_dets.size(); i++) {
        cv::Mat img_mask = scale_mask(lane_masks[i], img);
        cv::Rect r = get_rect(img, lane_dets[i].bbox);

        // Encontrar pontos da lane dentro do ROI
        std::vector<cv::Point> lane_roi_points;
        for (int y = roi_start_y; y <= roi_end_y; y += 10) {
            if (y < 0 || y >= img.rows) continue;

            int left_x = -1, right_x = -1;
            for (int x = r.x; x < r.x + r.width; x++) {
                if (x < 0 || x >= img.cols) continue;

                float val = img_mask.at<float>(y, x);
                if (val > 0.5) {
                    if (left_x == -1) left_x = x;
                    right_x = x;
                }
            }

            if (left_x != -1 && right_x != -1) {
                int center_x = (left_x + right_x) / 2;
                lane_roi_points.push_back(cv::Point(center_x, y));
            }
        }

        // Se encontrou pontos da lane dentro do ROI, desenhar círculo na base
        #if SHOW_LANE_BASE_POINTS
        if (!lane_roi_points.empty()) {
            // Encontrar o ponto mais próximo da base do ROI
            cv::Point base_point = lane_roi_points[0];
            int min_distance_to_base = std::abs(base_point.y - roi_base_y);

            for (const auto& point : lane_roi_points) {
                int distance_to_base = std::abs(point.y - roi_base_y);
                if (distance_to_base < min_distance_to_base) {
                    min_distance_to_base = distance_to_base;
                    base_point = point;
                }
            }

            // Desenhar círculo vermelho na base da lane
            cv::circle(img, base_point, 6, cv::Scalar(0, 0, 255), -1, cv::LINE_AA); // Vermelho sólido
            cv::circle(img, base_point, 8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA); // Contorno branco

            // Adicionar texto "Lane x Base" se habilitado
            #if SHOW_DEBUG_INFO
            std::string base_text = "Lane " + std::to_string(i) + " Base";
            cv::putText(img, base_text, cv::Point(base_point.x + 15, base_point.y - 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
            #endif
        }
        #endif

        // Desenhar círculo no topo da lane (mais próximo do topo do ROI)
        #if SHOW_LANE_TOP_POINTS
        if (!lane_roi_points.empty()) {
            // Encontrar o ponto mais próximo do topo do ROI
            cv::Point top_point = lane_roi_points[0];
            int min_distance_to_top = std::abs(top_point.y - roi_start_y);

            for (const auto& point : lane_roi_points) {
                int distance_to_top = std::abs(point.y - roi_start_y);
                if (distance_to_top < min_distance_to_top) {
                    min_distance_to_top = distance_to_top;
                    top_point = point;
                }
            }

            // Desenhar círculo ciano no topo da lane
            cv::circle(img, top_point, LANE_TOP_POINT_RADIUS, LANE_TOP_POINT_COLOR, -1, cv::LINE_AA); // Ciano sólido
            cv::circle(img, top_point, LANE_TOP_POINT_RADIUS + 2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA); // Contorno branco

            // Adicionar texto "Lane x Top" se habilitado
            #if SHOW_DEBUG_INFO
            std::string top_text = "Lane " + std::to_string(i) + " Top";
            cv::putText(img, top_text, cv::Point(top_point.x + 15, top_point.y + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, LANE_TOP_POINT_COLOR, 1);
            #endif
        }
        #endif
    }

    // SEGUNDO: Coletar pontos das lanes ativas para cálculo do centro
    std::vector<std::vector<cv::Point>> lane_points_array;

    // Processar apenas lanes ativas (filtradas) para cálculo do centro
    for (int lane_idx : filtered_lane_indices) {
        cv::Mat img_mask = scale_mask(lane_masks[lane_idx], img);
        cv::Rect r = get_rect(img, lane_dets[lane_idx].bbox);

        // Coletar pontos da linha da faixa ativa
        std::vector<cv::Point> lane_points;
        for (int y = r.y; y < r.y + r.height; y += 10) {
            if (y < 0 || y >= img.rows) continue;

            int left_x = -1, right_x = -1;
            for (int x = r.x; x < r.x + r.width; x++) {
                if (x < 0 || x >= img.cols) continue;

                float val = img_mask.at<float>(y, x);
                if (val > 0.5) {
                    if (left_x == -1) left_x = x;
                    right_x = x;
                }
            }

            if (left_x != -1 && right_x != -1) {
                int center_x = (left_x + right_x) / 2;
                lane_points.push_back(cv::Point(center_x, y));
            }
        }

        if (!lane_points.empty()) {
            lane_points_array.push_back(lane_points);
        }
    }

    // Calcular e desenhar o ponto médio entre as linhas das faixas
    if (lane_points_array.size() >= 2) {
        // Definir ROI como o meio do frame (área central)
        int roi_start_y = img.rows * ROI_START_Y_RATIO;  // Configurável via roi_config.h
        int roi_end_y = img.rows * ROI_END_Y_RATIO;      // Configurável via roi_config.h

                // Desenhar retângulo do ROI para visualização
        #if SHOW_ROI_RECTANGLE
        cv::rectangle(img, cv::Point(0, roi_start_y), cv::Point(img.cols, roi_end_y),
                      cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
        #endif

        // Adicionar texto "ROI (Two Lanes)"
        #if SHOW_ROI_TEXT
        cv::putText(img, "ROI (Two Lanes)", cv::Point(10, roi_start_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
        #endif

        // Encontrar pontos das duas faixas na região ROI
        std::vector<cv::Point> left_lane_roi, right_lane_roi;

        // Determinar qual faixa é esquerda e qual é direita baseado na posição real
        bool first_is_left = false;
        if (!lane_points_array[0].empty() && !lane_points_array[1].empty()) {
            // Calcular centro médio de cada faixa para determinar posição
            float first_lane_center_x = 0, second_lane_center_x = 0;
            int first_count = 0, second_count = 0;

            for (const auto& point : lane_points_array[0]) {
                if (point.y >= roi_start_y && point.y <= roi_end_y) {
                    first_lane_center_x += point.x;
                    first_count++;
                }
            }

            for (const auto& point : lane_points_array[1]) {
                if (point.y >= roi_start_y && point.y <= roi_end_y) {
                    second_lane_center_x += point.x;
                    second_count++;
                }
            }

            if (first_count > 0 && second_count > 0) {
                first_lane_center_x /= first_count;
                second_lane_center_x /= second_count;
                first_is_left = first_lane_center_x < second_lane_center_x;
            }
        }

        // Filtrar pontos da faixa esquerda na ROI
        if (first_is_left) {
            for (const auto& point : lane_points_array[0]) {
                if (point.y >= roi_start_y && point.y <= roi_end_y) {
                    left_lane_roi.push_back(point);
                }
            }
            for (const auto& point : lane_points_array[1]) {
                if (point.y >= roi_start_y && point.y <= roi_end_y) {
                    right_lane_roi.push_back(point);
                }
            }
        } else {
            for (const auto& point : lane_points_array[0]) {
                if (point.y >= roi_start_y && point.y <= roi_end_y) {
                    right_lane_roi.push_back(point);
                }
            }
            for (const auto& point : lane_points_array[1]) {
                if (point.y >= roi_start_y && point.y <= roi_end_y) {
                    left_lane_roi.push_back(point);
                }
            }
        }

        // Calcular pontos médios para cada linha na ROI
        if (!left_lane_roi.empty() && !right_lane_roi.empty()) {
            // Calcular centro da faixa esquerda na ROI
            cv::Point left_center(0, 0);
            for (const auto& point : left_lane_roi) {
                left_center.x += point.x;
                left_center.y += point.y;
            }
            left_center.x /= left_lane_roi.size();
            left_center.y /= left_lane_roi.size();

            // Calcular centro da faixa direita na ROI
            cv::Point right_center(0, 0);
            for (const auto& point : right_lane_roi) {
                right_center.x += point.x;
                right_center.y += point.y;
            }
            right_center.x /= right_lane_roi.size();
            right_center.y /= right_lane_roi.size();

            // Calcular ponto médio entre as duas faixas
            cv::Point center_point(
                (left_center.x + right_center.x) / 2,
                (left_center.y + right_center.y) / 2
            );

                        // Desenhar o ponto médio como um círculo verde
            #if SHOW_CENTER_POINT
            cv::circle(img, center_point, 8, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
            cv::circle(img, center_point, 10, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            #endif

            // Adicionar texto "Center" para o centro da pista
            #if SHOW_CENTER_TEXT
            cv::putText(img, "Center", cv::Point(center_point.x + 15, center_point.y - 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            #endif

            // Adicionar linha de referência do centro
            #if SHOW_CENTER_REFERENCE_LINE
            cv::line(img, cv::Point(center_point.x, center_point.y - 20),
                     cv::Point(center_point.x, center_point.y + 20),
                     cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            #endif

            // Adicionar linhas conectando o centro às faixas se habilitado
            #if SHOW_CENTER_TO_LANES_CONNECTION
            if (!left_lane_roi.empty() && !right_lane_roi.empty()) {
                // Linha do centro à faixa esquerda
                cv::line(img, center_point, left_center, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                // Linha do centro à faixa direita
                cv::line(img, center_point, right_center, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
            #endif
        }
    } else if (lane_points_array.size() == 1) {
        // Caso apenas uma faixa seja encontrada, estimar o centro baseado nas medidas físicas
        // Medidas físicas: distância entre faixas = 24cm, largura do carro = 21cm
        // Assumindo que a câmera está centralizada no carro

        int roi_start_y = img.rows * ROI_START_Y_RATIO;  // Configurável via roi_config.h
        int roi_end_y = img.rows * ROI_END_Y_RATIO;      // Configurável via roi_config.h

                // Desenhar retângulo do ROI para visualização
        #if SHOW_ROI_RECTANGLE
        cv::rectangle(img, cv::Point(0, roi_start_y), cv::Point(img.cols, roi_end_y),
                      cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
        #endif

        // Adicionar texto "ROI (Single Lane)"
        #if SHOW_ROI_TEXT
        cv::putText(img, "ROI (Single Lane)", cv::Point(10, roi_start_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
        #endif

        // Desenhar círculo na base do ROI
        #if SHOW_ROI_BASE_CENTER
        cv::Point roi_base_center(img.cols / 2, roi_end_y);
        cv::circle(img, roi_base_center, 8, cv::Scalar(255, 0, 255), -1, cv::LINE_AA);
        cv::circle(img, roi_base_center, 10, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

        // Adicionar texto "ROI Base Center"
        cv::putText(img, "ROI Base Center", cv::Point(roi_base_center.x + 15, roi_base_center.y - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2);
        #endif

        // Filtrar pontos da faixa encontrada na ROI
        std::vector<cv::Point> single_lane_roi;
        for (const auto& point : lane_points_array[0]) {
            if (point.y >= roi_start_y && point.y <= roi_end_y) {
                single_lane_roi.push_back(point);
            }
        }

        if (!single_lane_roi.empty()) {
            // Calcular centro da faixa encontrada
            cv::Point lane_center(0, 0);
            for (const auto& point : single_lane_roi) {
                lane_center.x += point.x;
                lane_center.y += point.y;
            }
            lane_center.x /= single_lane_roi.size();
            lane_center.y /= single_lane_roi.size();

            // DEBUG: Desenhar círculo vermelho na base da lane
            // Encontrar o ponto mais próximo da base do ROI para esta lane
            cv::Point lane_base_point = single_lane_roi[0]; // Ponto inicial
            int min_distance_to_base = std::abs(lane_base_point.y - roi_base_y);

            for (const auto& point : single_lane_roi) {
                int distance_to_base = std::abs(point.y - roi_base_y);
                if (distance_to_base < min_distance_to_base) {
                    min_distance_to_base = distance_to_base;
                    lane_base_point = point;
                }
            }

            // REMOVIDO: Todos os elementos visuais pesados para MÁXIMO FPS
            // REMOVIDO: cv::circle() para base da lane
            // REMOVIDO: cv::putText() para identificação
            // REMOVIDO: cv::line() para conexão

            // Calcular inclinação da faixa para determinar se é esquerda ou direita
            float single_lane_slope = 0.0f;
            if (single_lane_roi.size() >= 2) {
                // Usar regressão linear para calcular a inclinação
                float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
                int n = single_lane_roi.size();

                for (const auto& point : single_lane_roi) {
                    sum_x += point.x;
                    sum_y += point.y;
                    sum_xy += point.x * point.y;
                    sum_x2 += point.x * point.x;
                }

                float denominator = n * sum_x2 - sum_x * sum_x;
                if (std::abs(denominator) > 1e-6) {
                    single_lane_slope = (n * sum_xy - sum_x * sum_y) / denominator;
                }
            }

            // Determinar se é faixa esquerda ou direita baseado na inclinação
            bool is_left_lane = false;
            if (std::abs(single_lane_slope) > 0.1) {
                // Para uma faixa que vai da esquerda para direita (inclinação positiva), é "Right Lane"
                // Para uma faixa que vai da direita para esquerda (inclinação negativa), é "Left Lane"
                is_left_lane = single_lane_slope < 0;
            } else {
                // Fallback para posição X se inclinação for muito pequena
                is_left_lane = lane_center.x < img.cols / 2;
            }

            // Calcular o centro estimado baseado na faixa encontrada
            cv::Point estimated_center;

            if (is_left_lane) {
                // Se é faixa esquerda, o centro está a 12cm (metade de 24cm) à direita
                // Assumindo que a largura da imagem representa aproximadamente 45cm (24cm + 21cm)
                float pixels_per_cm = img.cols / 45.0f;
                float offset_pixels = 12.0f * pixels_per_cm;
                estimated_center.x = lane_center.x + offset_pixels;
                estimated_center.y = lane_center.y;

                // REMOVIDO: Texto de estimativa para MÁXIMO FPS
            } else {
                // Se é faixa direita, o centro está a 12cm à esquerda
                float pixels_per_cm = img.cols / 45.0f;
                float offset_pixels = 12.0f * pixels_per_cm;
                estimated_center.x = lane_center.x - offset_pixels;
                estimated_center.y = lane_center.y;

                // REMOVIDO: Texto de estimativa para MÁXIMO FPS
            }

            // REMOVIDO: Textos de debug para MÁXIMO FPS
            // REMOVIDO: Informações de slope e filtragem

            // Garantir que o centro estimado esteja dentro dos limites da imagem
            estimated_center.x = std::max(0, std::min(estimated_center.x, img.cols - 1));
            estimated_center.y = std::max(0, std::min(estimated_center.y, img.rows - 1));

                        // Desenhar o centro estimado
            #if SHOW_ESTIMATED_CENTER
            cv::circle(img, estimated_center, 8, cv::Scalar(0, 255, 255), -1, cv::LINE_AA); // Ciano sólido
            cv::circle(img, estimated_center, 10, cv::Scalar(0, 255, 255), 2, cv::LINE_AA); // Contorno ciano
            #endif

            // Adicionar texto "Est. Center"
            #if SHOW_ESTIMATED_TEXT
            cv::putText(img, "Est. Center", cv::Point(estimated_center.x + 15, estimated_center.y - 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
            #endif

            // Adicionar linha de referência do centro estimado
            #if SHOW_ESTIMATED_REFERENCE_LINE
            cv::line(img, cv::Point(estimated_center.x, estimated_center.y - 20),
                     cv::Point(estimated_center.x, estimated_center.y + 20),
                     cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            #endif

            // Adicionar linha conectando o centro estimado à faixa se habilitado
            #if SHOW_CENTER_TO_LANES_CONNECTION
            cv::line(img, estimated_center, lane_center, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
            #endif

            // Informações de coordenadas e medidas físicas removidas para clareza da interface
        }
    } else {
        // Caso nenhuma faixa seja encontrada
        int roi_start_y = img.rows * ROI_START_Y_RATIO;  // Configurável via roi_config.h
        int roi_end_y = img.rows * ROI_END_Y_RATIO;      // Configurável via roi_config.h

                // Desenhar retângulo do ROI para visualização
        #if SHOW_ROI_RECTANGLE
        cv::rectangle(img, cv::Point(0, roi_start_y), cv::Point(img.cols, roi_end_y),
                      cv::Scalar(128, 128, 128), 2, cv::LINE_AA);
        #endif

        // Adicionar texto "ROI (No Lanes)"
        #if SHOW_ROI_TEXT
        cv::putText(img, "ROI (Single Lane)", cv::Point(10, roi_start_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(128, 128, 128), 2);
        #endif

        // Desenhar círculo na base do ROI
        #if SHOW_ROI_BASE_CENTER
        cv::Point roi_base_center(img.cols / 2, roi_end_y);
        cv::circle(img, roi_base_center, 8, cv::Scalar(128, 128, 128), -1, cv::LINE_AA);
        cv::circle(img, roi_base_center, 10, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

        // Adicionar texto "ROI Base Center"
        cv::putText(img, "ROI Base Center", cv::Point(roi_base_center.x + 15, roi_base_center.y - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 128, 128), 2);
        #endif

        // Desenhar círculo no topo do ROI
        #if SHOW_ROI_TOP_CENTER
        cv::Point roi_top_center(img.cols / 2, roi_start_y);
        cv::circle(img, roi_top_center, ROI_TOP_CENTER_RADIUS, cv::Scalar(128, 128, 128), -1, cv::LINE_AA); // Cinza sólido
        cv::circle(img, roi_top_center, ROI_TOP_CENTER_RADIUS + 2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA); // Contorno branco

        // Adicionar texto "ROI Top Center"
        cv::putText(img, "ROI Top Center", cv::Point(roi_top_center.x + 15, roi_top_center.y + 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 128, 128), 2);
        #endif

        // Adicionar mensagem de aviso
        cv::putText(img, "No lanes detected", cv::Point(img.cols/2 - 100, img.rows/2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

        // Informações de medidas físicas removidas para clareza da interface
    }
}

void process_decode_ptr_host_obb(std::vector<Detection>& res, const float* decode_ptr_host, int bbox_element,
                                 cv::Mat& img, int count) {
    Detection det;
    for (int i = 0; i < count; i++) {
        int basic_pos = 1 + i * bbox_element;
        int keep_flag = decode_ptr_host[basic_pos + 6];
        if (keep_flag == 1) {
            det.bbox[0] = decode_ptr_host[basic_pos + 0];
            det.bbox[1] = decode_ptr_host[basic_pos + 1];
            det.bbox[2] = decode_ptr_host[basic_pos + 2];
            det.bbox[3] = decode_ptr_host[basic_pos + 3];
            det.conf = decode_ptr_host[basic_pos + 4];
            det.class_id = decode_ptr_host[basic_pos + 5];
            det.angle = decode_ptr_host[basic_pos + 7];
            res.push_back(det);
        }
    }
}

void batch_process_obb(std::vector<std::vector<Detection>>& res_batch, const float* decode_ptr_host, int batch_size,
                       int bbox_element, const std::vector<cv::Mat>& img_batch) {
    res_batch.resize(batch_size);
    int count = static_cast<int>(*decode_ptr_host);
    count = std::min(count, kMaxNumOutputBbox);
    for (int i = 0; i < batch_size; i++) {
        auto& img = const_cast<cv::Mat&>(img_batch[i]);
        process_decode_ptr_host_obb(res_batch[i], &decode_ptr_host[i * count], bbox_element, img, count);
    }
}

std::tuple<float, float, float> convariance_matrix(Detection res) {
    float w = res.bbox[2];
    float h = res.bbox[3];

    float a = w * w / 12.0;
    float b = h * h / 12.0;
    float c = res.angle;

    float cos_r = std::cos(c);
    float sin_r = std::sin(c);

    float cos_r2 = cos_r * cos_r;
    float sin_r2 = sin_r * sin_r;

    float a_val = a * cos_r2 + b * sin_r2;
    float b_val = a * sin_r2 + b * cos_r2;
    float c_val = (a - b) * cos_r * sin_r;

    return std::make_tuple(a_val, b_val, c_val);
}

static float probiou(const Detection& res1, const Detection& res2, float eps = 1e-7) {
    // Calculate the prob iou between oriented bounding boxes, https://arxiv.org/pdf/2106.06072v1.pdf.
    float a1, b1, c1, a2, b2, c2;
    std::tuple<float, float, float> matrix1 = {a1, b1, c1};
    std::tuple<float, float, float> matrix2 = {a2, b2, c2};
    matrix1 = convariance_matrix(res1);
    matrix2 = convariance_matrix(res2);
    a1 = std::get<0>(matrix1);
    b1 = std::get<1>(matrix1);
    c1 = std::get<2>(matrix1);
    a2 = std::get<0>(matrix2);
    b2 = std::get<1>(matrix2);
    c2 = std::get<2>(matrix2);

    float x1 = res1.bbox[0], y1 = res1.bbox[1];
    float x2 = res2.bbox[0], y2 = res2.bbox[1];

    float t1 = ((a1 + a2) * std::pow(y1 - y2, 2) + (b1 + b2) * std::pow(x1 - x2, 2)) /
               ((a1 + a2) * (b1 + b2) - std::pow(c1 + c2, 2) + eps);
    float t2 = ((c1 + c2) * (x2 - x1) * (y1 - y2)) / ((a1 + a2) * (b1 + b2) - std::pow(c1 + c2, 2) + eps);
    float t3 = std::log(
            ((a1 + a2) * (b1 + b2) - std::pow(c1 + c2, 2)) /
                    (4 * std::sqrt(std::max(a1 * b1 - c1 * c1, 0.0f)) * std::sqrt(std::max(a2 * b2 - c2 * c2, 0.0f)) +
                     eps) +
            eps);

    float bd = 0.25f * t1 + 0.5f * t2 + 0.5f * t3;
    bd = std::max(std::min(bd, 100.0f), eps);
    float hd = std::sqrt(1.0 - std::exp(-bd) + eps);

    return 1 - hd;
}

void nms_obb(std::vector<Detection>& res, float* output, float conf_thresh, float nms_thresh) {
    int det_size = sizeof(Detection) / sizeof(float);
    std::map<float, std::vector<Detection>> m;

    for (int i = 0; i < output[0]; i++) {

        if (output[1 + det_size * i + 4] <= conf_thresh)
            continue;
        Detection det;
        memcpy(&det, &output[1 + det_size * i], det_size * sizeof(float));
        if (m.count(det.class_id) == 0)
            m.emplace(det.class_id, std::vector<Detection>());
        m[det.class_id].push_back(det);
    }
    for (auto it = m.begin(); it != m.end(); it++) {
        auto& dets = it->second;
        std::sort(dets.begin(), dets.end(), cmp);
        for (size_t m = 0; m < dets.size(); ++m) {
            auto& item = dets[m];
            res.push_back(item);
            for (size_t n = m + 1; n < dets.size(); ++n) {
                if (probiou(item, dets[n]) >= nms_thresh) {
                    dets.erase(dets.begin() + n);
                    --n;
                }
            }
        }
    }
}

void batch_nms_obb(std::vector<std::vector<Detection>>& res_batch, float* output, int batch_size, int output_size,
                   float conf_thresh, float nms_thresh) {
    res_batch.resize(batch_size);
    for (int i = 0; i < batch_size; i++) {
        nms_obb(res_batch[i], &output[i * output_size], conf_thresh, nms_thresh);
    }
}

static std::vector<cv::Point> get_corner(cv::Mat& img, const Detection& box) {
    float cos_value, sin_value;

    // Calculate center point and width/height
    float x1 = box.bbox[0];
    float y1 = box.bbox[1];
    float w = box.bbox[2];
    float h = box.bbox[3];
    float angle = box.angle * 180.0f / CV_PI;  // Convert radians to degrees

    // Print original angle
    std::cout << "Original angle: " << angle << std::endl;

    // Swap width and height if height is greater than or equal to width
    if (h >= w) {
        std::swap(w, h);
        angle = fmod(angle + 90.0f, 180.0f);  // Adjust angle to be within [0, 180)
    }

    // Ensure the angle is between 0 and 180 degrees
    if (angle < 0) {
        angle += 360.0f;  // Convert to positive value
    }
    if (angle > 180.0f) {
        angle -= 180.0f;  // Subtract 180 from angles greater than 180
    }

    // Print adjusted angle
    std::cout << "Adjusted angle: " << angle << std::endl;

    // Convert to normal angle value
    float normal_angle = fmod(angle, 180.0f);
    if (normal_angle < 0) {
        normal_angle += 180.0f;  // Ensure it's a positive value
    }

    // Print normal angle value
    std::cout << "Normal angle: " << normal_angle << std::endl;

    cos_value = std::cos(angle * CV_PI / 180.0f);  // Convert to radians
    sin_value = std::sin(angle * CV_PI / 180.0f);

    // Calculate each corner point
    float l = x1 - w / 2;  // Left boundary
    float r = x1 + w / 2;  // Right boundary
    float t = y1 - h / 2;  // Top boundary
    float b = y1 + h / 2;  // Bottom boundary

    // Use get_rect function to scale the coordinates
    float bbox[4] = {l, t, r, b};
    cv::Rect rect = get_rect(img, bbox);

    float x_ = (rect.x + rect.x + rect.width) / 2;   // Center x
    float y_ = (rect.y + rect.y + rect.height) / 2;  // Center y
    float width = rect.width;                        // Width
    float height = rect.height;                      // Height

    // Calculate each corner point
    std::vector<cv::Point> corner_points(4);
    float vec1x = width / 2 * cos_value;
    float vec1y = width / 2 * sin_value;
    float vec2x = -height / 2 * sin_value;
    float vec2y = height / 2 * cos_value;

    corner_points[0] = cv::Point(int(round(x_ + vec1x + vec2x)), int(round(y_ + vec1y + vec2y)));  // Top-left corner
    corner_points[1] = cv::Point(int(round(x_ + vec1x - vec2x)), int(round(y_ + vec1y - vec2y)));  // Top-right corner
    corner_points[2] =
            cv::Point(int(round(x_ - vec1x - vec2x)), int(round(y_ - vec1y - vec2y)));  // Bottom-right corner
    corner_points[3] = cv::Point(int(round(x_ - vec1x + vec2x)), int(round(y_ - vec1y + vec2y)));  // Bottom-left corner

    // Check and adjust corner points to ensure the rectangle is parallel to image boundaries
    for (auto& point : corner_points) {
        point.x = std::max(0, std::min(point.x, img.cols - 1));
        point.y = std::max(0, std::min(point.y, img.rows - 1));
    }

    return corner_points;
}

void draw_bbox_obb(std::vector<cv::Mat>& img_batch, std::vector<std::vector<Detection>>& res_batch) {
    static std::vector<uint32_t> colors = {0xFF3838, 0xFF9D97, 0xFF701F, 0xFFB21D, 0xCFD231, 0x48F90A, 0x92CC17,
                                           0x3DDB86, 0x1A9334, 0x00D4BB, 0x2C99A8, 0x00C2FF, 0x344593, 0x6473FF,
                                           0x0018EC, 0x8438FF, 0x520085, 0xCB38FF, 0xFF95C8, 0xFF37C7};
    for (size_t i = 0; i < img_batch.size(); i++) {
        auto& res = res_batch[i];
        auto& img = img_batch[i];
        for (auto& obj : res) {
            auto color = colors[(int)obj.class_id % colors.size()];
            auto bgr = cv::Scalar(color & 0xFF, color >> 8 & 0xFF, color >> 16 & 0xFF);
            auto corner_points = get_corner(img, obj);
            cv::polylines(img, std::vector<std::vector<cv::Point>>{corner_points}, true, bgr, 1);

            auto text = (std::to_string((int)(obj.class_id)) + ":" + to_string_with_precision(obj.conf));
            cv::Size textsize = cv::getTextSize(text, 0, 0.3, 1, nullptr);

            int width = textsize.width;
            int height = textsize.height;
            bool outside = (corner_points[0].y - height >= 3) ? true : false;
            cv::Point p1(corner_points[0].x, corner_points[0].y), p2;
            p2.x = corner_points[0].x + width;
            if (outside) {
                p2.y = corner_points[0].y - height - 3;
            } else {
                p2.y = corner_points[0].y + height + 3;
            }
            cv::rectangle(img, p1, p2, bgr, -1, cv::LINE_AA);
            cv::putText(
                    img, text,
                    cv::Point(corner_points[0].x, (outside ? corner_points[0].y - 2 : corner_points[0].y + height + 2)),
                    0, 0.3, cv::Scalar::all(255), 1, cv::LINE_AA);
        }
    }
}

void draw_drivable_highlight(cv::Mat& img, std::vector<Detection>& dets, std::vector<cv::Mat>& masks,
                             std::unordered_map<int, std::string>& labels_map) {
    #if !ENABLE_DRIVABLE_HIGHLIGHT
        return;
    #endif

    // Separar detecções de drivable (usar a mesma lógica da Mask View)
    std::vector<Detection> drivable_dets;
    std::vector<cv::Mat> drivable_masks;

    for (size_t i = 0; i < dets.size(); i++) {
        auto it = labels_map.find(dets[i].class_id);
        if (it != labels_map.end() && it->second == "drivable") {
            drivable_dets.push_back(dets[i]);
            drivable_masks.push_back(masks[i]);
        }
    }

    if (drivable_dets.empty()) return;

    // ====== DESENHAR ÁREAS DRIVABLE (MESMA LÓGICA DA MASK VIEW) ======
    for (size_t i = 0; i < drivable_dets.size(); i++) {
        // Usar scale_mask para correção de proporção igual à Mask View
        cv::Mat img_mask = scale_mask(drivable_masks[i], img);

        // Usar cor ciano para compatibilidade visual
        cv::Scalar bgr = cv::Scalar(255, 255, 0); // Ciano (BGR)

        cv::Rect r = get_rect(img, drivable_dets[i].bbox);
        for (int x = r.x; x < r.x + r.width; x++) {
            for (int y = r.y; y < r.y + r.height; y++) {
                if (x < 0 || x >= img.cols || y < 0 || y >= img.rows) continue;
                float val = img_mask.at<float>(y, x);
                if (val <= 0.5)
                    continue;

                // Aplicar a mesma mistura de cores que a Mask View usa
                img.at<cv::Vec3b>(y, x)[0] = img.at<cv::Vec3b>(y, x)[0] / 2 + bgr[0] / 2;
                img.at<cv::Vec3b>(y, x)[1] = img.at<cv::Vec3b>(y, x)[1] / 2 + bgr[1] / 2;
                img.at<cv::Vec3b>(y, x)[2] = img.at<cv::Vec3b>(y, x)[2] / 2 + bgr[2] / 2;
            }
        }
    }
}


