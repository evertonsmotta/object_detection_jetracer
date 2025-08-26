// ./yolov8_seg -d best_202507181755.engine ~/Documents/recordingSeameRoad/frames/ c my_classes.txt

// ./yolov8_seg -d best_202507181755.engine cam c my_classes.txt

// gst-launch-1.0 -v udpsrc port=5000 caps="application/x-rtp, media=video, encoding-name=H264, payload=96, clock-rate=90000" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <experimental/filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <thread>
#include <chrono>
#include <mosquitto.h>
#include <signal.h>
#include "cuda_utils.h"
#include "logging.h"
#include "model.h"
#include "postprocess.h"
#include "preprocess.h"
#include "utils.h"
#include "performance_config.hpp"  // Configurações de performance
#include "drivable_visualization_config.h"

// ====== CONFIGURAÇÕES DO MQTT ======
// HiveMQ Cloud (ATIVO)
#define MQTT_BROKER "972e24210b544ba49bfb9c1d3164d02b.s1.eu.hivemq.cloud"
#define MQTT_PORT 8883
#define MQTT_USERNAME "jetracer"
#define MQTT_PASSWORD "Ft_seame5"

// Broker Local (comentado)
// #define MQTT_BROKER "10.21.221.67"
// #define MQTT_PORT 1883
#define MQTT_TOPIC_LANE_TOUCH "jetracer/lane_touch"
#define MQTT_TOPIC_PASSADEIRA "jetracer/passadeira"
#define MQTT_TOPIC_STOP_SIGN "jetracer/stop_sign"
#define MQTT_TOPIC_SPEED_50 "jetracer/speed_50"
#define MQTT_TOPIC_SPEED_80 "jetracer/speed_80"
#define MQTT_TOPIC_JETRACER "jetracer/jetracer"
#define MQTT_TOPIC_GATE "jetracer/gate"

// ====== INCLUSÕES PARA SISTEMA MPC ======
#include "autonomous_controller.hpp"
#include "jetracer.hpp"

// ====== VARIÁVEIS GLOBAIS ======
std::atomic<bool> g_running{true};

// ====== VARIÁVEIS MQTT ======
struct mosquitto *mosq = nullptr;
bool mqtt_connected = false;
std::atomic<bool> mqtt_running{false};

// ====== TRATAMENTO DE SINAL ======
void signal_handler(int signal) {
    std::cout << "\n[INFO] Sinal recebido: " << signal << " - Encerrando aplicação..." << std::endl;
    g_running = false;
}

// ====== FUNÇÕES MQTT PARA CLASSES ESPECÍFICAS ======
void publishPassadeira(bool detected) {
    if (!mqtt_connected || !mosq) {
        std::cerr << "[MQTT] Não conectado, pulando publicação de passadeira" << std::endl;
        return;
    }

    std::string message = detected ? "1" : "0";
    int ret = mosquitto_publish(mosq, NULL, MQTT_TOPIC_PASSADEIRA, message.size(), message.c_str(), 0, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cerr << "[MQTT] Falha ao publicar passadeira: " << mosquitto_strerror(ret) << std::endl;
        mqtt_connected = false; // Marcar como desconectado para tentar reconectar
    } else {
        std::cout << "[MQTT] Passadeira: " << message << std::endl;
    }
}

void publishStopSign(bool detected) {
    if (!mqtt_connected || !mosq) {
        std::cerr << "[MQTT] Não conectado, pulando publicação de stop sign" << std::endl;
        return;
    }

    std::string message = detected ? "1" : "0";
    int ret = mosquitto_publish(mosq, NULL, MQTT_TOPIC_STOP_SIGN, message.size(), message.c_str(), 0, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cerr << "[MQTT] Falha ao publicar stop sign: " << mosquitto_strerror(ret) << std::endl;
        mqtt_connected = false; // Marcar como desconectado para tentar reconectar
    } else {
        std::cout << "[MQTT] Stop Sign: " << message << std::endl;
    }
}

void publishSpeed50(bool detected) {
    if (!mqtt_connected || !mosq) {
        std::cerr << "[MQTT] Não conectado, pulando publicação de speed 50" << std::endl;
        return;
    }

    std::string message = detected ? "1" : "0";
    int ret = mosquitto_publish(mosq, NULL, MQTT_TOPIC_SPEED_50, message.size(), message.c_str(), 0, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cerr << "[MQTT] Falha ao publicar speed 50: " << mosquitto_strerror(ret) << std::endl;
        mqtt_connected = false; // Marcar como desconectado para tentar reconectar
    } else {
        std::cout << "[MQTT] Speed 50: " << message << std::endl;
    }
}

void publishSpeed80(bool detected) {
    if (!mqtt_connected || !mosq) {
        std::cerr << "[MQTT] Não conectado, pulando publicação de speed 80" << std::endl;
        return;
    }

    std::string message = detected ? "1" : "0";
    int ret = mosquitto_publish(mosq, NULL, MQTT_TOPIC_SPEED_80, message.size(), message.c_str(), 0, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cerr << "[MQTT] Falha ao publicar speed 80: " << mosquitto_strerror(ret) << std::endl;
        mqtt_connected = false; // Marcar como desconectado para tentar reconectar
    } else {
        std::cout << "[MQTT] Speed 80: " << message << std::endl;
    }
}

void publishJetRacer(bool detected) {
    if (!mqtt_connected || !mosq) {
        std::cerr << "[MQTT] Não conectado, pulando publicação de jetracer" << std::endl;
        return;
    }

    std::string message = detected ? "1" : "0";
    int ret = mosquitto_publish(mosq, NULL, MQTT_TOPIC_JETRACER, message.size(), message.c_str(), 0, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cerr << "[MQTT] Falha ao publicar jetracer: " << mosquitto_strerror(ret) << std::endl;
        mqtt_connected = false; // Marcar como desconectado para tentar reconectar
    } else {
        std::cout << "[MQTT] JetRacer: " << message << std::endl;
    }
}

void publishGate(bool detected) {
    if (!mqtt_connected || !mosq) {
        std::cerr << "[MQTT] Não conectado, pulando publicação de gate" << std::endl;
        return;
    }

    std::string message = detected ? "1" : "0";
    int ret = mosquitto_publish(mosq, NULL, MQTT_TOPIC_GATE, message.size(), message.c_str(), 0, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        std::cerr << "[MQTT] Falha ao publicar gate: " << mosquitto_strerror(ret) << std::endl;
        mqtt_connected = false; // Marcar como desconectado para tentar reconectar
    } else {
        std::cout << "[MQTT] Gate: " << message << std::endl;
    }
}

// ====== CALLBACKS MQTT ======
void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) {
        std::cout << "[MQTT] Conectado com sucesso ao broker!" << std::endl;
        std::cout << "[MQTT] Conexão TLS estabelecida e autenticada" << std::endl;
        mqtt_connected = true;
    } else {
        std::cerr << "[MQTT] Falha na conexão, código: " << rc << std::endl;

        // Interpretar códigos de falha de conexão
        switch (rc) {
            case 1:
                std::cerr << "[MQTT] Protocolo incorreto" << std::endl;
                break;
            case 2:
                std::cerr << "[MQTT] Identificador de cliente inválido" << std::endl;
                break;
            case 3:
                std::cerr << "[MQTT] Servidor indisponível" << std::endl;
                break;
            case 4:
                std::cerr << "[MQTT] Credenciais inválidas" << std::endl;
                break;
            case 5:
                std::cerr << "[MQTT] Não autorizado" << std::endl;
                break;
            case 6:
                std::cerr << "[MQTT] Erro de rede" << std::endl;
                break;
            default:
                std::cerr << "[MQTT] Código de erro desconhecido" << std::endl;
                break;
        }

        mqtt_connected = false;
    }
}

void on_disconnect(struct mosquitto *mosq, void *obj, int rc) {
    std::cout << "[MQTT] Desconectado do broker, código: " << rc << std::endl;

    // Interpretar códigos de desconexão
    switch (rc) {
        case 0:
            std::cout << "[MQTT] Desconexão solicitada pelo cliente" << std::endl;
            break;
        case 1:
            std::cout << "[MQTT] Erro de protocolo incorreto" << std::endl;
            break;
        case 2:
            std::cout << "[MQTT] Identificador de cliente inválido" << std::endl;
            break;
        case 3:
            std::cout << "[MQTT] Servidor indisponível" << std::endl;
            break;
        case 4:
            std::cout << "[MQTT] Credenciais inválidas" << std::endl;
            break;
        case 5:
            std::cout << "[MQTT] Não autorizado" << std::endl;
            break;
        case 6:
            std::cout << "[MQTT] Erro de rede" << std::endl;
            break;
        case 7:
            std::cout << "[MQTT] Conexão perdida (timeout/erro TLS)" << std::endl;
            break;
        default:
            std::cout << "[MQTT] Código de erro desconhecido" << std::endl;
            break;
    }

    mqtt_connected = false;
}

// ====== FUNÇÕES DE INICIALIZAÇÃO E LIMPEZA MQTT ======
void initMQTT() {
    mosquitto_lib_init();

    // ClientID único baseado no PID
    std::string cid = "yolov8_detector_" + std::to_string(getpid());
    mosq = mosquitto_new(cid.c_str(), true, nullptr);
    if (!mosq) {
        throw std::runtime_error("Erro ao criar cliente MQTT");
    }

    std::cout << "[MQTT] Cliente criado com ID: " << cid << std::endl;

    // Configurar callbacks
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_disconnect_callback_set(mosq, on_disconnect);

    // Configurar callback de log para debug
    mosquitto_log_callback_set(mosq, [](struct mosquitto *mosq, void *userdata, int level, const char *str) {
        if (level <= MOSQ_LOG_WARNING) { // Apenas logs importantes
            std::cout << "[MQTT LOG] " << str << std::endl;
        }
    });

    // Configurações adicionais para estabilidade
    mosquitto_max_inflight_messages_set(mosq, 20);
    mosquitto_message_retry_set(mosq, 3);

    // Configurar autenticação se necessário (para HiveMQ Cloud)
    #ifdef MQTT_USERNAME
    int auth_ret = mosquitto_username_pw_set(mosq, MQTT_USERNAME, MQTT_PASSWORD);
    if (auth_ret != MOSQ_ERR_SUCCESS) {
        throw std::runtime_error("Erro ao configurar autenticação MQTT: " + std::string(mosquitto_strerror(auth_ret)));
    }
    #endif

    // Configurar TLS para HiveMQ Cloud (porta 8883)
    if (MQTT_PORT == 8883) {
        // Usar bundle de CAs do sistema (mais seguro que tls_insecure_set)
        int tls_ret = mosquitto_tls_set(mosq, "/etc/ssl/certs/ca-certificates.crt", nullptr, nullptr, nullptr, nullptr);
        if (tls_ret != MOSQ_ERR_SUCCESS) {
            std::cerr << "[WARNING] Falha ao configurar TLS com CA bundle: " << mosquitto_strerror(tls_ret) << std::endl;
            std::cerr << "[WARNING] Tentando configuração TLS alternativa..." << std::endl;

            // Fallback: configuração TLS básica
            tls_ret = mosquitto_tls_set(mosq, nullptr, nullptr, nullptr, nullptr, nullptr);
            if (tls_ret != MOSQ_ERR_SUCCESS) {
                std::cerr << "[ERROR] Falha na configuração TLS alternativa: " << mosquitto_strerror(tls_ret) << std::endl;
            }
        } else {
            std::cout << "[MQTT] TLS configurado com CA bundle do sistema" << std::endl;
        }

        // Nota: TLS 1.2 é o padrão no OpenSSL 1.1.1f
        std::cout << "[MQTT] TLS 1.2 será usado por padrão (OpenSSL 1.1.1f)" << std::endl;

        // Configurar opções TLS adicionais
        tls_ret = mosquitto_tls_opts_set(mosq, 1, nullptr, nullptr);
        if (tls_ret != MOSQ_ERR_SUCCESS) {
            std::cerr << "[WARNING] Falha ao configurar opções TLS: " << mosquitto_strerror(tls_ret) << std::endl;
        }

        std::cout << "[MQTT] Configuração TLS robusta aplicada para HiveMQ Cloud" << std::endl;
    }

    // Configurar keepalive otimizado
    int keepalive = 60; // 1 minuto (otimizado para estabilidade)

    // Conectar de forma assíncrona para melhor performance
    int ret = mosquitto_connect_async(mosq, MQTT_BROKER, MQTT_PORT, keepalive);
    if (ret != MOSQ_ERR_SUCCESS) {
        throw std::runtime_error("Erro ao iniciar conexão assíncrona MQTT: " + std::string(mosquitto_strerror(ret)));
    }

    std::cout << "[MQTT] Iniciando conexão assíncrona ao broker MQTT em " << MQTT_BROKER << ":" << MQTT_PORT << " (keepalive: " << keepalive << "s)" << std::endl;

    // Usar o loop da biblioteca (mais eficiente que thread customizada)
    ret = mosquitto_loop_start(mosq);
    if (ret != MOSQ_ERR_SUCCESS) {
        throw std::runtime_error("Erro ao iniciar loop MQTT: " + std::string(mosquitto_strerror(ret)));
    }

    std::cout << "[MQTT] Loop MQTT iniciado com sucesso" << std::endl;
}

void cleanupMQTT() {
    if (mosq) {
        // Parar o loop da biblioteca
        mosquitto_loop_stop(mosq, true);

        // Desconectar e limpar
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosq = nullptr;
    }

    mosquitto_lib_cleanup();
    mqtt_connected = false;
    mqtt_running = false;
    std::cout << "[MQTT] Conexão MQTT encerrada" << std::endl;
}

// ====== FUNÇÃO DE RECONEXÃO MQTT ======
void reconnectMQTT() {
    if (!mqtt_connected && mosq) {
        std::cout << "[MQTT] Tentando reconectar..." << std::endl;

        // Aguardar um pouco antes de tentar reconectar
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Verificar se o cliente ainda existe e está válido
        if (!mqtt_connected) {
            std::cout << "[MQTT] Cliente não está conectado, tentando reconectar..." << std::endl;

            // Tentar reconectar usando o sistema assíncrono
            int ret = mosquitto_reconnect_async(mosq);
            if (ret != MOSQ_ERR_SUCCESS) {
                std::cerr << "[MQTT] Falha na reconexão assíncrona: " << mosquitto_strerror(ret) << std::endl;

                // Se falhar, tentar recriar a conexão
                std::cout << "[MQTT] Tentando recriar conexão..." << std::endl;

                // Parar o loop atual
                mosquitto_loop_stop(mosq, true);
                mosquitto_disconnect(mosq);
                mosquitto_destroy(mosq);

                // Recriar cliente MQTT com ID único
                std::string cid = "yolov8_detector_" + std::to_string(getpid()) + "_reconnect";
                mosq = mosquitto_new(cid.c_str(), true, nullptr);
                if (mosq) {
                    // Reconfigurar callbacks
                    mosquitto_connect_callback_set(mosq, on_connect);
                    mosquitto_disconnect_callback_set(mosq, on_disconnect);

                    // Reconfigurar callback de log
                    mosquitto_log_callback_set(mosq, [](struct mosquitto *mosq, void *userdata, int level, const char *str) {
                        if (level <= MOSQ_LOG_WARNING) {
                            std::cout << "[MQTT LOG] " << str << std::endl;
                        }
                    });

                    // Reconfigurar autenticação
                    #ifdef MQTT_USERNAME
                    mosquitto_username_pw_set(mosq, MQTT_USERNAME, MQTT_PASSWORD);
                    #endif

                    // Reconfigurar TLS
                    if (MQTT_PORT == 8883) {
                        // Usar CA bundle do sistema
                        int tls_ret = mosquitto_tls_set(mosq, "/etc/ssl/certs/ca-certificates.crt", nullptr, nullptr, nullptr, nullptr);
                        if (tls_ret != MOSQ_ERR_SUCCESS) {
                            std::cerr << "[WARNING] Falha ao reconfigurar TLS com CA bundle: " << mosquitto_strerror(tls_ret) << std::endl;
                            // Fallback
                            tls_ret = mosquitto_tls_set(mosq, nullptr, nullptr, nullptr, nullptr, nullptr);
                        }

                        // TLS 1.2 é o padrão no OpenSSL 1.1.1f

                        // Opções TLS
                        mosquitto_tls_opts_set(mosq, 1, nullptr, nullptr);
                    }

                    // Tentar conectar novamente de forma assíncrona
                    ret = mosquitto_connect_async(mosq, MQTT_BROKER, MQTT_PORT, 60);
                    if (ret == MOSQ_ERR_SUCCESS) {
                        // Iniciar loop
                        ret = mosquitto_loop_start(mosq);
                        if (ret == MOSQ_ERR_SUCCESS) {
                            std::cout << "[MQTT] Reconexão bem-sucedida!" << std::endl;
                        } else {
                            std::cerr << "[MQTT] Falha ao iniciar loop após reconexão: " << mosquitto_strerror(ret) << std::endl;
                        }
                    } else {
                        std::cerr << "[MQTT] Falha na reconexão após recriação: " << mosquitto_strerror(ret) << std::endl;
                    }
                }
            } else {
                std::cout << "[MQTT] Reconexão assíncrona iniciada..." << std::endl;
            }
        } else {
            std::cout << "[MQTT] Cliente ainda está conectado, verificando status..." << std::endl;
            // Verificar se realmente está conectado
            if (mqtt_connected) {
                std::cout << "[MQTT] Cliente reconectado com sucesso!" << std::endl;
            }
        }
    }
}

// ====== FUNÇÃO PARA DESENHAR BOUNDING BOXES DAS CLASSES ESPECÍFICAS ======
void drawSpecificClassBBoxes(cv::Mat& img, const std::vector<Detection>& detections,
                             const std::unordered_map<int, std::string>& labels_map) {
    // Cores para cada classe específica
    std::unordered_map<std::string, cv::Scalar> class_colors = {
        {"passadeira", cv::Scalar(0, 255, 255)},      // Ciano
        {"stop sign", cv::Scalar(0, 0, 255)},         // Vermelho
        {"speed 50", cv::Scalar(255, 0, 255)},        // Magenta
        {"speed 80", cv::Scalar(0, 255, 0)},          // Verde
        {"jetracer", cv::Scalar(255, 255, 0)},        // Amarelo
        {"gate", cv::Scalar(255, 165, 0)}             // Laranja
    };

    for (const auto& det : detections) {
        auto it = labels_map.find(det.class_id);
        if (it != labels_map.end()) {
            std::string class_name = it->second;

            // Verificar se é uma das classes que queremos destacar
            if (class_colors.find(class_name) != class_colors.end()) {
                // Obter retângulo do bounding box
                cv::Rect r = get_rect(img, det.bbox);

                // Cor da classe
                cv::Scalar color = class_colors[class_name];

                // Desenhar retângulo
                cv::rectangle(img, r, color, 3);

                // Preparar texto do label
                std::string label = class_name;
                if (class_name == "stop sign") label = "STOP";
                else if (class_name == "speed 50") label = "50";
                else if (class_name == "speed 80") label = "80";

                // Calcular tamanho do texto
                int font_face = cv::FONT_HERSHEY_SIMPLEX;
                double font_scale = 0.8;
                int thickness = 2;
                cv::Size text_size = cv::getTextSize(label, font_face, font_scale, thickness, nullptr);

                // Posição do texto (acima do bbox)
                cv::Point text_pos(r.x, r.y - 10);
                if (text_pos.y < text_size.height + 5) {
                    text_pos.y = r.y + text_size.height + 5;
                }

                // Desenhar fundo do texto
                cv::Point bg_pt1(text_pos.x - 2, text_pos.y - text_size.height - 2);
                cv::Point bg_pt2(text_pos.x + text_size.width + 2, text_pos.y + 2);
                cv::rectangle(img, bg_pt1, bg_pt2, cv::Scalar(0, 0, 0), -1);

                // Desenhar texto
                cv::putText(img, label, text_pos, font_face, font_scale,
                           cv::Scalar(255, 255, 255), thickness);

                // Desenhar confiança (opcional)
                std::string conf_text = std::to_string(static_cast<int>(det.conf * 100)) + "%";
                cv::Point conf_pos(r.x + r.width - text_size.width, r.y + r.height + 20);
                cv::putText(img, conf_text, conf_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                           color, 1);
            }
        }
    }
}

// ====== FUNÇÃO PARA DETECTAR E PUBLICAR CLASSES ESPECÍFICAS ======
void detectAndPublishClasses(const std::vector<Detection>& detections, const std::unordered_map<int, std::string>& labels_map) {
    bool passadeira_detected = false;
    bool stop_sign_detected = false;
    bool speed_50_detected = false;
    bool speed_80_detected = false;
    bool jetracer_detected = false;
    bool gate_detected = false;

    // Verificar cada detecção
    for (const auto& det : detections) {
        auto it = labels_map.find(det.class_id);
        if (it != labels_map.end()) {
            std::string class_name = it->second;

            if (class_name == "passadeira") {
                passadeira_detected = true;
            } else if (class_name == "stop sign") {
                stop_sign_detected = true;
            } else if (class_name == "speed 50") {
                speed_50_detected = true;
            } else if (class_name == "speed 80") {
                speed_80_detected = true;
            } else if (class_name == "jetracer") {
                jetracer_detected = true;
            } else if (class_name == "gate") {
                gate_detected = true;
            }
        }
    }

    // Publicar status de cada classe via MQTT
    publishPassadeira(passadeira_detected);
    publishStopSign(stop_sign_detected);
    publishSpeed50(speed_50_detected);
    publishSpeed80(speed_80_detected);
    publishJetRacer(jetracer_detected);
    publishGate(gate_detected);

    // Log das detecções
    if (passadeira_detected || stop_sign_detected || speed_50_detected ||
        speed_80_detected || jetracer_detected || gate_detected) {
        std::cout << "[DETECTION] Classes detectadas: ";
        if (passadeira_detected) std::cout << "passadeira ";
        if (stop_sign_detected) std::cout << "stop_sign ";
        if (speed_50_detected) std::cout << "speed_50 ";
        if (speed_80_detected) std::cout << "speed_80 ";
        if (jetracer_detected) std::cout << "jetracer ";
        if (gate_detected) std::cout << "gate ";
        std::cout << std::endl;
    }
}

// ====== CONFIGURAÇÕES DO SISTEMA MPC ======
// Ativar sistema MPC
#define ENABLE_MPC_SYSTEM 1

// ====== CONFIGURAÇÕES DE VISUALIZAÇÃO ======
// Habilitar visualização da área crítica
#define ENABLE_CRITICAL_AREA_VISUALIZATION 1

// Habilitar display em tempo real (requer OpenCV com GUI)
#ifdef ENABLE_CRITICAL_AREA_VISUALIZATION
#define ENABLE_DISPLAY 1
#endif

// ====== VARIÁVEIS GLOBAIS DO SISTEMA MPC ======
#if ENABLE_MPC_SYSTEM
std::unique_ptr<mpc::AutonomousController> autonomous_controller;
std::unique_ptr<jetracer::control::JetRacer> jetracer_controller;
bool mpc_system_active = false;
#endif

// ====== VARIÁVEIS GLOBAIS DE CALIBRAÇÃO DA CÂMERA ======
cv::Mat cameraMatrix, distCoeffs;
bool camera_calibration_loaded = false;

Logger gLogger;
using namespace nvinfer1;
const int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
const static int kOutputSegSize = 32 * (kInputH / 4) * (kInputW / 4);

static cv::Rect get_downscale_rect(float bbox[4], float scale) {

    float left = bbox[0];
    float top = bbox[1];
    float right = bbox[0] + bbox[2];
    float bottom = bbox[1] + bbox[3];

    left = left < 0 ? 0 : left;
    top = top < 0 ? 0 : top;
    right = right > kInputW ? kInputW : right;
    bottom = bottom > kInputH ? kInputH : bottom;

    left /= scale;
    top /= scale;
    right /= scale;
    bottom /= scale;
    return cv::Rect(int(left), int(top), int(right - left), int(bottom - top));
}

std::vector<cv::Mat> process_mask(const float* proto, int proto_size, std::vector<Detection>& dets) {

    std::vector<cv::Mat> masks;
    for (size_t i = 0; i < dets.size(); i++) {

        cv::Mat mask_mat = cv::Mat::zeros(kInputH / 4, kInputW / 4, CV_32FC1);
        auto r = get_downscale_rect(dets[i].bbox, 4);

        for (int x = r.x; x < r.x + r.width; x++) {
            for (int y = r.y; y < r.y + r.height; y++) {
                float e = 0.0f;
                for (int j = 0; j < 32; j++) {
                    e += dets[i].mask[j] * proto[j * proto_size / 32 + y * mask_mat.cols + x];
                }
                e = 1.0f / (1.0f + expf(-e));
                mask_mat.at<float>(y, x) = e;
            }
        }
        cv::resize(mask_mat, mask_mat, cv::Size(kInputW, kInputH));
        masks.push_back(mask_mat);
    }
    return masks;
}

void serialize_engine(std::string& wts_name, std::string& engine_name, std::string& sub_type, float& gd, float& gw,
                      int& max_channels) {
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();
    IHostMemory* serialized_engine = nullptr;

    serialized_engine = buildEngineYolov8Seg(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);

    assert(serialized_engine);
    std::ofstream p(engine_name, std::ios::binary);
    if (!p) {
        std::cout << "could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());

    delete serialized_engine;
    delete config;
    delete builder;
}

void deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine,
                        IExecutionContext** context) {
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char* serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    *runtime = createInferRuntime(gLogger);
    assert(*runtime);
    *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
    assert(*engine);
    *context = (*engine)->createExecutionContext();
    assert(*context);
    delete[] serialized_engine;
}

void prepare_buffer(ICudaEngine* engine, float** input_buffer_device, float** output_buffer_device,
                    float** output_seg_buffer_device, float** output_buffer_host, float** output_seg_buffer_host,
                    float** decode_ptr_host, float** decode_ptr_device, std::string cuda_post_process) {
    assert(engine->getNbBindings() == 3);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    const int outputIndex_seg = engine->getBindingIndex("proto");

    assert(inputIndex == 0);
    assert(outputIndex == 1);
    assert(outputIndex_seg == 2);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)input_buffer_device, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)output_buffer_device, kBatchSize * kOutputSize * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)output_seg_buffer_device, kBatchSize * kOutputSegSize * sizeof(float)));

    if (cuda_post_process == "c") {
        *output_buffer_host = new float[kBatchSize * kOutputSize];
        *output_seg_buffer_host = new float[kBatchSize * kOutputSegSize];
    } else if (cuda_post_process == "g") {
        if (kBatchSize > 1) {
            std::cerr << "Do not yet support GPU post processing for multiple batches" << std::endl;
            exit(0);
        }
        // Allocate memory for decode_ptr_host and copy to device
        *decode_ptr_host = new float[1 + kMaxNumOutputBbox * bbox_element];
        CUDA_CHECK(cudaMalloc((void**)decode_ptr_device, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element)));
    }
}

void infer(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, float* output_seg,
           int batchsize, float* decode_ptr_host, float* decode_ptr_device, int model_bboxes,
           std::string cuda_post_process) {
    // infer on the batch asynchronously, and DMA output back to host
    auto start = std::chrono::system_clock::now();
    context.enqueue(batchsize, buffers, stream, nullptr);
    if (cuda_post_process == "c") {

        std::cout << "kOutputSize:" << kOutputSize << std::endl;
        CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost,
                                   stream));
        std::cout << "kOutputSegSize:" << kOutputSegSize << std::endl;
        CUDA_CHECK(cudaMemcpyAsync(output_seg, buffers[2], batchsize * kOutputSegSize * sizeof(float),
                                   cudaMemcpyDeviceToHost, stream));

        auto end = std::chrono::system_clock::now();
        std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << "ms" << std::endl;
    } else if (cuda_post_process == "g") {
        CUDA_CHECK(
                cudaMemsetAsync(decode_ptr_device, 0, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), stream));
        cuda_decode((float*)buffers[1], model_bboxes, kConfThresh, decode_ptr_device, kMaxNumOutputBbox, stream);
        cuda_nms(decode_ptr_device, kNmsThresh, kMaxNumOutputBbox, stream);  //cuda nms
        CUDA_CHECK(cudaMemcpyAsync(decode_ptr_host, decode_ptr_device,
                                   sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), cudaMemcpyDeviceToHost,
                                   stream));
        auto end = std::chrono::system_clock::now();
        std::cout << "inference and gpu postprocess time: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    }

    CUDA_CHECK(cudaStreamSynchronize(stream));
}

bool parse_args(int argc, char** argv, std::string& wts, std::string& engine, std::string& img_dir,
                std::string& sub_type, std::string& cuda_post_process, std::string& labels_filename, float& gd,
                float& gw, int& max_channels) {
    if (argc < 4)
        return false;
    if (std::string(argv[1]) == "-s" && argc == 5) {
        wts = std::string(argv[2]);
        engine = std::string(argv[3]);
        sub_type = std::string(argv[4]);
        if (sub_type == "n") {
            gd = 0.33;
            gw = 0.25;
            max_channels = 1024;
        } else if (sub_type == "s") {
            gd = 0.33;
            gw = 0.50;
            max_channels = 1024;
        } else if (sub_type == "m") {
            gd = 0.67;
            gw = 0.75;
            max_channels = 576;
        } else if (sub_type == "l") {
            gd = 1.0;
            gw = 1.0;
            max_channels = 512;
        } else if (sub_type == "x") {
            gd = 1.0;
            gw = 1.25;
            max_channels = 640;
        } else {
            return false;
        }
    } else if (std::string(argv[1]) == "-d" && argc == 6) {
        engine = std::string(argv[2]);
        img_dir = std::string(argv[3]);
        cuda_post_process = std::string(argv[4]);
        labels_filename = std::string(argv[5]);
    } else {
        return false;
    }
    return true;
}

// ====== FUNÇÃO PARA CARREGAR CALIBRAÇÃO DA CÂMERA ======
bool load_camera_calibration(const std::string& calibration_file) {
    std::cout << "[DEBUG] Tentando abrir arquivo: " << calibration_file << std::endl;

    // Verificar se o arquivo existe primeiro
    std::ifstream file_check(calibration_file);
    if (!file_check.good()) {
        std::cerr << "[ERROR] Arquivo não existe: " << calibration_file << std::endl;
        return false;
    }
    file_check.close();
    std::cout << "[DEBUG] Arquivo existe e é acessível" << std::endl;

    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ERROR] Não foi possível abrir arquivo de calibração: " << calibration_file << std::endl;
        std::cerr << "[ERROR] Possível problema de permissão ou formato do arquivo" << std::endl;
        return false;
    }

    std::cout << "[DEBUG] FileStorage aberto com sucesso" << std::endl;

    // Verificar se as chaves existem no arquivo
    cv::FileNode camera_node = fs["CameraMatrix"];
    cv::FileNode dist_node = fs["DistCoeffs"];

    if (camera_node.empty()) {
        std::cerr << "[ERROR] Chave 'CameraMatrix' não encontrada no arquivo" << std::endl;
        fs.release();
        return false;
    }

    if (dist_node.empty()) {
        std::cerr << "[ERROR] Chave 'DistCoeffs' não encontrada no arquivo" << std::endl;
        fs.release();
        return false;
    }

    std::cout << "[DEBUG] Chaves CameraMatrix e DistCoeffs encontradas" << std::endl;

    // Carregar os dados
    fs["CameraMatrix"] >> cameraMatrix;
    fs["DistCoeffs"] >> distCoeffs;
    fs.release();

    std::cout << "[DEBUG] CameraMatrix carregado: " << cameraMatrix.size() << std::endl;
    std::cout << "[DEBUG] DistCoeffs carregado: " << distCoeffs.size() << std::endl;

    if (cameraMatrix.empty() || distCoeffs.empty()) {
        std::cerr << "[ERROR] Dados de calibração inválidos no arquivo: " << calibration_file << std::endl;
        std::cerr << "[ERROR] CameraMatrix vazia: " << cameraMatrix.empty() << std::endl;
        std::cerr << "[ERROR] DistCoeffs vazia: " << distCoeffs.empty() << std::endl;
        return false;
    }

    // Verificar se os dados têm o formato correto
    if (cameraMatrix.rows != 3 || cameraMatrix.cols != 3) {
        std::cerr << "[ERROR] CameraMatrix deve ser 3x3, mas é: " << cameraMatrix.size() << std::endl;
        return false;
    }

    if (distCoeffs.rows != 1 || distCoeffs.cols < 4) {
        std::cerr << "[ERROR] DistCoeffs deve ter pelo menos 4 coeficientes, mas tem: " << distCoeffs.size() << std::endl;
        return false;
    }

    std::cout << "[SUCCESS] Calibração da câmera carregada com sucesso!" << std::endl;
    std::cout << "[INFO] Matriz da câmera: " << cameraMatrix.size() << std::endl;
    std::cout << "[INFO] Coeficientes de distorção: " << distCoeffs.size() << std::endl;
    std::cout << "[DEBUG] cameraMatrix: " << cameraMatrix << std::endl;
    std::cout << "[DEBUG] distCoeffs: " << distCoeffs << std::endl;

    camera_calibration_loaded = true;
    std::cout << "[DEBUG] camera_calibration_loaded definido como true" << std::endl;
    return true;
}

// ====== FUNÇÃO PARA APLICAR CORREÇÃO DE DISTORÇÃO ======
cv::Mat apply_camera_correction(const cv::Mat& input_frame) {
    if (!camera_calibration_loaded || input_frame.empty()) {
        std::cout << "[WARNING] apply_camera_correction: calibração não carregada ou frame vazio" << std::endl;
        return input_frame;
    }

    cv::Mat corrected_frame;
    try {
        cv::undistort(input_frame, corrected_frame, cameraMatrix, distCoeffs);
        std::cout << "[DEBUG] Correção aplicada com sucesso - Input: " << input_frame.size()
                  << " | Output: " << corrected_frame.size() << std::endl;

        // Debug adicional: verificar se a correção realmente mudou algo
        if (input_frame.size() == corrected_frame.size()) {
            cv::Mat diff;
            cv::absdiff(input_frame, corrected_frame, diff);
            double mean_diff = cv::mean(diff)[0];
            std::cout << "[DEBUG] Diferença média entre frame original e corrigido: " << mean_diff << std::endl;

            if (mean_diff < 1.0) {
                std::cout << "[WARNING] Correção pode não estar funcionando - diferença muito pequena" << std::endl;
            }
        }
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] Falha ao aplicar correção de distorção: " << e.what() << std::endl;
        return input_frame;
    }

    return corrected_frame;
}

int main(int argc, char** argv) {
    // ====== CONFIGURAR TRATAMENTO DE SINAL ======
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cudaSetDevice(kGpuId);

    std::string wts_name = "";
    std::string engine_name = "";
    std::string img_dir;
    std::string sub_type = "";
    std::string cuda_post_process = "";
    std::string labels_filename = "../coco.txt";
    int model_bboxes = 0;
    float gd = 0.0f, gw = 0.0f;
    int max_channels = 0;

    if (!parse_args(argc, argv, wts_name, engine_name, img_dir, sub_type, cuda_post_process, labels_filename, gd, gw,
                    max_channels)) {
        std::cerr << "Arguments not right!\n";
        std::cerr << "./yolov8 -s [.wts] [.engine] [n/s/m/l/x]\n";
        std::cerr << "./yolov8 -d [.engine] <dir|cam> [c/g] labels.txt\n";
        return -1;
    }

    // Serialização (opcional)
    if (!wts_name.empty()) {
        serialize_engine(wts_name, engine_name, sub_type, gd, gw, max_channels);
        return 0;
    }

    // Deserialização + buffers
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
    deserialize_engine(engine_name, &runtime, &engine, &context);

    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));
    cuda_preprocess_init(kMaxInputImageSize);

    auto out_dims = engine->getBindingDimensions(1);
    model_bboxes = out_dims.d[0];

    float* device_buffers[3];
    float* output_buffer_host = nullptr;
    float* output_seg_buffer_host = nullptr;
    float* decode_ptr_host = nullptr;
    float* decode_ptr_device = nullptr;

    prepare_buffer(engine, &device_buffers[0], &device_buffers[1], &device_buffers[2], &output_buffer_host,
                   &output_seg_buffer_host, &decode_ptr_host, &decode_ptr_device, cuda_post_process);

    // Labels
    std::unordered_map<int, std::string> labels_map;
    read_labels(labels_filename, labels_map);
    assert(kNumClass == (int)labels_map.size());

    // ====== INICIALIZAÇÃO MQTT ======
    try {
        initMQTT();
        std::cout << "[INFO] MQTT inicializado com sucesso!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Falha ao inicializar MQTT: " << e.what() << std::endl;
        std::cerr << "[WARNING] Continuando sem MQTT..." << std::endl;
    }

    // ====== SISTEMA MPC SIMPLIFICADO ======
    // Sistema MPC temporariamente desabilitado para focar no streaming UDP
    std::cout << "[INFO] Sistema MPC simplificado - foco no streaming UDP" << std::endl;
    std::cout << "[INFO] Use Ctrl+C para sair da aplicação" << std::endl;

    // Diretório de saída (não falha se já existir)
    const std::string out_dir = "frames_and_masks";
    if (::mkdir(out_dir.c_str(), 0777) == -1 && errno != EEXIST) {
        perror("mkdir");
    }

    // Escolha de fonte: diretório vs câmera
    bool use_camera = (img_dir == "cam" || img_dir == "camera");

    // Sistema MPC disponível para uso futuro
    // std::unique_ptr<mpc::AutonomousController> autonomous_controller;
    // std::unique_ptr<jetracer::control::JetRacer> jetracer_controller;

    if (use_camera) {
        // ---- CAMERA (Jetson CSI via GStreamer) ----
        std::string pipeline =
                "nvarguscamerasrc sensor-id=0 ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! "
                "video/x-raw, format=BGR ! appsink";

        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
        if (!cap.isOpened()) {
            std::cerr << "[ERROR] Failed to open camera.\n";
            goto CLEANUP;
        }

        // ====== CARREGAR CALIBRAÇÃO DA CÂMERA ======
        std::cout << "[INFO] Carregando calibração da câmera..." << std::endl;

        // Verificar diferentes caminhos possíveis
        std::vector<std::string> possible_paths = {
            "calibration_camera/calibration.yml",
            "./calibration_camera/calibration.yml",
            "/home/jetson/Documents/e-codes/final/yolov8/calibration_camera/calibration.yml"
        };

        bool calibration_loaded = false;
        for (const auto& path : possible_paths) {
            std::cout << "[DEBUG] Tentando caminho: " << path << std::endl;

            // Verificar se o arquivo existe
            std::ifstream test_file(path);
            if (test_file.good()) {
                std::cout << "[DEBUG] Arquivo existe: " << path << std::endl;
                test_file.close();

                if (load_camera_calibration(path)) {
                    std::cout << "[SUCCESS] Calibração carregada do caminho: " << path << std::endl;
                    std::cout << "[DEBUG] cameraMatrix: " << cameraMatrix.size() << " | distCoeffs: " << distCoeffs.size() << std::endl;
                    std::cout << "[DEBUG] camera_calibration_loaded = " << (camera_calibration_loaded ? "true" : "false") << std::endl;
                    calibration_loaded = true;
                    break;
                } else {
                    std::cout << "[ERROR] Falha ao carregar calibração do caminho: " << path << std::endl;
                }
            } else {
                std::cout << "[DEBUG] Arquivo não encontrado: " << path << std::endl;
            }
        }

        if (!calibration_loaded) {
            std::cout << "[WARNING] Calibração não carregada - usando imagem sem correção" << std::endl;
            std::cout << "[DEBUG] camera_calibration_loaded = " << (camera_calibration_loaded ? "true" : "false") << std::endl;
        }

        // === VIDEO STREAMING SETUP (UDP) ===
        // IP do PC de destino (troque pelo IP real do outro PC)
        std::string remote_ip = "100.124.102.80";
        int port_out = 5000;            // Porta de recepção no PC remoto
        int out_w = 1280, out_h = 480;  // 2x(640x480) lado-a-lado
        int out_fps = 30;

        // Opção A) Encoder por CPU (x264) — funciona em qualquer máquina
        std::string pipeline_out =
                "appsrc is-live=true block=false format=time "
                "caps=video/x-raw,format=BGR,width=" +
                std::to_string(out_w) + ",height=" + std::to_string(out_h) + ",framerate=" + std::to_string(out_fps) +
                "/1 ! "
                "videoconvert ! "
                "x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 bitrate=2000 ! "
                "rtph264pay config-interval=1 pt=96 ! "
                "udpsink host=" +
                remote_ip + " port=" + std::to_string(port_out) + " sync=false async=false";

        // Opção B) Encoder por hardware (Jetson) — mais leve/rápido
        // OBS: disponível em JetPack 4.x/5.x; caso sua imagem tenha apenas omxh264enc, troque abaixo.
        std::string pipeline_out_hw =
                "appsrc is-live=true block=false format=time "
                "caps=video/x-raw,format=BGR,width=" +
                std::to_string(out_w) + ",height=" + std::to_string(out_h) + ",framerate=" + std::to_string(out_fps) +
                "/1 ! "
                "videoconvert ! "
                "nvv4l2h264enc insert-sps-pps=true iframeinterval=30 idrinterval=30 control-rate=1 bitrate=4000000 "
                "preset-level=1 "
                "! h264parse config-interval=1 "
                "! rtph264pay pt=96 "
                "! udpsink host=" +
                remote_ip + " port=" + std::to_string(port_out) + " sync=false async=false";

        // **Escolha UMA das duas**
        std::cout << "[DEBUG] Pipeline UDP: " << pipeline_out << std::endl;
        cv::VideoWriter udp_writer(pipeline_out, cv::CAP_GSTREAMER, 0, out_fps, cv::Size(out_w, out_h), true);
        // cv::VideoWriter udp_writer(pipeline_out_hw, cv::CAP_GSTREAMER, 0, out_fps, cv::Size(out_w, out_h), true);

        if (!udp_writer.isOpened()) {
            std::cerr << "[ERROR] Failed to open GStreamer UDP writer.\n";
            std::cerr << "[ERROR] Pipeline: " << pipeline_out << std::endl;
            return 1;
        }

        std::cout << "[SUCCESS] UDP Writer aberto com sucesso!" << std::endl;
        std::cout << "[INFO] Enviando para: " << remote_ip << ":" << port_out << std::endl;

        // Janela de visualização removida para focar no streaming UDP
        cv::Mat frame;
        size_t frame_id = 0;

        // ====== VARIÁVEIS PARA MEDIÇÃO DE FPS ======
        auto start_time = std::chrono::high_resolution_clock::now();
        auto last_fps_time = start_time;
        int fps_counter = 0;
        double current_fps = 0.0;
        const int fps_update_interval = PerformanceConfig::FPS_UPDATE_INTERVAL;

        // ====== FLAGS DE OTIMIZAÇÃO DE PERFORMANCE ======
        const bool ENABLE_VISUAL_OVERLAY = PerformanceConfig::ENABLE_VISUAL_OVERLAY_FINAL;
        const bool ENABLE_LANE_LINES = PerformanceConfig::ENABLE_LANE_LINES_FINAL;
        const bool ENABLE_UDP_STREAMING = PerformanceConfig::ENABLE_UDP_STREAMING_FINAL;
        const bool ENABLE_CONSOLE_LOG = PerformanceConfig::ENABLE_CONSOLE_LOG_FINAL;
        const bool ENABLE_FPS_DISPLAY = PerformanceConfig::ENABLE_FPS_DISPLAY_FINAL;

        // Debug: mostrar configuração atual
        std::cout << "[DEBUG] Configuração de Performance:" << std::endl;
        std::cout << "  ENABLE_VISUAL_OVERLAY: " << (ENABLE_VISUAL_OVERLAY ? "true" : "false") << std::endl;
        std::cout << "  ENABLE_LANE_LINES: " << (ENABLE_LANE_LINES ? "true" : "false") << std::endl;
        std::cout << "  ENABLE_UDP_STREAMING: " << (ENABLE_UDP_STREAMING ? "true" : "false") << std::endl;
        std::cout << "  ENABLE_CONSOLE_LOG: " << (ENABLE_CONSOLE_LOG ? "true" : "false") << std::endl;
        std::cout << "  ENABLE_FPS_DISPLAY: " << (ENABLE_FPS_DISPLAY ? "true" : "false") << std::endl;
        std::cout << "[DEBUG] Streaming UDP: " << (ENABLE_UDP_STREAMING ? "HABILITADO" : "DESABILITADO") << std::endl;
        std::cout << "[DEBUG] IP de destino: 100.124.102.80:5000" << std::endl;

        // ====== OTIMIZAÇÕES DE PERFORMANCE ======
        cv::Mat left_view(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat combined_view(480, 1280, CV_8UC3);
        cv::Mat lane_bin;
        std::vector<cv::Mat> masks_full;

        // Variáveis para o sistema de segurança
        cv::Mat img;
        std::vector<cv::Mat> masks;
        bool save_img = false;

        // Pré-alocar buffers para evitar alocações no loop
        masks_full.reserve(PerformanceConfig::MAX_MASKS_PREALLOC);

        while (g_running) {
            auto frame_start_time = std::chrono::high_resolution_clock::now();

            // Verificar se deve encerrar
            if (!g_running) {
                std::cout << "[INFO] Encerrando aplicação..." << std::endl;
                break;
            }

            cap >> frame;
            if (frame.empty())
                break;

            // ====== APLICAR CORREÇÃO DE DISTORÇÃO DA CÂMERA ======
            if (camera_calibration_loaded) {
                cv::Mat original_frame = frame.clone();
                std::cout << "[DEBUG] Aplicando correção de distorção - Frame " << frame_id
                          << " | Tamanho original: " << original_frame.size() << std::endl;

                frame = apply_camera_correction(frame);

                // Log de debug a cada 30 frames para não sobrecarregar
                if (frame_id % 30 == 0) {
                    std::cout << "[DEBUG] Correção de distorção aplicada - Frame " << frame_id
                              << " | Original: " << original_frame.size()
                              << " | Corrigido: " << frame.size() << std::endl;

                    // Verificação adicional: confirmar que a correção foi aplicada
                    if (frame.size() == original_frame.size()) {
                        std::cout << "[DEBUG] Frame corrigido mantém a mesma resolução (640x480)" << std::endl;
                        std::cout << "[DEBUG] Correção aplicada com sucesso - distorção removida" << std::endl;
                    } else {
                        std::cout << "[WARNING] Frame corrigido mudou de resolução - pode indicar problema" << std::endl;
                        std::cout << "[WARNING] Original: " << original_frame.size() << " | Corrigido: " << frame.size() << std::endl;
                    }
                }
            } else {
                // Log de debug a cada 30 frames
                if (frame_id % 30 == 0) {
                    std::cout << "[WARNING] Correção de distorção NÃO aplicada - calibração não carregada" << std::endl;
                }
            }

            // ====== PREPROCESS + INFER ======
            // IMPORTANTE: Verificar se estamos usando o frame corrigido para inferência
            if (camera_calibration_loaded && frame_id % 30 == 0) {
                std::cout << "[DEBUG] Inferência: usando frame corrigido para YOLOv8 - tamanho: " << frame.size() << std::endl;
                std::cout << "[DEBUG] Frame será redimensionado de " << frame.size() << " para " << kInputW << "x" << kInputH << " para YOLOv8" << std::endl;
            }

            std::vector<cv::Mat> img_batch{frame};
            cuda_batch_preprocess(img_batch, device_buffers[0], kInputW, kInputH, stream);

            infer(*context, stream, (void**)device_buffers, output_buffer_host, output_seg_buffer_host, 1,
                  decode_ptr_host, decode_ptr_device, model_bboxes, cuda_post_process);

            // ====== NMS ======
            std::vector<std::vector<Detection>> res_batch;
            batch_nms(res_batch, output_buffer_host, 1, kOutputSize, kConfThresh, kNmsThresh);
            auto& res = res_batch[0];

            // ====== VERIFICAR E RECONECTAR MQTT SE NECESSÁRIO ======
            if (!mqtt_connected && frame_id % 30 == 0) { // Tentar reconectar a cada 30 frames
                std::cout << "[MQTT] Status: Desconectado - tentando reconectar..." << std::endl;

                // Verificar se o cliente MQTT ainda é válido
                if (mosq && mqtt_connected) {
                    std::cout << "[MQTT] Cliente ainda está conectado, atualizando status..." << std::endl;
                } else {
                    reconnectMQTT();
                }
            }

            // ====== DETECTAR E PUBLICAR CLASSES ESPECÍFICAS VIA MQTT ======
            detectAndPublishClasses(res, labels_map);

            // ====== SEPARA DETECÇÕES DE "LANE" ======
            std::vector<Detection> lane_detections;
            lane_detections.clear(); // Reutilizar vetor
            for (const auto& d : res) {
                auto it = labels_map.find(d.class_id);
                if (it != labels_map.end() && it->second == "lane") {
                    lane_detections.push_back(d);
                }
            }

            // ====== MÁSCARA UNIFICADA DAS LANES (OTIMIZADA) ======
            cv::Mat lane_merged = cv::Mat::zeros(kInputH, kInputW, CV_32FC1);
            std::vector<cv::Mat> lane_masks;

            if (!lane_detections.empty()) {
                lane_masks = process_mask(output_seg_buffer_host, kOutputSegSize, lane_detections);
                for (size_t i = 0; i < lane_masks.size(); ++i) {
                    lane_merged = cv::max(lane_merged, lane_masks[i]);
                }

                // Debug: verificar se as máscaras estão sendo geradas corretamente
                if (camera_calibration_loaded && frame_id % 30 == 0) {
                    std::cout << "[DEBUG] Máscaras de lane geradas a partir do frame corrigido" << std::endl;
                    std::cout << "[DEBUG] Tamanho da máscara unificada: " << lane_merged.size() << " (resolução YOLOv8)" << std::endl;
                    std::cout << "[DEBUG] IMPORTANTE: Máscaras serão redimensionadas para 640x480 no Overlay View" << std::endl;
                }
            }

            // ====== MÁSCARA UNIFICADA DAS ÁREAS DRIVABLE ======
            cv::Mat drivable_merged = cv::Mat::zeros(kInputH, kInputW, CV_32FC1);
            std::vector<Detection> drivable_detections;
            std::vector<cv::Mat> drivable_masks;

            // Filtrar detecções drivable
            for (const auto& det : res) {
                auto it = labels_map.find(det.class_id);
                if (it != labels_map.end() && it->second == "drivable") {
                    drivable_detections.push_back(det);
                }
            }

            if (!drivable_detections.empty()) {
                drivable_masks = process_mask(output_seg_buffer_host, kOutputSegSize, drivable_detections);
                for (size_t i = 0; i < drivable_masks.size(); ++i) {
                    drivable_merged = cv::max(drivable_merged, drivable_masks[i]);
                }
            }

            // ====== VERIFICAÇÃO SIMPLIFICADA DE SEGURANÇA ======
            // Apenas verificar se há lanes detectadas
            int lane_count = 0;
            for (const auto& det : res) {
                auto it = labels_map.find(det.class_id);
                if (it != labels_map.end() && it->second == "lane") {
                    lane_count++;
                }
            }

            if (lane_count == 0) {
                std::cout << "[INFO] Nenhuma lane detectada neste frame" << std::endl;
            } else {
                std::cout << "[INFO] Lanes detectadas: " << lane_count << std::endl;
            }

        // ====== VISUALIZAÇÃO SIMPLIFICADA ======
        // Removida visualização complexa da área crítica para simplificar

            // ====== SISTEMA MPC SIMPLIFICADO ======
            // Sistema MPC temporariamente desabilitado para focar no streaming UDP
            if (frame_id % 60 == 0) { // Log a cada 60 frames
                std::cout << "[INFO] Sistema MPC simplificado - foco no streaming UDP" << std::endl;
                std::cout << "[MQTT] Status da conexão: " << (mqtt_connected ? "CONECTADO" : "DESCONECTADO") << std::endl;
            }

            // ====== LEFT VIEW: MÁSCARA COMBINADA (640x480) - OTIMIZADA ======
            if (ENABLE_VISUAL_OVERLAY) {
                left_view.setTo(cv::Scalar(0, 0, 0)); // Reset view

                // CRÍTICO: Usar a mesma correção de escala que o Overlay View
                cv::Mat temp_frame = cv::Mat::zeros(640, 480, CV_8UC3); // Frame temporário para scale_mask
                cv::Mat combined_mask = cv::Mat::zeros(640, 480, CV_8UC3);

                // Aplicar máscara das lanes em verde usando scale_mask
                if (!lane_detections.empty()) {
                    for (size_t i = 0; i < lane_masks.size(); ++i) {
                        // Usar scale_mask para correção de proporção igual ao Overlay View
                        cv::Mat lane_scaled = scale_mask(lane_masks[i], temp_frame);

                        // Converter para binário
                        cv::Mat lane_bin;
                        cv::threshold(lane_scaled, lane_bin, 0.5, 255.0, cv::THRESH_BINARY);
                        lane_bin.convertTo(lane_bin, CV_8UC1);

                        // Criar máscara colorida para lanes (verde)
                        cv::Mat lane_color_mask = cv::Mat::zeros(640, 480, CV_8UC3);
                        lane_color_mask.setTo(cv::Scalar(0, 255, 0), lane_bin);
                        combined_mask = cv::max(combined_mask, lane_color_mask);
                    }
                }

                // Aplicar máscara das áreas drivable em azul usando scale_mask
                if (!drivable_detections.empty()) {
                    for (size_t i = 0; i < drivable_masks.size(); ++i) {
                        // Usar scale_mask para correção de proporção igual ao Overlay View
                        cv::Mat drivable_scaled = scale_mask(drivable_masks[i], temp_frame);

                        // Converter para binário
                        cv::Mat drivable_bin;
                        cv::threshold(drivable_scaled, drivable_bin, 0.5, 255.0, cv::THRESH_BINARY);
                        drivable_bin.convertTo(drivable_bin, CV_8UC1);

                        // Criar máscara colorida para áreas drivable (azul)
                        cv::Mat drivable_color_mask = cv::Mat::zeros(640, 480, CV_8UC3);
                        drivable_color_mask.setTo(cv::Scalar(255, 0, 0), drivable_bin);
                        combined_mask = cv::max(combined_mask, drivable_color_mask);
                    }
                }

                // IMPORTANTE: Aplicar correção de distorção na máscara combinada se necessário
                if (camera_calibration_loaded) {
                    // A máscara já foi gerada a partir do frame corrigido e agora usa scale_mask
                    std::cout << "[DEBUG] Mask View: usando máscaras do frame corrigido com scale_mask" << std::endl;
                }

                // A máscara combinada já está na resolução correta (640x480)
                left_view = combined_mask.clone();

                // CRÍTICO: Garantir que left_view tenha exatamente o mesmo tamanho
                if (left_view.size() != cv::Size(640, 480)) {
                    std::cout << "[WARNING] Redimensionando left_view de " << left_view.size() << " para 640x480" << std::endl;
                    cv::resize(left_view, left_view, cv::Size(640, 480));
                }

                // Desenhar texto de uma vez só
                cv::putText(left_view, "Mask View", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 255, 255}, 2);
                cv::putText(left_view, "Lanes: " + std::to_string(lane_detections.size()), {10, 70},
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 255}, 2);
                cv::putText(left_view, "Drivable: " + std::to_string(drivable_detections.size()), {10, 100},
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 255}, 2);

                // Status simplificado
                cv::putText(left_view, "STATUS: STREAMING", {10, 140}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
                cv::putText(left_view, "MODE: SIMPLIFIED", {10, 170}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 165, 255}, 2);

                // Adicionar informação de debug sobre a correção
                if (camera_calibration_loaded) {
                    cv::putText(left_view, "CORRIGIDO", {10, 200}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
                    cv::putText(left_view, "scale_mask()", {10, 225}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
                } else {
                    cv::putText(left_view, "SEM CORRECAO", {10, 200}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 0, 0}, 2);
                }

                if (ENABLE_FPS_DISPLAY) {
                    cv::putText(left_view, "FPS: " + std::to_string(static_cast<int>(current_fps)), {10, 250},
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 255}, 2);
                }
            } else {
                cv::putText(left_view, "No lanes detected", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 255, 255}, 2);
                if (ENABLE_FPS_DISPLAY) {
                    cv::putText(left_view, "FPS: " + std::to_string(static_cast<int>(current_fps)), {10, 70},
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 255}, 2);
                }
            }

            // ====== RIGHT VIEW: OVERLAY SOBRE O FRAME (640x480) - OTIMIZADA ======
            cv::Mat right_view;
            if (ENABLE_VISUAL_OVERLAY) {
                // IMPORTANTE: Garantir que estamos usando o frame corrigido
                if (camera_calibration_loaded) {
                    // O frame já foi corrigido anteriormente no loop principal
                    right_view = frame.clone(); // frame já corrigido (640x480)
                    std::cout << "[DEBUG] Overlay View: usando frame corrigido - tamanho: " << frame.size() << std::endl;

                    // CRÍTICO: Verificar se as máscaras estão na resolução correta
                    if (frame.size() == cv::Size(640, 480)) {
                        std::cout << "[DEBUG] Overlay View: frame corrigido confirmado - resolução da câmera (640x480)" << std::endl;
                        std::cout << "[DEBUG] IMPORTANTE: Máscaras serão redimensionadas de 320x320 para 640x480" << std::endl;
                    } else {
                        std::cout << "[WARNING] Overlay View: resolução inesperada - esperado 640x480, obtido " << frame.size() << std::endl;
                    }
                } else {
                    right_view = frame.clone(); // frame original se não houver calibração
                    std::cout << "[DEBUG] Overlay View: usando frame original - sem correção" << std::endl;
                }

                // Processar máscaras apenas se necessário
                if (!res.empty()) {
                    masks_full = process_mask(output_seg_buffer_host, kOutputSegSize, res);

                    // IMPORTANTE: Verificar se as máscaras estão sendo aplicadas no frame correto
                    if (camera_calibration_loaded && frame_id % 30 == 0) {
                        std::cout << "[DEBUG] Overlay View: aplicando máscaras no frame corrigido" << std::endl;
                        std::cout << "[DEBUG] Número de máscaras: " << masks_full.size() << std::endl;
                        std::cout << "[DEBUG] Tamanho do right_view: " << right_view.size() << " (resolução da câmera)" << std::endl;
                        std::cout << "[DEBUG] CRÍTICO: Máscaras devem ser redimensionadas de 320x320 para 640x480" << std::endl;
                    }

                    // Ocultar drivable area e bboxs - usar apenas draw_mask_only para lanes
                    draw_mask_only(right_view, res, masks_full, labels_map);
                    draw_drivable_highlight(right_view, res, masks_full, labels_map);

                    // Desenhar bounding boxes das classes específicas
                    drawSpecificClassBBoxes(right_view, res, labels_map);

                    // Adicionar visualização das linhas das faixas (opcional)
                    if (ENABLE_LANE_LINES) {
                        draw_lane_lines(right_view, res, masks_full, labels_map);
                    }
                }

                cv::putText(right_view, "Overlay View", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);

                // Adicionar informação de debug sobre a correção
                if (camera_calibration_loaded) {
                    cv::putText(right_view, "CORRIGIDO", {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
                    cv::putText(right_view, "640x480", {10, 85}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
                } else {
                    cv::putText(right_view, "SEM CORRECAO", {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 0, 0}, 2);
                }

                if (ENABLE_FPS_DISPLAY) {
                    cv::putText(right_view, "FPS: " + std::to_string(static_cast<int>(current_fps)), {10, 110},
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
                }

                // Debug: verificar se right_view foi criada corretamente
                if (right_view.empty()) {
                    std::cerr << "[ERROR] right_view está vazia após processamento!" << std::endl;
                }

                // CRÍTICO: Garantir que right_view tenha exatamente o mesmo tamanho
                if (right_view.size() != cv::Size(640, 480)) {
                    std::cout << "[WARNING] Redimensionando right_view de " << right_view.size() << " para 640x480" << std::endl;
                    cv::resize(right_view, right_view, cv::Size(640, 480));
                }
            } else {
                // Modo de performance: apenas clonar o frame sem processamento visual
                right_view = frame.clone();
                if (ENABLE_FPS_DISPLAY) {
                    cv::putText(right_view, "FPS: " + std::to_string(static_cast<int>(current_fps)), {10, 30},
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
                }

                // Debug: verificar se right_view foi criada corretamente
                if (right_view.empty()) {
                    std::cerr << "[ERROR] right_view está vazia no modo performance!" << std::endl;
                }

                // CRÍTICO: Garantir que right_view tenha exatamente o mesmo tamanho
                if (right_view.size() != cv::Size(640, 480)) {
                    std::cout << "[WARNING] Redimensionando right_view (performance) de " << right_view.size() << " para 640x480" << std::endl;
                    cv::resize(right_view, right_view, cv::Size(640, 480));
                }
            }

                        // ====== CONTROLE SIMPLIFICADO ======
            // Controle via teclado simplificado
            // Use Ctrl+C para sair da aplicação

            // Aguardar um pequeno intervalo para não sobrecarregar o sistema
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // ====== COMBINA E ENVIA (1280x480) - OTIMIZADA ======
            if (ENABLE_UDP_STREAMING) {
                // Garantir que as views estejam prontas antes de combinar
                if (left_view.empty() || right_view.empty()) {
                    std::cerr << "[WARNING] Views vazias, pulando frame" << std::endl;
                    continue;
                }

                // Usar hconcat com destino pré-alocado para evitar alocações
                cv::hconcat(left_view, right_view, combined_view);

                // Verificar se a combinação foi bem-sucedida
                if (combined_view.empty()) {
                    std::cerr << "[ERROR] Falha ao combinar views" << std::endl;
                    continue;
                }

                                // Tentar enviar o frame
                try {
                    udp_writer.write(combined_view);

                    // Log a cada 30 frames para não sobrecarregar
                    if (frame_id % 30 == 0) {
                        std::cout << "[UDP] Frame " << frame_id << " enviado com sucesso. Tamanho: "
                                  << combined_view.cols << "x" << combined_view.rows << std::endl;
                    }
                } catch (const cv::Exception& e) {
                    std::cerr << "[ERROR] Falha ao enviar frame UDP: " << e.what() << std::endl;
                }
            }

            // ====== CÁLCULO DE FPS ======
            fps_counter++;
            auto current_time = std::chrono::high_resolution_clock::now();

            // Calcular FPS a cada intervalo definido
            if (fps_counter % fps_update_interval == 0) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_fps_time).count();
                current_fps = (fps_update_interval * 1000.0) / elapsed;
                last_fps_time = current_time;
            }

            // (opcional) log com FPS
            if (ENABLE_CONSOLE_LOG && (++frame_id % PerformanceConfig::CONSOLE_LOG_INTERVAL) == 0) {
                std::cout << "Processed frame: " << frame_id
                          << " | Lanes detected: " << lane_detections.size()
                          << " | FPS: " << std::fixed << std::setprecision(1) << current_fps << std::endl;
            }
        }

        cap.release();
        cv::destroyAllWindows();

        // ====== LIMPEZA MQTT ======
        cleanupMQTT();

    } else {
        // ---- DIRETÓRIO DE IMAGENS (fluxo original) ----
        std::vector<std::string> file_names;
        if (read_files_in_dir(img_dir.c_str(), file_names) < 0) {
            std::cerr << "read_files_in_dir failed.\n";
            goto CLEANUP;
        }

        for (size_t i = 0; i < file_names.size(); i += kBatchSize) {
            // Lote
            std::vector<cv::Mat> img_batch;
            std::vector<std::string> img_name_batch;
            for (size_t j = i; j < i + kBatchSize && j < file_names.size(); ++j) {
                cv::Mat img = cv::imread(img_dir + "/" + file_names[j]);
                img_batch.push_back(img);
                img_name_batch.push_back(file_names[j]);
            }

            // Preprocess
            cuda_batch_preprocess(img_batch, device_buffers[0], kInputW, kInputH, stream);

            // Inferência
            infer(*context, stream, (void**)device_buffers, output_buffer_host, output_seg_buffer_host, kBatchSize,
                  decode_ptr_host, decode_ptr_device, model_bboxes, cuda_post_process);

            if (cuda_post_process == "c") {
                std::vector<std::vector<Detection>> res_batch;
                batch_nms(res_batch, output_buffer_host, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

                for (size_t b = 0; b < img_batch.size(); ++b) {
                    auto& res = res_batch[b];
                    cv::Mat img = img_batch[b];

                    auto masks = process_mask(&output_seg_buffer_host[b * kOutputSegSize], kOutputSegSize, res);

                    // União das lanes + salvar máscara unificada
                    cv::Mat lane_merged = cv::Mat::zeros(kInputH, kInputW, CV_32FC1);
                    for (size_t k = 0; k < res.size(); ++k) {
                        int cls = res[k].class_id;
                        auto it = labels_map.find(cls);
                        if (it != labels_map.end() && it->second == "lane") {
                            lane_merged = cv::max(lane_merged, masks[k]);
                        }
                    }
                    cv::Mat lane_bin;
                    cv::threshold(lane_merged, lane_bin, 0.5, 255.0, cv::THRESH_BINARY);
                    lane_bin.convertTo(lane_bin, CV_8UC1);
                    cv::imwrite(out_dir + "/_" + img_name_batch[b] + "_lane_mask.png", lane_bin);

                    // Desenho padrão e salvar
                    draw_mask_only(img, res, masks, labels_map);
                    draw_drivable_highlight(img, res, masks, labels_map);

                    // Adicionar visualização das linhas das faixas
                    draw_lane_lines(img, res, masks, labels_map);

                    // Desenhar bounding boxes das classes específicas
                    drawSpecificClassBBoxes(img, res, labels_map);

                    cv::imwrite(out_dir + "/_" + img_name_batch[b], img);
                }
            } else if (cuda_post_process == "g") {
                std::cerr << "seg_postprocess is not support in gpu right now\n";
            }
        }
    }

CLEANUP:
    // ====== LIMPEZA MQTT ======
    cleanupMQTT();

    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(device_buffers[0]));
    CUDA_CHECK(cudaFree(device_buffers[1]));
    CUDA_CHECK(cudaFree(device_buffers[2]));
    if (decode_ptr_device)
        CUDA_CHECK(cudaFree(decode_ptr_device));
    delete[] decode_ptr_host;
    delete[] output_buffer_host;
    delete[] output_seg_buffer_host;
    cuda_preprocess_destroy();

    delete context;
    delete engine;
    delete runtime;
    return 0;
}

