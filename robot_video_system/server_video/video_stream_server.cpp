/*
 * Video Streaming Server for Robot Camera
 * Uses GStreamer to capture video from USB camera and stream via UDP
 */

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <signal.h>

class VideoStreamServer {
public:
    VideoStreamServer(const std::string& host, int port, int cameraId = 0) 
        : host_(host), port_(port), cameraId_(cameraId), running_(false) {
        
        // Инициализация GStreamer
        gst_init(NULL, NULL);
    }

    ~VideoStreamServer() {
        stop();
    }

    bool start() {
        if (running_) {
            std::cout << "Server is already running!" << std::endl;
            return false;
        }

        // Создание pipeline для захвата и передачи видео
        std::string pipelineStr = 
            "v4l2src device=/dev/video" + std::to_string(cameraId_) + " ! "
            "videoconvert ! "
            "videoscale ! "
            "video/x-raw,width=640,height=480,format=RGB ! "
            "videorate ! "
            "video/x-raw,framerate=15/1 ! "
            "x264enc speed-preset=ultrafast tune=zerolatency bitrate=500 ! "
            "rtph264pay config-interval=1 pt=96 ! "
            "udpsink host=" + host_ + " port=" + std::to_string(port_);

        pipeline_ = gst_parse_launch(pipelineStr.c_str(), NULL);
        if (!pipeline_) {
            std::cerr << "Failed to create GStreamer pipeline" << std::endl;
            return false;
        }

        // Получение элементов pipeline
        sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");

        // Запуск pipeline
        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cerr << "Failed to start GStreamer pipeline" << std::endl;
            gst_object_unref(pipeline_);
            pipeline_ = NULL;
            return false;
        }

        running_ = true;
        std::cout << "Video streaming started on " << host_ << ":" << port_ << std::endl;

        return true;
    }

    void stop() {
        if (!running_) return;

        running_ = false;

        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = NULL;
        }

        std::cout << "Video streaming stopped" << std::endl;
    }

    bool isRunning() const {
        return running_;
    }

private:
    std::string host_;
    int port_;
    int cameraId_;
    GstElement* pipeline_;
    GstElement* sink_;
    std::atomic<bool> running_;
};

// Глобальные переменные для корректного завершения
VideoStreamServer* g_server = nullptr;
std::atomic<bool> g_running{false};

// Обработчик сигнала для корректного завершения
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", stopping server..." << std::endl;
    g_running = false;
    
    if (g_server) {
        g_server->stop();
    }
    
    exit(0);
}

int main(int argc, char *argv[]) {
    // Установка обработчиков сигналов
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Параметры по умолчанию
    std::string host = "192.168.1.100"; // IP-адрес ноутбука (замените на нужный)
    int port = 5000; // Порт для передачи видео
    int cameraId = 0; // ID камеры (обычно 0 для первой камеры)

    // Проверка аргументов командной строки
    if (argc >= 2) {
        host = argv[1];
    }
    if (argc >= 3) {
        port = std::stoi(argv[2]);
    }
    if (argc >= 4) {
        cameraId = std::stoi(argv[3]);
    }

    std::cout << "=== Robot Video Stream Server ===" << std::endl;
    std::cout << "Streaming video from camera " << cameraId << " to " << host << ":" << port << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    // Создание сервера
    VideoStreamServer server(host, port, cameraId);
    g_server = &server;
    g_running = true;

    // Запуск стриминга
    if (!server.start()) {
        std::cerr << "Failed to start video streaming server" << std::endl;
        return -1;
    }

    // Ожидание завершения
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    server.stop();

    std::cout << "Server exited normally" << std::endl;
    return 0;
}