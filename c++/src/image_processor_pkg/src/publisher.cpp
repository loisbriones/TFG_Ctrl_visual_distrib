#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "image_processor_pkg/msg/object_location.hpp"
#include "opencv2/opencv.hpp"

// Incluimos el header de tu clase ColorDetector
#include "image_processor_pkg/color_detector.hpp"

using namespace std::chrono_literals;

// Mapeo de modos de cámara (Width, Height)
const std::map<int, std::pair<int, int>> CAMERA_MODES = {
    {0, {160, 120}},
    {1, {320, 240}},
    {2, {640, 480}},
    {3, {800, 600}},
    {4, {1280, 720}}
};

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor() : Node("image_processor") {
        // ---- CALL GROUPS ----
        image_processor_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        debug_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        //---- NODE-ID ----
        this->declare_parameter("node_id","rpiTemp");
        node_id = this->get_parameter("node_id").as_string();

        // ----- CAMARA -----
        this->declare_parameter("camera.mode", 2);
        int mode = this->get_parameter("camera.mode").as_int();
        
        auto dims = CAMERA_MODES.at(mode);
        width = dims.first;
        height = dims.second;

        cam.open(0, cv::CAP_V4L2);
        cam.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cam.set(cv::CAP_PROP_FRAME_HEIGHT, height);

        // ---- CALIBRACION ----
        this->declare_parameter("modo_calibracion", true);
        modo_calibracion = this->get_parameter("modo_calibracion").as_bool();
        
        // ----- DETECTION -----
        this->declare_parameter("detection.min_area", 50);
        this->declare_parameter("detection.target_color_1", "rojo");
        this->declare_parameter("detection.target_color_2", "verde");
        this->declare_parameter("detection.kernel_size", 5);

        min_area = this->get_parameter("detection.min_area").as_int();
        target_color_1 = this->get_parameter("detection.target_color_1").as_string();
        target_color_2 = this->get_parameter("detection.target_color_2").as_string();
        kernel_size = this->get_parameter("detection.kernel_size").as_int();

        // Inicializamos el detector con los parámetros actuales
        color_detector = std::make_unique<ColorDetector>(target_color_1, target_color_2, kernel_size);

        // Posiciones ROI y Tracking
        rx = 0; ry = 0; rw = width; rh = height;
        roi_size = 150;
        current_cx = -1; current_cy = -1; 
        prev_cx = -1; prev_cy = -1;

        // ---- DEBUG ----
        this->declare_parameter("debug", false);
        debug = this->get_parameter("debug").as_bool();
        new_data_available = false;

        // ---- ACTUALIZACION PARAMETROS ----
        callback_handle = this->add_on_set_parameters_callback(
            std::bind(&ImageProcessor::parameters_callback, this, std::placeholders::_1));

        // ---- PUBLISHER ----
        auto qos = rclcpp::SensorDataQoS();
        object_location_publisher = this->create_publisher<image_processor_pkg::msg::ObjectLocation>("/object_position", qos);
        debug_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("camara_debug", qos);

        // ---- TIMER ----
        timer = this->create_wall_timer(33ms, std::bind(&ImageProcessor::process_frame, this), image_processor_group);
        debug_timer = this->create_wall_timer(66ms, std::bind(&ImageProcessor::_tarea_debug, this), debug_group);

        RCLCPP_INFO(this->get_logger(), "Node ImageProcessor Ready (C++)");
    }

private:
    // Callback de parámetros
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : params) {
            if (param.get_name() == "detection.min_area") {
                min_area = param.as_int();
            } else if (param.get_name() == "detection.target_color_1") {
                target_color_1 = param.as_string();
                actualizar_detector();
            } else if (param.get_name() == "detection.target_color_2") {
                target_color_2 = param.as_string();
                actualizar_detector();
            } else if (param.get_name() == "detection.kernel_size") {
                kernel_size = param.as_int();
                actualizar_detector();
            } else if (param.get_name() == "debug") {
                debug = param.as_bool();
            } else if (param.get_name() == "modo_calibracion") {
                modo_calibracion = param.as_bool();
                mascara_trayectoria = generar_mascara();
            }
        }
        return result;
    }

    void actualizar_detector() {
        color_detector = std::make_unique<ColorDetector>(target_color_1, target_color_2, kernel_size);
    }

    cv::Mat generar_mascara() {
        cv::Mat mascara = cv::Mat::zeros(height, width, CV_8UC1);
        if (puntos_trayectoria.size() < 2) return mascara;

        std::vector<cv::Point> puntos;
        for (const auto& p : puntos_trayectoria) {
            puntos.push_back(cv::Point(p.first, p.second));
        }

        const cv::Point* pts[1] = { puntos.data() };
        int npts[] = { static_cast<int>(puntos.size()) };

        cv::polylines(mascara, pts, npts, 1, true, cv::Scalar(255), 15);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
        cv::dilate(mascara, mascara, kernel);
        return mascara;
    }

    void process_frame() {
        cv::Mat frame;
        if (!cam.read(frame)) return;

        if (modo_calibracion) {
            // Se actualiza la llamada según el nuevo color_detector.cpp
            auto points = color_detector->find_object(frame, min_area);
            for (const auto& p : points) {
                puntos_trayectoria.push_back({p.cx, p.cy});
            }
            return;
        }

        // Predicción de ROI
        int pred_x, pred_y;
        if (prev_cx != -1 && current_cx != -1) {
            pred_x = current_cx + (current_cx - prev_cx);
            pred_y = current_cy + (current_cy - prev_cy);
        } else if (current_cx != -1) {
            pred_x = current_cx; pred_y = current_cy;
        } else {
            pred_x = width / 2; pred_y = height / 2;
            roi_size = std::max(width, height);
        }

        int half_roi = roi_size / 2;
        int x1 = std::max(0, pred_x - half_roi);
        int y1 = std::max(0, pred_y - half_roi);
        int x2 = std::min(width, pred_x + half_roi);
        int y2 = std::min(height, pred_y + half_roi);

        if (x2 <= x1 || y2 <= y1) return;

        cv::Rect roi_rect(x1, y1, x2 - x1, y2 - y1);
        cv::Mat roi_frame = frame(roi_rect);
        
        cv::Mat frame_procesar;
        if (!mascara_trayectoria.empty()) {
            cv::Mat mask_roi = mascara_trayectoria(roi_rect);
            cv::bitwise_and(roi_frame, roi_frame, frame_procesar, mask_roi);
        } else {
            frame_procesar = roi_frame.clone();
        }

        auto start_proc = std::chrono::steady_clock::now();
        // Se actualiza la llamada aquí también
        auto points = color_detector->find_object(frame_procesar, min_area);
        auto end_proc = std::chrono::steady_clock::now();

        double proc_duration = std::chrono::duration<double, std::milli>(end_proc - start_proc).count();

        if (!points.empty()) {
            auto p = points[0];
            int global_cx = x1 + p.cx;
            int global_cy = y1 + p.cy;

            prev_cx = current_cx; prev_cy = current_cy;
            current_cx = global_cx; current_cy = global_cy;
            roi_size = 150;

            image_processor_pkg::msg::ObjectLocation msg;
            msg.node_id = node_id;
            msg.color = p.color;
            msg.x = global_cx;
            msg.y = global_cy;
            msg.proc_time = proc_duration;
            msg.stamp = this->get_clock()->now();
            object_location_publisher->publish(msg);
        } else {
            roi_size = std::min(roi_size + 50, std::max(width, height));
            current_cx = -1; prev_cx = -1;
        }

        if (debug && !new_data_available) {
            next_debug_frame = frame_procesar.clone();
            next_debug_points = points; // Aquí usamos el nombre corregido
            new_data_available = true;
        }
    }

    void _tarea_debug() {
        if (!new_data_available) return;

        for (const auto& p : next_debug_points) {
            cv::circle(next_debug_frame, cv::Point(p.cx, p.cy), 5, cv::Scalar(0, 255, 0), -1);
        }

        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
        cv::imencode(".jpg", next_debug_frame, buffer, params);

        sensor_msgs::msg::CompressedImage img_msg;
        img_msg.header.stamp = this->get_clock()->now();
        img_msg.format = "jpeg";
        img_msg.data = buffer;
        debug_publisher->publish(img_msg);

        new_data_available = false;
    }

    // Variables de miembro
    cv::VideoCapture cam;
    int width, height, roi_size;
    int rx, ry, rw, rh;
    int current_cx, current_cy, prev_cx, prev_cy;
    bool modo_calibracion, debug, new_data_available;
    int min_area, kernel_size;
    std::string target_color_1, target_color_2, node_id;

    std::vector<std::pair<int, int>> puntos_trayectoria;
    cv::Mat mascara_trayectoria;
    cv::Mat next_debug_frame;
    
    // CORRECCIÓN: Usar el nombre de estructura correcto definido en el .hpp
    std::vector<DetectedPoint> next_debug_points; 

    std::unique_ptr<ColorDetector> color_detector;

    rclcpp::CallbackGroup::SharedPtr image_processor_group;
    rclcpp::CallbackGroup::SharedPtr debug_group;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr debug_timer;
    rclcpp::Publisher<image_processor_pkg::msg::ObjectLocation>::SharedPtr object_location_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_publisher;
    OnSetParametersCallbackHandle::SharedPtr callback_handle;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessor>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
