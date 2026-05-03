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
        
        // Coger modo camara para saber w,h del frame
        auto dims = CAMERA_MODES.at(mode);
        width = dims.first;
        height = dims.second;
        
        // Seleccionamos la camara
        cam.open(0, cv::CAP_V4L2);
        // Configuramos el ancho de la camara
        cam.set(cv::CAP_PROP_FRAME_WIDTH, width);
        // Configuramos el alto de la camara
        cam.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        // Desactivamos el autoenfoque de la camara
        cam.set(cv::CAP_PROP_AUTOFOCUS, 0);

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

        // Inicializamos el detector, clase con la logica de deteccion 
        color_detector = std::make_unique<ColorDetector>(target_color_1, target_color_2, kernel_size);

        // Posiciones ROI
        rx = 0; ry = 0; rw = width; rh = height;
        // Tamaño del ROI
        roi_size = 150;
        // Posicion en un momento concreto de la esquina
        current_cx = -1; current_cy = -1; 
        prev_cx = -1; prev_cy = -1;

        // ---- DEBUG ----
        this->declare_parameter("debug", false);
        debug = this->get_parameter("debug").as_bool();
        new_data_available = false;
        debug_x = 0;
        debug_y = 0;

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
        // Crear lienzo negro
        cv::Mat mascara = cv::Mat::zeros(height, width, CV_8UC1);
        if (puntos_trayectoria.size() < 2) return mascara;

        std::vector<cv::Point> puntos;
        for (const auto& p : puntos_trayectoria) {
            puntos.push_back(cv::Point(p.first, p.second));
        }
        
        const cv::Point* pts[1] = { puntos.data() };
        int npts[] = { static_cast<int>(puntos.size()) };
        
        // Dibuyjar la trayectoria uniendo los puntos con lineas blancas
        cv::polylines(mascara, pts, npts, 1, true, cv::Scalar(255), 15);

        // Creamos el kernel
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
        // Cierre morfologico para eliminar pequeños puntos negros que puedan quedar fruto de no detectar nada 
        cv::morphologyEx(mascara, mascara, cv::MORPH_CLOSE, kernel_mask);
        // Dilatiacion expandimos los bordes de la mascara hacia fuera aumenta el area de la mascara
        cv::dilate(mascara, mascara, kernel);

        return mascara;
    }

    void process_frame() {
        cv::Mat frame;
        if (!cam.read(frame)) return;

        if (modo_calibracion) {
            // Buscamos en todo el frame para detectar el coche
            auto points = color_detector->find_object(frame, min_area);
            
            // Guardamos la trayectoria para despues generar la mascara
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
            // Si no detectamos nada buscamos en todo el frame 
            pred_x = width / 2; pred_y = height / 2;
            roi_size = std::max(width, height);
        }

        int half_roi = roi_size / 2;
        // Calcular las coordenadas de la esquina superior izquierda del ROI 
        int x1 = std::max(0, pred_x - half_roi);
        int y1 = std::max(0, pred_y - half_roi);
        // Calcular las coordenadas de la esquina inferior derecha del ROI
        int x2 = std::min(width, pred_x + half_roi);
        int y2 = std::min(height, pred_y + half_roi);

        cv::Rect roi_rect(x1, y1, x2 - x1, y2 - y1);
        cv::Mat roi_frame = frame(roi_rect);
        
        cv::Mat frame_procesar;

        if (!mascara_trayectoria.empty()) {
            // Aplicamos la mascara sobre la zona del frame a la que aplicamos el ROI
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
            // Traducir las coordenadas locales del ROI a globales del Frame
            int global_cx = x1 + p.cx;
            int global_cy = y1 + p.cy;
            
            // Actualizar el estado para la siguiente iteracion
            prev_cx = current_cx; prev_cy = current_cy;
            current_cx = global_cx; current_cy = global_cy;
            roi_size = 150;

            // 2. Bucle para publicar TODOS los puntos detectados
            for (const auto& p : points) {
                image_processor_pkg::msg::ObjectLocation msg;
                msg.node_id = node_id;
                msg.color = p.color;
                msg.x = x1 + p.cx; 
                msg.y = y1 + p.cy; 
                msg.proc_time = proc_duration;
                msg.stamp = this->get_clock()->now();
                
                object_location_publisher->publish(msg);
            }

        } else {
            // Si no detectamos nada actualizamos area de busqueda para el proximo frame 
            roi_size = std::min(roi_size + 50, std::max(width, height));
            current_cx = -1; prev_cx = -1;
        }

        if (debug && !new_data_available) {
            next_debug_frame = frame.clone();
            debug_x = x1;
            debug_y = y1;
            next_debug_points = points; // Aquí usamos el nombre corregido
            new_data_available = true;
        }
    }

    void _tarea_debug() {
        if (!new_data_available) return;

        for (const auto& p : next_debug_points) {
            cv::circle(next_debug_frame, cv::Point(debug_x + p.cx, debug_y + p.cy), 5, cv::Scalar(0, 255, 255), -1);
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

    cv::VideoCapture cam;
    int width, height, roi_size;
    int rx, ry, rw, rh;
    int current_cx, current_cy, prev_cx, prev_cy;
    bool modo_calibracion, debug, new_data_available;
    int min_area, kernel_size;
    int debug_x,debug_y;
    std::string target_color_1, target_color_2, node_id;

    std::vector<std::pair<int, int>> puntos_trayectoria;
    cv::Mat mascara_trayectoria;
    cv::Mat next_debug_frame;
    
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
