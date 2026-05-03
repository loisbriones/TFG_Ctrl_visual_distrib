#include "image_processor_pkg/color_detector.hpp"

ColorDetector::ColorDetector(std::string target_1, std::string target_2, int kernel_size) {
    // Configuramos el kernel que vamos a usar para detectar los colores
    this->kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    // Establecemos el primer color que queremos buscar
    this->t1 = target_1;
    // Establecemos el segundo color que queremos buscar
    this->t2 = target_2;

    color_ranges["rojo"] = { {cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255)}, {cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255)} };
    color_ranges["naranja"] = { {cv::Scalar(11, 100, 100), cv::Scalar(25, 255, 255)} };
    color_ranges["amarillo"] = { {cv::Scalar(25, 100, 150), cv::Scalar(35, 255, 255)} };
    color_ranges["verde"] = { {cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255)} };
    color_ranges["cian"] = { {cv::Scalar(75, 40, 30), cv::Scalar(105, 255, 255)} };
    color_ranges["azul"] = { {cv::Scalar(95, 100, 40), cv::Scalar(130, 255, 255)} };
    color_ranges["violeta"] = { {cv::Scalar(120, 50, 50), cv::Scalar(150, 255, 255)} };
}

void ColorDetector::get_mask_for_color(const cv::Mat& hsv_frame, const std::string& color_name, cv::Mat& mask) {

    mask = cv::Mat::zeros(hsv_frame.size(), CV_8UC1);
    if (color_ranges.find(color_name) == color_ranges.end()) return;

    for (const auto& range : color_ranges[color_name]) {
        cv::Mat temp_mask;
        // Filtramos la imagen para detectar pixeles dentro del rango de color
        cv::inRange(hsv_frame, range.first, range.second, temp_mask);
        cv::bitwise_or(mask, temp_mask, mask);
    }
}

std::vector<DetectedPoint> ColorDetector::find_object(cv::Mat frame, double min_area) {
    cv::Mat hsv;
    // Convertimos el frame que tenemos que procesar a HSV
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2;
    // Generamos una mascara donde solo van pixeles que tienen target_color_1
    get_mask_for_color(hsv, this->t1, mask1);
    get_mask_for_color(hsv, this->t2, mask2);

    // Buscamos los objetos dentro de la mascara para target_color_1
    auto points = detectar(mask1, this->t1, min_area);
    // Buscamos los objetos dentro de la mascara para target_color_2
    auto points2 = detectar(mask2, this->t2, min_area);
    points.insert(points.end(), points2.begin(), points2.end());

    return points;
}

std::vector<DetectedPoint> ColorDetector::detectar(cv::Mat mask, std::string color_name, double min_area) {
    std::vector<DetectedPoint> puntos;
    cv::Mat mask_limpia;
    // Aplicamos un cierre morfologico para eliminar ruido o pequeñas imprecisiones
    cv::morphologyEx(mask, mask_limpia, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    // Lista de puntos que representa el contorno exterior edl objeto que acabamos de detectar
    cv::findContours(mask_limpia, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& c : contours) {
        if (cv::contourArea(c) > min_area) {
            // Devuelve 4 puntos que representan un rectangulo qe envuelve el contorno detectado
            cv::Rect r = cv::boundingRect(c);
            int cx = r.x + (r.width / 2);
            int cy = r.y + (r.height / 2);
            puntos.push_back({color_name, cx, cy});
        }
    }
    return puntos;
}
