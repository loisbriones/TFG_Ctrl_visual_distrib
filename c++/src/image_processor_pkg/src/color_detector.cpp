#include "image_processor_pkg/color_detector.hpp"

ColorDetector::ColorDetector(std::string target_1, std::string target_2, int kernel_size) {
    this->kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));

    // Definir rangos (H, S, V)
    color_ranges["rojo"] = { {cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255)}, {cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255)} };
    color_ranges["naranja"] = { {cv::Scalar(11, 100, 100), cv::Scalar(25, 255, 255)} };
    color_ranges["amarillo"] = { {cv::Scalar(26, 100, 100), cv::Scalar(34, 255, 255)} };
    color_ranges["verde"] = { {cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255)} };
    color_ranges["cian"] = { {cv::Scalar(86, 100, 100), cv::Scalar(100, 255, 255)} };
    color_ranges["azul"] = { {cv::Scalar(101, 100, 100), cv::Scalar(130, 255, 255)} };
    color_ranges["violeta"] = { {cv::Scalar(120, 50, 50), cv::Scalar(150, 255, 255)} };
}

void ColorDetector::get_mask_for_color(const cv::Mat& hsv_frame, const std::string& color_name, cv::Mat& mask) {
    mask = cv::Mat::zeros(hsv_frame.size(), CV_8UC1);
    for (const auto& range : color_ranges[color_name]) {
        cv::Mat temp_mask;
        cv::inRange(hsv_frame, range.first, range.second, temp_mask);
        cv::bitwise_or(mask, temp_mask, mask);
    }
}

std::vector<DetectedPoint> ColorDetector::find_object(cv::Mat frame, double min_area, std::string target_1, std::string target_2) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2;
    get_mask_for_color(hsv, target_1, mask1);
    get_mask_for_color(hsv, target_2, mask2);

    auto points = detectar(mask1, target_1, min_area);
    auto points2 = detectar(mask2, target_2, min_area);
    points.insert(points.end(), points2.begin(), points2.end());

    return points;
}

std::vector<DetectedPoint> ColorDetector::detectar(cv::Mat mask, std::string color_name, double min_area) {
    std::vector<DetectedPoint> puntos;
    cv::Mat mask_limpia;
    cv::morphologyEx(mask, mask_limpia, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_limpia, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& c : contours) {
        if (cv::contourArea(c) > min_area) {
            cv::Rect r = cv::boundingRect(c);
            int cx = r.x + (r.width / 2);
            int cy = r.y + (r.height / 2);
            puntos.push_back({color_name, cx, cy});
        }
    }
    return puntos;
}