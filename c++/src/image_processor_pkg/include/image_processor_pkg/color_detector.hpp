#ifndef COLOR_DETECTOR_HPP
#define COLOR_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>

struct DetectedPoint {
    std::string color;
    int cx;
    int cy;
};

class ColorDetector {
public:
    ColorDetector(std::string target_1, std::string target_2, int kernel_size);
    std::vector<DetectedPoint> find_object(cv::Mat frame, double min_area, std::string target_1, std::string target_2);

private:
    cv::Mat kernel;
    std::map<std::string, std::vector<std::pair<cv::Scalar, cv::Scalar>>> color_ranges;
    
    void get_mask_for_color(const cv::Mat& hsv_frame, const std::string& color_name, cv::Mat& mask);
    std::vector<DetectedPoint> detectar(cv::Mat mask, std::string color_name, double min_area);
};

#endif