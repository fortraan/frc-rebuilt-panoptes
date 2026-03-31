#ifndef KC_VISION_GRID_FUEL_DETECTOR_H
#define KC_VISION_GRID_FUEL_DETECTOR_H

#include <array>
#include <vector>

#include <opencv2/core/mat.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct Cell {
    std::array<cv::Point2d, 4> points;
    cv::Rect2i roi;
    cv::Mat mask;
};

struct Results {
    cv::Mat occupancy;
    cv::Mat binaryOccupancy;
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
};

class GridFuelDetector {
    cv::Size2i gridSize;
    std::vector<std::vector<Cell>> cells; // stored row-major: [column][row]

    void generateGrid(const Eigen::Isometry3d& gridToCamera, double cellSize,
                      const cv::Size2i& imageSize, const Eigen::Matrix3d& intrinsicMatrix);

    static double processCell(const cv::Mat& filtered, const Cell& cell);

public:
    double occupancyThreshold;
    cv::Scalar hsvLow, hsvHigh;

    GridFuelDetector(const Eigen::Isometry3d& gridToCamera, const cv::Size2i& gridSize, double cellSize,
                     const cv::Size2i& imageSize, const Eigen::Matrix3d& intrinsicMatrix,
                     double occupancyThreshold, const cv::Scalar& hsvLow, const cv::Scalar& hsvHigh);

    Results processFrame(const cv::Mat& frame) const;

};
#endif //KC_VISION_GRID_FUEL_DETECTOR_H