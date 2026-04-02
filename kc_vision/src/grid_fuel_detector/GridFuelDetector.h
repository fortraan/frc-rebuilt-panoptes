#ifndef KC_VISION_GRID_FUEL_DETECTOR_H
#define KC_VISION_GRID_FUEL_DETECTOR_H

#include <array>
#include <vector>

#include <opencv2/core.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct Cell {
    int x, y;
    std::array<cv::Point2i, 4> points;
    cv::Rect2i roi;
    cv::Mat mask;
};

struct Results {
    // uint8 occupancy grid. cell values are in the range [0, 255], with an empty cell being 0 and a full cell being 255.
    cv::Mat occupancy;
    // uint8 binary occupancy grid. an unoccupied cell is set to 0, and an occupied cell is set to 255.
    cv::Mat binaryOccupancy;
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
};

class GridFuelDetector {
    cv::Size2i gridSize;
    std::vector<Cell> cells;

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

    void drawGrid(cv::Mat& mat) const;
};
#endif //KC_VISION_GRID_FUEL_DETECTOR_H