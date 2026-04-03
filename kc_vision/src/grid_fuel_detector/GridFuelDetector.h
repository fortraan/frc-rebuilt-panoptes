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

struct Clump {
    int area;
    cv::Rect2i boundingBox;
    cv::Point2d centroid;
};

struct Results {
    // uint8 occupancy grid. cell values are in the range [0, 255], with an empty cell being 0 and a full cell being 255.
    cv::Mat occupancy;
    // uint8 binary occupancy grid. an unoccupied cell is set to 0, and an occupied cell is set to 255.
    cv::Mat binaryOccupancy;
    cv::Mat labels;
    std::vector<Clump> clumps;
};

class GridFuelDetector {
    cv::Size2i gridSize;
    std::vector<Cell> cells;
    bool showDebugDisplays;

    void generateGrid(const Eigen::Isometry3d& gridToCamera, double cellSize,
                      const cv::Size2i& imageSize, const Eigen::Matrix3d& intrinsicMatrix);

    static double processCell(const cv::Mat& filtered, const Cell& cell);

public:
    double occupancyThreshold;
    cv::Scalar hsvLow, hsvHigh;

    GridFuelDetector(const Eigen::Isometry3d& gridToCamera, const cv::Size2i& gridSize, double cellSize,
                     const cv::Size2i& imageSize, const Eigen::Matrix3d& intrinsicMatrix,
                     bool showDebugDisplays);

    [[nodiscard]] Results processFrame(const cv::Mat& frame) const;

    void drawGrid(cv::Mat& mat, const cv::Scalar& color = { 255, 255, 255 }) const;
};
#endif //KC_VISION_GRID_FUEL_DETECTOR_H