#include "GridFuelDetector.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "utilities.h"

void GridFuelDetector::generateGrid(const Eigen::Isometry3d& gridToCamera, const double cellSize,
                                    const cv::Size2i& imageSize, const Eigen::Matrix3d& intrinsicMatrix) {
    const Eigen::Matrix<double, 3, 4> offset {
        { 0, 1, 1, 0 },
        { 0, 0, 1, 1 },
        { 0, 0, 0, 0 }
    };

    cv::Mat maskBuffer(imageSize, CV_8UC1);

    cells.reserve(gridSize.height);
    // todo allow cells to clip outside the frame
    for (int row = 0; row < gridSize.height; row++) {
        std::vector<Cell> rowVector;
        rowVector.reserve(gridSize.width);

        const double y = row * cellSize;
        for (int col = 0; col < gridSize.width; col++) {
            const double x = col * cellSize;

            // compute the positions of the corners in grid space
            const Eigen::Vector3d t(x, y, 0);
            Eigen::Matrix<double, 3, 4> points = t.replicate(1, 4) + cellSize * offset;
            // transform and project the points into image space
            Eigen::Matrix<double, 3, 4> projected = intrinsicMatrix * gridToCamera * points;

            std::array<cv::Point2d, 4> projectedPoints;
            for (int i = 0; i < 4; i++) {
                // extract and normalize point
                const Eigen::Vector2d xy = projected(Eigen::seq(0, 1), i) / projected(2, i);
                projectedPoints[i] = { xy.x(), xy.y() };
            }

            const auto roi = cv::boundingRect(projectedPoints);

            // clear the mask buffer. yes, it would be more efficient to only clear the area we draw over,
            // but that adds some complexity depending on the rounding behavior of cv::boundingRect. we
            // only need to generate the cells once, so this doesn't need to be super optimized anyway.
            maskBuffer.setTo(0);
            // draw the mask in the buffer
            cv::fillConvexPoly(maskBuffer, projectedPoints, { 255 });
            // copy the roi to a mat
            cv::Mat mask;
            maskBuffer(roi).copyTo(mask);

            rowVector.emplace_back(Cell {
                .points = projectedPoints,
                .roi = roi,
                .mask = std::move(mask)
            });
        }
        // now that we've filled the row vector, move it into cells
        cells.emplace_back(std::move(rowVector));
    }
}

GridFuelDetector::GridFuelDetector(const Eigen::Isometry3d& gridToCamera, const cv::Size2i& gridSize, const double cellSize,
                                   const cv::Size2i& imageSize, const Eigen::Matrix3d& intrinsicMatrix,
                                   const double occupancyThreshold, const cv::Scalar& hsvLow, const cv::Scalar& hsvHigh) :
    gridSize(gridSize), occupancyThreshold(occupancyThreshold), hsvLow(hsvLow), hsvHigh(hsvHigh)
{
    generateGrid(gridToCamera, cellSize, imageSize, intrinsicMatrix);
}

Results GridFuelDetector::processFrame(const cv::Mat& frame) const {
    cv::Mat hsv, filtered;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsvLow, hsvHigh, filtered);
    cv::Mat grid(gridSize, CV_64FC1);
    for (int y = 0; y < gridSize.height; y++) {
        for (int x = 0; x < gridSize.width; x++) {
            grid.at<double>(y, x) = processCell(filtered, cells[y][x]);
        }
    }
    cv::Mat binaryGrid(gridSize, CV_8UC1);
    cv::threshold(grid, binaryGrid, occupancyThreshold, 1, cv::THRESH_BINARY);
    cv::Mat labels(gridSize, CV_32SC1);
    cv::Mat stats, centroids;
    cv::connectedComponentsWithStats(binaryGrid, labels, stats, centroids);
    return {
        std::move(labels),
        std::move(stats),
        std::move(centroids)
    };
}

double GridFuelDetector::processCell(const cv::Mat& filtered, const Cell& cell) {
    KC_DEBUG_ASSERT(filtered.channels() == 1, "filtered should be a binary mask!");
    KC_DEBUG_ASSERT(cell.mask.size() == cell.roi.size(), "cell ROI is not the same size as the cell mask!");
    return cv::mean(filtered(cell.roi), cell.mask)[0];
}
