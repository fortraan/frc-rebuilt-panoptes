#include "GridFuelDetector.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include "utilities.h"

void GridFuelDetector::generateGrid(const Eigen::Isometry3d& gridToCamera, const double cellSize,
                                    const cv::Size2i& imageSize, const Eigen::Matrix3d& intrinsicMatrix) {
    const Eigen::Matrix<double, 3, 4> offset {
        { 0, 1, 1, 0 },
        { 0, 0, 1, 1 },
        { 0, 0, 0, 0 }
    };

    cv::Mat maskBuffer(imageSize, CV_8UC1);
    const cv::Rect2i imageBounds({ 0, 0 }, imageSize);

    cells.reserve(gridSize.area());
    // todo allow cells to clip outside the frame
    for (int row = 0; row < gridSize.height; row++) {
        const double y = row * cellSize;
        for (int col = 0; col < gridSize.width; col++) {
            const double x = col * cellSize;

            // compute the positions of the corners in grid space
            const Eigen::Vector3d t(x, y, 0);
            const Eigen::Matrix<double, 3, 4> points = t.replicate(1, 4) + cellSize * offset;
            // transform and project the points into image space
            const Eigen::Matrix<double, 3, 4> transformed = gridToCamera * points;
            const Eigen::Matrix<double, 3, 4> projected = intrinsicMatrix * transformed;

            std::array<cv::Point2i, 4> projectedPoints;
            for (int i = 0; i < 4; i++) {
                // extract and normalize point
                const Eigen::Vector2d xy = projected(Eigen::seq(0, 1), i) / projected(2, i);
                projectedPoints[i] = { static_cast<int>(xy.x()), static_cast<int>(xy.y()) };
            }

            // compute bounding box of the project points
            const auto roi = cv::boundingRect(projectedPoints);
            // compute intersection of image bounds and polygon bounding box
            const auto clippedRoI = roi & imageBounds;

            // discard the cell if it's completely outside the image
            if (clippedRoI.empty()) {
                // todo log a warning
                continue;
            }

            // clear the mask buffer. yes, it would be more efficient to only clear the area we draw over,
            // but that adds some complexity depending on the rounding behavior of cv::boundingRect. we
            // only need to generate the cells once, so this doesn't need to be super optimized anyway.
            maskBuffer.setTo(0);
            // draw the mask in the buffer
            cv::fillPoly(maskBuffer, projectedPoints, { 255 });
            // copy the roi to a mat
            cv::Mat mask(clippedRoI.size(), maskBuffer.type());
            maskBuffer(clippedRoI).copyTo(mask);

            cells.emplace_back(Cell {
                .x = col,
                .y = row,
                .points = projectedPoints,
                .roi = clippedRoI,
                .mask = std::move(mask)
            });
        }
    }
}

GridFuelDetector::GridFuelDetector(const Eigen::Isometry3d& gridToCamera, const cv::Size2i& gridSize, const double cellSize,
                                   const cv::Size2i& imageSize, const Eigen::Matrix3d& intrinsicMatrix,
                                   const bool showDebugDisplays) :
    gridSize(gridSize), showDebugDisplays(showDebugDisplays),
    occupancyThreshold(1), hsvLow(180, 255, 255), hsvHigh(0, 0, 0)
{
    generateGrid(gridToCamera, cellSize, imageSize, intrinsicMatrix);
}

Results GridFuelDetector::processFrame(const cv::Mat& frame) const {
    // todo: reuse buffers
    // hsv: CV_8UC3, filtered: CV_8UC1 - false: 0, true: 255
    cv::Mat hsv, filtered;
    cv::cvtColor(frame, hsv, cv::COLOR_RGB2HSV);
    cv::inRange(hsv, hsvLow, hsvHigh, filtered);
    cv::Mat grid(gridSize, CV_64FC1, cv::Scalar(0));
    for (const auto& cell : cells) {
        grid.at<double>(cell.y, cell.x) = processCell(filtered, cell);
    }
    // cv::threshold will output the same type as what it's given as input, so convert to 8U
    cv::Mat quantizedGrid(gridSize, CV_8U);
    grid.convertTo(quantizedGrid, quantizedGrid.type());
    cv::Mat binaryGrid(gridSize, CV_8U);
    cv::threshold(quantizedGrid, binaryGrid, 255.0 * occupancyThreshold, 255, cv::THRESH_BINARY);

    cv::Mat labels(gridSize, CV_16U);
    cv::Mat stats, centroids;
    int numClumps = cv::connectedComponentsWithStats(
        binaryGrid, labels, stats, centroids, 4, labels.type()
    );

    KC_DEBUG_ASSERT(stats.type() == CV_32S, "stats is the wrong type!");
    KC_DEBUG_ASSERT(centroids.type() == CV_64F, "centroids is the wrong type!");
    std::vector<Clump> clumps;
    clumps.reserve(numClumps);
    for (int i = 0; i < numClumps; i++) {
        using enum cv::ConnectedComponentsTypes;
        clumps.emplace_back(
            stats.at<int32_t>(i, CC_STAT_AREA),
            cv::Rect2i(
                stats.at<int32_t>(i, CC_STAT_LEFT),
                stats.at<int32_t>(i, CC_STAT_TOP),
                stats.at<int32_t>(i, CC_STAT_WIDTH),
                stats.at<int32_t>(i, CC_STAT_HEIGHT)
            ),
            cv::Point2d(
                centroids.at<double>(i, 0),
                centroids.at<double>(i, 1)
            )
        );
    }

    if (showDebugDisplays) {
        cv::Mat drawBuffer;
        cv::cvtColor(frame, drawBuffer, cv::COLOR_RGB2BGR);
        drawGrid(drawBuffer);
        cv::imshow("Grid (Color)", drawBuffer);
        cv::waitKey(1);

        cv::cvtColor(filtered, drawBuffer, cv::COLOR_GRAY2BGR);
        drawGrid(drawBuffer, { 255, 0, 0 });
        cv::imshow("Grid (Binary)", drawBuffer);
        cv::waitKey(1);
    }

    return {
        std::move(grid),
        std::move(binaryGrid),
        std::move(labels),
        std::move(clumps)
    };
}

void GridFuelDetector::drawGrid(cv::Mat& mat, const cv::Scalar& color) const {
    for (const auto& cell : cells) {
        cv::polylines(mat, cell.points, true, color, 2);
    }
}

double GridFuelDetector::processCell(const cv::Mat& filtered, const Cell& cell) {
    KC_DEBUG_ASSERT(filtered.channels() == 1, "filtered should be a binary mask!");
    KC_DEBUG_ASSERT(cell.mask.size() == cell.roi.size(), "cell ROI is not the same size as the cell mask!");
    return cv::mean(filtered(cell.roi), cell.mask)[0];
}
