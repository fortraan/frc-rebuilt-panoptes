#include <map>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <kc_vision_msgs/msg/clumps.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {
    /**
     * Converts an HSV color to its RGBA equivalent.
     * @param h Hue in the range [0, 360). 0 is red, 120 is green, 240 is blue.
     * @param s Saturation in the range [0, 1].
     * @param v Value in the range [0, 1].
     * @return ROS2 std_msgs/ColorRGBA message with alpha set to 1.0.
     */
    std_msgs::msg::ColorRGBA hsvToRgba(double h, const double s, const double v) {
        double r = 0, g = 0, b = 0;

        if (s <= 0.0) {
            r = g = b = v;
        } else {
            h = std::fmod(h, 360.0);
            if (h < 0) h += 360.0;
            h /= 60.0;
            const int i = static_cast<int>(h);
            const double f = h - i;
            const double p = v * (1.0 - s);
            const double q = v * (1.0 - s * f);
            const double t = v * (1.0 - s * (1.0 - f));

            switch (i % 6) {
                case 0: r = v; g = t; b = p; break;
                case 1: r = q; g = v; b = p; break;
                case 2: r = p; g = v; b = t; break;
                case 3: r = p; g = q; b = v; break;
                case 4: r = t; g = p; b = v; break;
                case 5: r = v; g = p; b = q; break;
                default: r = g = b = 0; break;
            }
        }

        std_msgs::msg::ColorRGBA color;
        color.r = static_cast<float>(r);
        color.g = static_cast<float>(g);
        color.b = static_cast<float>(b);
        color.a = 1.0f;
        return color;
    }
}

class GridFuelVisualizer : public rclcpp::Node {
    std::string markerNamespace;

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> markerPublisher;

    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>> labelsSubscription;
    std::shared_ptr<rclcpp::Subscription<kc_vision_msgs::msg::Clumps>> clumpsSubscription;

    std::mutex mutex;
    std::shared_ptr<const kc_vision_msgs::msg::Clumps> clumpsMsg;

    visualization_msgs::msg::Marker visualizeLabels(const std::shared_ptr<const nav_msgs::msg::OccupancyGrid>& labels) const {
        visualization_msgs::msg::Marker marker;

        marker.header = labels->header;
        marker.ns = markerNamespace;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::MODIFY;
        marker.pose.position.z = 0.05;
        marker.scale.x = marker.scale.y = 0.8 * labels->info.resolution;
        marker.scale.z = 0.05;
        marker.frame_locked = false;
        using namespace std::chrono_literals;
        marker.lifetime = rclcpp::Duration(500ms);

        const auto numClumps = clumpsMsg->clumps.size();

        if (numClumps > 0) {
            std::vector<std::pair<uint16_t, int>> labelsAndAreas;
            labelsAndAreas.reserve(numClumps);
            std::ranges::transform(clumpsMsg->clumps, std::back_inserter(labelsAndAreas), [](const auto& clump) {
                return std::make_pair(clump.label, clump.area);
            });
            // sort by area in descending order
            std::ranges::sort(labelsAndAreas, std::ranges::greater(), [](const auto& pair) {
                return pair.second;
            });

            std::unordered_map<uint16_t, std_msgs::msg::ColorRGBA> colorMap;
            colorMap.reserve(numClumps);
            long numTotalCells = 0;
            for (size_t i = 0; i < numClumps; i++) {
                const auto& [label, area] = labelsAndAreas[i];
                constexpr double HUE_GREEN = 120, HUE_RED = 0;
                double hue;
                if (numClumps == 1) hue = HUE_GREEN;
                else hue = std::lerp(HUE_GREEN, HUE_RED, static_cast<double>(i) / static_cast<double>(numClumps - 1));
                colorMap.emplace(label, hsvToRgba(hue, 1, 1));
                numTotalCells += area;
            }

            marker.points.reserve(numTotalCells);
            marker.colors.reserve(numTotalCells);
            for (uint32_t y = 0; y < labels->info.height; y++) {
                for (uint32_t x = 0; x < labels->info.width; x++) {
                    const int8_t label = labels->data[y * labels->info.width + x];
                    if (colorMap.contains(label)) {
                        geometry_msgs::msg::Point point;
                        point.x = labels->info.resolution * (0.5 + static_cast<double>(x));
                        point.y = labels->info.resolution * (0.5 + static_cast<double>(y));
                        marker.points.push_back(point);
                        marker.colors.push_back(colorMap[label]);
                    }
                }
            }
        }

        return marker;
    }

    visualization_msgs::msg::Marker visualizeClumps(const std::shared_ptr<const nav_msgs::msg::OccupancyGrid>& labels) const {
        visualization_msgs::msg::Marker marker;

        marker.header = labels->header;
        marker.ns = markerNamespace;
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::MODIFY;
        constexpr auto FUEL_DIAMETER = 0.15;
        marker.pose.position.z = 0.5 * FUEL_DIAMETER + 0.2;
        marker.scale.x = marker.scale.y = marker.scale.z = FUEL_DIAMETER;
        marker.frame_locked = false;
        using namespace std::chrono_literals;
        marker.lifetime = rclcpp::Duration(500ms);
        marker.color.r = 252.0 / 255.0;
        marker.color.g = 198.0 / 255.0;
        marker.color.b =   3.0 / 255.0;
        marker.color.a = 1;

        marker.points.reserve(clumpsMsg->clumps.size());
        for (const auto& clump : clumpsMsg->clumps) {
            geometry_msgs::msg::Point point;
            point.x = clump.centroid.x;
            point.y = clump.centroid.y;
            marker.points.push_back(point);
        }

        return marker;
    }

    void onLabelsReceived(const std::shared_ptr<const nav_msgs::msg::OccupancyGrid>& labels) {
        std::lock_guard lock(mutex);
        visualization_msgs::msg::MarkerArray markers;
        markers.markers.emplace_back(visualizeLabels(labels));
        markers.markers.emplace_back(visualizeClumps(labels));
        markerPublisher->publish(markers);
    }

    void onClumpsReceived(const std::shared_ptr<const kc_vision_msgs::msg::Clumps>& msg) {
        std::lock_guard lock(mutex);
        clumpsMsg = msg;
    }

public:
    GridFuelVisualizer() : Node("grid_fuel_visualizer") {
        markerNamespace = declare_parameter<std::string>("marker_namespace", get_namespace());
        markerPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("/markers_array", 1);
        labelsSubscription = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "labels", rclcpp::SensorDataQoS(),
            [this](std::shared_ptr<const nav_msgs::msg::OccupancyGrid> msg) {
                onLabelsReceived(msg);
            }
        );
        clumpsSubscription = create_subscription<kc_vision_msgs::msg::Clumps>(
            "clumps", rclcpp::SensorDataQoS(),
            [this](std::shared_ptr<const kc_vision_msgs::msg::Clumps> msg) {
                onClumpsReceived(msg);
            }
        );
    }
};

int main(const int argc, const char* const argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<GridFuelVisualizer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}