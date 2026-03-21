// APeCs: Apriltag PErspective ConSensus

#include "apecs.h"

#include <algorithm>
#include <cstdint>
#include <list>
#include <numeric>

#include "utilities.h"

using Pose = geometry_msgs::msg::Pose;
using Point = geometry_msgs::msg::Point;
using Quaternion = geometry_msgs::msg::Quaternion;

namespace {
    template <typename T>
    T add(T a, T b) {
        return a + b;
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> positionStats(const std::vector<const Point*>& positions,
                                                              const std::vector<double>& weights = { }) {
        KC_DEBUG_ASSERT(
            weights.empty() || weights.size() == positions.size(),
            "positionStats: # positions != # weights!"
        );
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const auto* position : positions) {
            mean += Eigen::Vector3d(position->x, position->y, position->z);
        }
        mean /= static_cast<double>(positions.size());

        Eigen::Vector3d variance = Eigen::Vector3d::Zero();
        for (const auto* position : positions) {
            variance += (Eigen::Vector3d(position->x, position->y, position->z) - mean).array().square().matrix();
        }
        if (!weights.empty()) {
            const double weightSum = std::reduce(weights.begin(), weights.end(), 0, add<double>);
            variance /= weightSum;
        } else {
            variance /= static_cast<double>(positions.size());
        }

        return {
            mean,
            variance
        };
    }

    // references:
    // https://www.acsu.buffalo.edu/%7Ejohnc/ave_quat07.pdf
    // https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
    Quaternion quatAverage(const std::vector<const Quaternion*>& quaternions,
                           const std::vector<double>& weights = { }) {
        if (quaternions.empty()) {
            Quaternion ret;
            ret.x = ret.y = ret.z = 0;
            ret.w = 1;
            return ret;
        }

        KC_DEBUG_ASSERT(quaternions.size() == weights.size(), "quatAverage: # quaternions != # weights!");

        const auto numQuaternions = static_cast<Eigen::Index>(quaternions.size());
        Eigen::Matrix4Xd q = Eigen::Matrix4Xd::Zero(4,  numQuaternions);
        for (Eigen::Index i = 0; i < numQuaternions; i++) {
            const auto* quaternion = quaternions[i];
            q(0, i) = quaternion->x;
            q(1, i) = quaternion->y;
            q(2, i) = quaternion->z;
            q(3, i) = quaternion->w;
            if (!weights.empty()) {
                q.col(i) *= weights[i];
            }
        }

        // todo it may be possible to derive covariance from the second moment
        const Eigen::MatrixXd secondMoment = q * q.transpose();
        const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(secondMoment);
        Eigen::Index maxIndex;
        solver.eigenvalues().maxCoeff(&maxIndex);
        const Eigen::Vector4d eigenVector = solver.eigenvectors().col(maxIndex);

        Quaternion ret;
        ret.x = eigenVector(0);
        ret.y = eigenVector(1);
        ret.z = eigenVector(2);
        ret.w = eigenVector(3);
        return ret;
    }

    rclcpp::Time timeAverage(const std::vector<rclcpp::Time>& times,
                             const std::vector<double>& weights = { }) {
        KC_DEBUG_ASSERT(weights.empty() || weights.size() == times.size(), "# times != # weights!");
        int64_t secondsSum = 0, nanosecondsSum = 0;
        for (const auto& time : times) {
            const auto secondsDouble = time.seconds();
            const auto secondsInt = static_cast<int64_t>(std::trunc(secondsDouble));
            const int64_t nanoseconds = time.nanoseconds() - RCUTILS_S_TO_NS(secondsInt);
            secondsSum += secondsInt;
            nanosecondsSum += nanoseconds;
        }
        // because we're basically doing fixed-point math here, seconds need to be carried from
        // nanosecondsSum to secondsSum
        const int64_t extraSeconds = RCUTILS_NS_TO_S(nanosecondsSum);
        secondsSum += extraSeconds;
        nanosecondsSum -= RCUTILS_S_TO_NS(extraSeconds);

        const auto numTimes = static_cast<double>(times.size());
        return {
            static_cast<int32_t>(static_cast<double>(secondsSum) / numTimes),
            static_cast<uint32_t>(static_cast<double>(nanosecondsSum) / numTimes)
        };
    }

    Model fitModel(const std::vector<std::pair<rclcpp::Time, const Pose*>>& transforms,
                   const std::vector<double>& weights = { }) {
        KC_DEBUG_ASSERT(weights.size() == 0 || weights.size() == transforms.size(), "# weights != # transforms!");

        std::vector<rclcpp::Time> times;
        std::vector<const Point*> positions;
        std::vector<const Quaternion*> rotations;
        times.reserve(transforms.size());
        positions.reserve(transforms.size());
        rotations.reserve(transforms.size());
        for (const auto& [time, pose] : transforms) {
            times.emplace_back(time);
            positions.push_back(&pose->position);
            rotations.push_back(&pose->orientation);
        }
        
        const auto timeMean = timeAverage(times);
        const auto [positionMean, positionVariance] = positionStats(positions, weights);
        const auto rotationMean = quatAverage(rotations, weights);
        
        Model ret;
        ret.mean.position.x = positionMean[0];
        ret.mean.position.y = positionMean[1];
        ret.mean.position.z = positionMean[2];
        ret.mean.orientation = rotationMean;
        std::copy_n(positionVariance.begin(), 3, ret.variance.begin());
        std::fill_n(ret.variance.begin() + 3, 3, 0); // todo rpy variance
        return ret;
    }

    Model apecsTwoPlus(const std::vector<const Observation*>& observations) {
        const auto predicate = [](const Observation* observation) {
            return observation->secondaryEstimate.has_value();
        };
        std::vector<const Observation*> singlets, duals;
        std::ranges::copy_if(observations, std::back_inserter(singlets), predicate);
        std::ranges::copy_if(observations, std::back_inserter(duals), std::not_fn(predicate));

        std::vector<std::pair<rclcpp::Time, const Pose*>> singletPairs;
        for (const auto* singlet : singlets) {
            singletPairs.emplace_back(singlet->time, &singlet->primaryEstimate);
        }
        
        // precompute average of singlet observation
        const auto singletModel = fitModel(singletPairs);

        const auto numSinglets = singlets.size();
        const auto numDuals = duals.size();

        KC_DEBUG_ASSERT(numDuals <= 64, "Too many duals to fit in combination variable!");

        std::vector<std::pair<rclcpp::Time, const Pose*>> transformCombo;
        transformCombo.reserve(1 + numDuals);

        std::vector<double> weights(1 + numDuals, 1);
        weights[0] = static_cast<double>(numSinglets);

        std::optional<Model> bestModel;

        if (numDuals != 0) {
            const uint64_t numCombinations = 1 << numDuals;
            for (uint64_t combination = 0; combination < numCombinations; combination++) {
                transformCombo.clear();
                transformCombo.emplace_back(singletModel.time, &singletModel.mean);
                for (size_t i = 0; i < numDuals; i++) {
                    transformCombo.emplace_back(
                        duals[i]->time,
                        1 & combination >> i ? &duals[i]->primaryEstimate
                                             : &duals[i]->secondaryEstimate.value()
                    );
                }

                const auto model = fitModel(transformCombo, weights);
                if (!bestModel || model.variance.sum() < bestModel->variance.sum()) {
                    bestModel = model;
                }
            }
        }

        return bestModel.value_or(singletModel);
    }
}

std::optional<Model> apecs(const std::vector<Observation>& observations) {
    // at least one tag is required for any kind of estimate to be derived.
    if (observations.empty()) return std::nullopt;

    if (observations.size() == 1) {
        // one tag is a special case. one of the 2 solutions may have been rejected as an obvious outlier,
        // but if it wasn't we have a dilemma - which one is right? in this case, we have to fall back to
        // reprojection error. the primary estimate is the one with lower reprojection error, so it's the
        // one we go with. we can't calculate covariance, of course, but it's better than nothing.
        return { {
            observations.front().time,
            observations.front().primaryEstimate,
            Eigen::Vector<double, 6>::Zero()
        } };
    }

    std::vector<const Observation*> observationPtrs;
    std::ranges::transform(observations, std::back_inserter(observationPtrs), [](const auto& observation) {
        return &observation;
    });
    return apecsTwoPlus(observationPtrs);
}
