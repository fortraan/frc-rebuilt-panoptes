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
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> positionStats(const std::vector<const Point*>& positions) {
        KC_DEBUG_ASSERT(!positions.empty(), "no positions provided to positionStats!");
        KC_DEBUG_ASSERT(positions.size() <= std::numeric_limits<Eigen::Index>::max(), "too many positions!");

        const auto numPositions = static_cast<Eigen::Index>(positions.size());

        // this function will be called multiple times per invocation of apecs() with the same number of positions, so
        // we can keep a few objects cached to reduce dynamic allocation. we use thread_local here to allow apecs() to
        // be reentrant.
        thread_local Eigen::Matrix3Xd mat;
        mat.resize(Eigen::NoChange, numPositions);

        for (Eigen::Index i = 0; i < numPositions; i++) {
            mat(0, i) = positions[i]->x;
            mat(1, i) = positions[i]->y;
            mat(2, i) = positions[i]->z;
        }

        Eigen::Vector3d mean = mat.rowwise().mean();
        thread_local Eigen::Matrix3Xd error = mat.colwise() - mean;
        Eigen::Matrix3d covariance;
        if (numPositions > 1) {
            covariance = error * error.transpose() / static_cast<double>(numPositions - 1);
        } else {
            covariance.setZero();
        }

        return {
            mean,
            covariance
        };
    }

    // reference:
    // Development of a Real-Time Attitude System Using a Quaternion Parameterization and Non-Dedicated GPS Receivers by
    // John B. Schleppe, eqn. 4.80 through 4.84d.
    // https://www.ucalgary.ca/engo_webdocs/GL/96.20096.JSchleppe.pdf
    Eigen::Matrix<double, 3, 4> quatToEulerJacobian(const Eigen::Vector4d& quaternion) {
        // todo
        return Eigen::Matrix<double, 3, 4>::Zero();
    }

    // references:
    // https://www.acsu.buffalo.edu/%7Ejohnc/ave_quat07.pdf
    // https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
    std::pair<Quaternion, Eigen::Matrix3d> quatAverage(const std::vector<const Quaternion*>& quaternions) {
        if (quaternions.empty()) {
            return {
                Quaternion(),
                Eigen::Matrix3d::Zero()
            };
        }

        KC_DEBUG_ASSERT(quaternions.size() <= std::numeric_limits<Eigen::Index>::max(), "too many quaternions!");
        const auto numQuaternions = static_cast<Eigen::Index>(quaternions.size());
        thread_local Eigen::Matrix4Xd q;
        q.resize(Eigen::NoChange,  numQuaternions);

        for (Eigen::Index i = 0; i < numQuaternions; i++) {
            const auto* quaternion = quaternions[i];
            q(0, i) = quaternion->x;
            q(1, i) = quaternion->y;
            q(2, i) = quaternion->z;
            q(3, i) = quaternion->w;
        }

        const Eigen::Matrix4d secondMoment = q * q.transpose();
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(secondMoment);
        Eigen::Index maxIndex;
        solver.eigenvalues().maxCoeff(&maxIndex);
        const Eigen::Vector4d mean = solver.eigenvectors().col(maxIndex);

        Eigen::Matrix3d rpyCovariance;
        if (numQuaternions > 1) {
            const Eigen::Matrix4d quatCovariance = (q - mean) * (q - mean).transpose() / static_cast<double>(numQuaternions - 1);
            const Eigen::Matrix<double, 3, 4> jacobian = quatToEulerJacobian(mean);
            rpyCovariance = jacobian * quatCovariance * jacobian.transpose();
        } else {
            rpyCovariance.setZero();
        }

        KC_DEBUG_ASSERT(!mean.hasNaN(), "quatAverage has nan!");

        Quaternion ret;
        ret.x = mean(0);
        ret.y = mean(1);
        ret.z = mean(2);
        ret.w = mean(3);

        return {
            ret,
            rpyCovariance
        };
    }

    rclcpp::Time timeAverage(const std::vector<rclcpp::Time>& times) {
        KC_DEBUG_ASSERT(!times.empty(), "no times provided to timeAverage!");

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

    Model fitModel(const std::vector<std::pair<rclcpp::Time, const Pose*>>& transforms) {
        thread_local std::vector<rclcpp::Time> times;
        thread_local std::vector<const Point*> positions;
        thread_local std::vector<const Quaternion*> rotations;
        times.clear();
        positions.clear();
        rotations.clear();
        times.reserve(transforms.size());
        positions.reserve(transforms.size());
        rotations.reserve(transforms.size());
        for (const auto& [time, pose] : transforms) {
            times.emplace_back(time);
            positions.push_back(&pose->position);
            rotations.push_back(&pose->orientation);
        }
        
        const auto timeMean = timeAverage(times);
        const auto [positionMean, positionCovariance] = positionStats(positions);
        const auto [rotationMean, rotationCovariance] = quatAverage(rotations);
        
        Model ret;
        ret.time = timeMean;
        ret.mean.position.x = positionMean[0];
        ret.mean.position.y = positionMean[1];
        ret.mean.position.z = positionMean[2];
        ret.mean.orientation = rotationMean;
        ret.covariance.setZero();
        ret.covariance(Eigen::seqN(0, 3), Eigen::seqN(0, 3)) = positionCovariance;
        ret.covariance(Eigen::seqN(3, 3), Eigen::seqN(3, 3)) = rotationCovariance;
        return ret;
    }

    std::optional<Model> apecsTwoPlus(const std::vector<const Observation*>& observations) {
        KC_DEBUG_ASSERT(!observations.empty(), "apecsTwoPlus requires at least 2 observations");

        const auto hasSecondary = [](const Observation* observation) {
            return observation->secondaryEstimate.has_value();
        };
        std::vector<const Observation*> singlets, duals;
        std::ranges::copy_if(observations, std::back_inserter(singlets), std::not_fn(hasSecondary));
        std::ranges::copy_if(observations, std::back_inserter(duals), hasSecondary);

        const auto numSinglets = singlets.size();
        const auto numDuals = duals.size();
        const auto hasSinglets = numSinglets != 0;
        const auto hasDuals = numDuals != 0;

        KC_DEBUG_ASSERT(numDuals <= 64, "Too many duals to fit in combination variable!");

        std::vector<std::pair<rclcpp::Time, const Pose*>> singletPairs;
        if (hasSinglets) {
            singletPairs.reserve(numSinglets);
            for (const auto* singlet : singlets) {
                singletPairs.emplace_back(singlet->time, &singlet->primaryEstimate);
            }
        }

        std::optional<Model> bestModel;
        if (hasDuals) {
            std::vector<std::pair<rclcpp::Time, const Pose*>> transformCombo;
            transformCombo.reserve(numSinglets + numDuals);
            const uint64_t numCombinations = 1 << numDuals;
            for (uint64_t combination = 0; combination < numCombinations; combination++) {
                transformCombo.clear();
                if (hasSinglets) std::ranges::copy(singletPairs, std::back_inserter(transformCombo));
                for (size_t i = 0; i < numDuals; i++) {
                    transformCombo.emplace_back(
                        duals[i]->time,
                        1 & combination >> i ? &duals[i]->primaryEstimate
                                             : &duals[i]->secondaryEstimate.value()
                    );
                }

                const auto model = fitModel(transformCombo);
                KC_DEBUG_ASSERT(!std::isnan(model.mean.position.x), "model t.x is nan!");
                KC_DEBUG_ASSERT(!std::isnan(model.mean.position.y), "model t.y is nan!");
                KC_DEBUG_ASSERT(!std::isnan(model.mean.position.z), "model t.z is nan!");
                KC_DEBUG_ASSERT(!std::isnan(model.mean.orientation.w), "model q.w is nan!");
                KC_DEBUG_ASSERT(!std::isnan(model.mean.orientation.x), "model q.x is nan!");
                KC_DEBUG_ASSERT(!std::isnan(model.mean.orientation.y), "model q.y is nan!");
                KC_DEBUG_ASSERT(!std::isnan(model.mean.orientation.z), "model q.z is nan!");
                if (!bestModel || model.covariance.diagonal().sum() < bestModel->covariance.diagonal().sum()) {
                    bestModel = model;
                }
            }
        }

        // if we have both singlets and duals, use the best model that includes both
        if (bestModel.has_value()) return bestModel;
        // if we've only got singlets, fit a model to them and return it
        if (hasSinglets) return fitModel(singletPairs);
        return std::nullopt;
    }
}

Observation::Observation(const rclcpp::Time& time, const geometry_msgs::msg::Pose& primaryEstimate,
    const std::optional<geometry_msgs::msg::Pose>& secondaryEstimate) :
    time(time), primaryEstimate(primaryEstimate), secondaryEstimate(secondaryEstimate) { }

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
            Eigen::Matrix<double, 6, 6>::Zero()
        } };
    }

    std::vector<const Observation*> observationPtrs;
    std::ranges::transform(observations, std::back_inserter(observationPtrs), [](const auto& observation) {
        return &observation;
    });
    return apecsTwoPlus(observationPtrs);
}
