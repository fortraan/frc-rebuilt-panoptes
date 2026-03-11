#include "ransac.h"

#include <algorithm>
#include <complex>
#include <cmath>
#include <ranges>
#include <random>
#include <vector>

#include "utilities.h"

namespace {
    Model fitModel(const std::vector<Pose>& poses) {
        using namespace std::complex_literals;

        double xSum = 0, ySum = 0;
        std::complex<double> headingSum = 0;

        for (const auto& [x, y, heading] : poses) {
            xSum += x;
            ySum += y;
            headingSum += std::exp(heading * 1.0i);
        }

        const auto numPoses = static_cast<double>(poses.size());

        Model model { };
        model.x.mean = xSum / numPoses;
        model.y.mean = ySum / numPoses;
        const std::complex<double> headingMoment = headingSum / numPoses;
        KC_DEBUG_ASSERT(std::norm(headingMoment) <= 1.0, "RANSAC: heading moment outside of unit circle!");
        model.heading.mean = std::arg(headingMoment);
        model.heading.standardDeviation = std::sqrt(
            -2 * std::log(std::min(1.0, std::norm(headingMoment)))
        );

        // circular mean and standard deviation can be acquired from the same metric (circular moment).
        // regular standard deviation unfortunately requires another loop. we'll reuse xSum and ySum.
        xSum = ySum = 0;
        for (const auto& pose : poses) {
            xSum += std::pow(pose.x - model.x.mean, 2);
            ySum += std::pow(pose.y - model.y.mean, 2);
        }
        model.x.standardDeviation = std::sqrt(xSum / numPoses);
        model.y.standardDeviation = std::sqrt(ySum / numPoses);

        return model;
    }

    double wrap(double x) {
        while (x < std::numbers::pi) x += 2 * std::numbers::pi;
        while (x > std::numbers::pi) x -= 2 * std::numbers::pi;
        return x;
    }

    int countInliers(const Model& model, const std::vector<Pose>& poses, const double allowedDeviations = 1) {
        return static_cast<int>(std::ranges::count_if(poses, [&](const Pose& pose) {
            return std::abs(pose.x - model.x.mean) < model.x.standardDeviation * allowedDeviations &&
                std::abs(pose.y - model.y.mean) < model.y.standardDeviation * allowedDeviations &&
                std::abs(wrap(pose.heading - model.heading.mean)) < model.heading.standardDeviation * allowedDeviations;
        }));
    }

    int factorial(const int x) {
        // simple iterative implementation. for our purposes, x should be small enough to not be a performance issue.
        int y = 1;
        for (int i = 1; i <= x; i++) {
            y *= i;
        }
        return y;
    }

    int nCr(const int n, const int r) {
        return factorial(n) / (factorial(r) * factorial(n - r));
    }
}

Model ransac(const std::vector<Pose>& poses, const RansacConfig& config) {
    auto [iterations, sampleSize, threshold] = config;

    // check basic case first - are all poses in agreement?
    // alternatively, do we not have enough poses to reach consensus?
    const Model allPosesModel = fitModel(poses);
    if (poses.size() < 3 || countInliers(allPosesModel, poses) == static_cast<int>(poses.size())) {
        // no RANSAC needed. return a model fitted to all poses.
        return allPosesModel;
    }

    if (sampleSize < 0) {
        sampleSize = std::ceil(static_cast<float>(poses.size()) / 2.0f);
    }
    // don't iterate more times than there are unique combinations
    // todo in this limiting case, iterate through all combinations instead of random choices
    //iterations = std::min(nCr(static_cast<int>(poses.size()), sampleSize), iterations);
    if (iterations < 0) {
        iterations = nCr(static_cast<int>(poses.size()), sampleSize);
    }

    Model bestModel { };
    auto bestLoss = std::numeric_limits<double>::max();
    int bestNumInliers = -1;
    std::vector<Pose> sample;
    sample.reserve(sampleSize);

    // static to allow reuse, increase randomness, and avoid putting a 5KB object on the stack
    static std::mt19937 generator(std::random_device().operator()());
    for (int i = 0; i < iterations; i++) {
        sample.clear();
        std::ranges::sample(poses, std::back_inserter(sample), sampleSize, generator);

        const Model model = fitModel(sample);

        // const int numInliers = countInliers(model, sample);
        // if (numInliers > bestNumInliers) {
        //     bestModel = model;
        //     bestNumInliers = numInliers;
        // }

        const double loss = model.x.standardDeviation + model.y.standardDeviation + model.heading.standardDeviation;
        if (loss < bestLoss) {
            bestModel = model;
            bestLoss = loss;
        }
    }

    return bestModel;
}