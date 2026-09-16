#include "thrFiringRemainderAlgorithm.h"

#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

namespace {
constexpr float kMaxThrust = 0.5F;
constexpr float kThrMinFireTime = 0.2F;
constexpr float kControlPeriod = 0.5F;
constexpr float kFinalTime = 3.0F;
constexpr size_t kTotalSteps = static_cast<size_t>(kFinalTime / kControlPeriod) + 1U;
constexpr float kOnTimeOversaturationFactor = 1.1F;
constexpr unsigned int kRandomSeed = 42U;

ThrFiringRemainderThrusterArray makeUniformThrusterArray(float maxThrust) {
    ThrFiringRemainderThrusterArray thrusterArray{};
    thrusterArray.maxThrust.fill(maxThrust);
    return thrusterArray;
}

ThrusterForceCmd createThrusterForceCmd(std::mt19937& rng, float maxThrust, ThrustPulsingRegime regime) {
    ThrusterForceCmd forces{};
    if (regime == ThrustPulsingRegime::OFF_PULSING) {
        std::uniform_real_distribution<float> forceDist(-maxThrust, 0.0F);
        for (size_t i = 0; i < kMaxThrusterCount; ++i) {
            forces.thrForce[i] = forceDist(rng);
        }
    } else {
        std::uniform_real_distribution<float> forceDist(0.0F, maxThrust);
        for (size_t i = 0; i < kMaxThrusterCount; ++i) {
            forces.thrForce[i] = forceDist(rng);
        }
    }
    return forces;
}

std::vector<std::vector<float>> computeExpectedOutputs(float maxThrust,
                                                       const ThrusterForceCmd& thrForce,
                                                       ThrustPulsingRegime regime,
                                                       float controlPeriod,
                                                       float thrMinFireTime,
                                                       float onTimeSaturationFactor,
                                                       size_t totalSteps) {
    std::vector<float> pulseRemainder(kMaxThrusterCount, 0.0F);
    std::vector<std::vector<float>> expected(totalSteps, std::vector<float>(kMaxThrusterCount));

    for (size_t idx = 0; idx < totalSteps; ++idx) {
        for (size_t thrIdx = 0; thrIdx < kMaxThrusterCount; ++thrIdx) {
            float thrust = thrForce.thrForce[thrIdx];
            if (regime == ThrustPulsingRegime::OFF_PULSING) {
                thrust += maxThrust;
            }
            thrust = std::max(thrust, 0.0F);

            float onTime = thrust / maxThrust * controlPeriod;
            onTime += pulseRemainder[thrIdx] * thrMinFireTime;
            pulseRemainder[thrIdx] = 0.0F;

            if (onTime < thrMinFireTime) {
                pulseRemainder[thrIdx] = onTime / thrMinFireTime;
                onTime = 0.0F;
            } else if (onTime >= controlPeriod) {
                onTime = onTimeSaturationFactor * controlPeriod;
            }
            expected[idx][thrIdx] = onTime;
        }
    }
    return expected;
}
}  // namespace

class ThrFiringRemainderTest : public ::testing::TestWithParam<ThrustPulsingRegime> {};

TEST_P(ThrFiringRemainderTest, ComputesCorrectOnTimes) {
    const ThrustPulsingRegime regime = GetParam();

    // Use seeded RNG for reproducibility
    std::mt19937 rng(kRandomSeed);
    // Generate thruster configuration and forces
    const auto thrusterArray = makeUniformThrusterArray(kMaxThrust);
    const auto thrForce = createThrusterForceCmd(rng, kMaxThrust, regime);

    // Compute expected outputs
    const auto expected = computeExpectedOutputs(
        kMaxThrust, thrForce, regime, kControlPeriod, kThrMinFireTime, kOnTimeOversaturationFactor, kTotalSteps);

    // Configure and run algorithm
    const ThrFiringControlParameters params{kThrMinFireTime, kControlPeriod, kOnTimeOversaturationFactor, regime};
    ThrFiringRemainderAlgorithm algorithm{ThrFiringRemainderConfig::create(thrusterArray, params)};

    // Run algorithm and collect outputs
    std::vector<std::vector<float>> actual(kTotalSteps, std::vector<float>(kMaxThrusterCount));
    for (size_t idx = 0; idx < kTotalSteps; ++idx) {
        const auto [OnTimeRequest] = algorithm.update(thrForce);
        for (size_t thrIdx = 0; thrIdx < kMaxThrusterCount; ++thrIdx) {
            actual[idx][thrIdx] = OnTimeRequest[thrIdx];
        }
    }

    // Verify results
    ASSERT_EQ(expected.size(), actual.size());
    for (size_t idx = 0; idx < expected.size(); ++idx) {
        for (size_t thrIdx = 0; thrIdx < kMaxThrusterCount; ++thrIdx) {
            EXPECT_NEAR(actual[idx][thrIdx], expected[idx][thrIdx], 1e-6F)
                << "Mismatch at step " << idx << " thruster " << thrIdx;
        }
    }
}

TEST(ThrFiringRemainderConfigTest, RejectsInvalidParameters) {
    ThrFiringRemainderThrusterArray validArray{};
    validArray.maxThrust.fill(kMaxThrust);
    const ThrFiringControlParameters validParams{
        kThrMinFireTime, kControlPeriod, kOnTimeOversaturationFactor, ThrustPulsingRegime::ON_PULSING};

    // Negative thrMinFireTime
    EXPECT_THROW(ThrFiringRemainderConfig::create(
                     validArray, {-0.1F, kControlPeriod, kOnTimeOversaturationFactor, ThrustPulsingRegime::ON_PULSING}),
                 fsw::invalid_argument);

    // Non-positive controlPeriod
    EXPECT_THROW(
        ThrFiringRemainderConfig::create(
            validArray, {kThrMinFireTime, -0.1F, kOnTimeOversaturationFactor, ThrustPulsingRegime::ON_PULSING}),
        fsw::invalid_argument);
    EXPECT_THROW(ThrFiringRemainderConfig::create(
                     validArray, {kThrMinFireTime, 0.0F, kOnTimeOversaturationFactor, ThrustPulsingRegime::ON_PULSING}),
                 fsw::invalid_argument);

    // onTimeSaturationFactor must be >= 1.0
    EXPECT_THROW(ThrFiringRemainderConfig::create(
                     validArray, {kThrMinFireTime, kControlPeriod, 0.99F, ThrustPulsingRegime::ON_PULSING}),
                 fsw::invalid_argument);

    // Negative maxThrust
    ThrFiringRemainderThrusterArray negThrustArray{};
    negThrustArray.maxThrust.fill(kMaxThrust);
    negThrustArray.maxThrust.at(0) = -0.1F;
    EXPECT_THROW(ThrFiringRemainderConfig::create(negThrustArray, validParams), fsw::invalid_argument);

    // Zero maxThrust: every slot is a divisor now, so zero is rejected
    ThrFiringRemainderThrusterArray zeroThrustArray{};
    zeroThrustArray.maxThrust.fill(kMaxThrust);
    zeroThrustArray.maxThrust.at(0) = 0.0F;
    EXPECT_THROW(ThrFiringRemainderConfig::create(zeroThrustArray, validParams), fsw::invalid_argument);

    // A default-constructed array is all zeros, so it is rejected too
    EXPECT_THROW(ThrFiringRemainderConfig::create(ThrFiringRemainderThrusterArray{}, validParams),
                 fsw::invalid_argument);

    // Valid configuration is accepted
    EXPECT_NO_THROW(ThrFiringRemainderConfig::create(validArray, validParams));
}

INSTANTIATE_TEST_SUITE_P(ThrFiringRemainder,
                         ThrFiringRemainderTest,
                         ::testing::Values(ThrustPulsingRegime::ON_PULSING, ThrustPulsingRegime::OFF_PULSING));
