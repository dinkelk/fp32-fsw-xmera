#include "thrFiringRemainderAlgorithm_c.h"
#include "thrFiringRemainderAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <algorithm>

namespace {
ThrFiringRemainderConfig configFromC(float maxThrust[MAX_EFF_CNT],
                                     const float thrMinFireTime,
                                     const float controlPeriod,
                                     const float onTimeSaturationFactor,
                                     const ThrFiringRemainderPulsingRegime pulsingRegime) {
    ThrFiringRemainderThrusterArray thrusterArray{};
    std::copy(maxThrust, maxThrust + kMaxThrusterCount, thrusterArray.maxThrust.begin());

    const ThrFiringControlParameters params{
        thrMinFireTime, controlPeriod, onTimeSaturationFactor, static_cast<ThrustPulsingRegime>(pulsingRegime)};

    return ThrFiringRemainderConfig::create(thrusterArray, params);
}
}  // namespace

uint32_t ThrFiringRemainderAlgorithm_getMaxThrusterCount(void) { return kMaxThrusterCount; }

bool ThrFiringRemainderAlgorithm_validateConfig(float maxThrust[MAX_EFF_CNT],
                                                const float thrMinFireTime,
                                                const float controlPeriod,
                                                const float onTimeSaturationFactor,
                                                const ThrFiringRemainderPulsingRegime pulsingRegime) {
    // Attempt to build the config through the real create path; success means valid,
    // a throw means invalid. Reusing configFromC keeps validation from drifting.
    try {
        (void)configFromC(maxThrust, thrMinFireTime, controlPeriod, onTimeSaturationFactor, pulsingRegime);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ThrFiringRemainderAlgorithmHandle* ThrFiringRemainderAlgorithm_create(
    float maxThrust[MAX_EFF_CNT],
    const float thrMinFireTime,
    const float controlPeriod,
    const float onTimeSaturationFactor,
    const ThrFiringRemainderPulsingRegime pulsingRegime) {
    return fsw::createHandle<::ThrFiringRemainderAlgorithm, ThrFiringRemainderAlgorithmHandle>(
        configFromC(maxThrust, thrMinFireTime, controlPeriod, onTimeSaturationFactor, pulsingRegime));
}

void ThrFiringRemainderAlgorithm_destroy(ThrFiringRemainderAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrFiringRemainderAlgorithm>(self);
}

void ThrFiringRemainderAlgorithm_setConfig(ThrFiringRemainderAlgorithmHandle* self,
                                           float maxThrust[MAX_EFF_CNT],
                                           const float thrMinFireTime,
                                           const float controlPeriod,
                                           const float onTimeSaturationFactor,
                                           const ThrFiringRemainderPulsingRegime pulsingRegime) {
    fsw::fromHandle<::ThrFiringRemainderAlgorithm>(self)->setConfig(
        configFromC(maxThrust, thrMinFireTime, controlPeriod, onTimeSaturationFactor, pulsingRegime));
}

void ThrFiringRemainderAlgorithm_reInitialize(ThrFiringRemainderAlgorithmHandle* self) {
    fsw::fromHandle<::ThrFiringRemainderAlgorithm>(self)->reInitialize();
}

ThrFiringRemainderOnTimeCmd ThrFiringRemainderAlgorithm_update(ThrFiringRemainderAlgorithmHandle* self,
                                                               const ThrFiringRemainderForceCmd* forceCmd) {
    ThrusterForceCmd cppCmd{};
    std::ranges::copy_n(forceCmd->thrForce, kMaxThrusterCount, cppCmd.thrForce.data());

    const auto [onTimeRequest] = fsw::fromHandle<::ThrFiringRemainderAlgorithm>(self)->update(cppCmd);

    ThrFiringRemainderOnTimeCmd result{};
    std::ranges::copy_n(onTimeRequest.data(), kMaxThrusterCount, result.onTimeRequest);
    return result;
}
