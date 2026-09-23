#include "solarArrayReference.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <stdexcept>

#include <utilities/fsw/eigenSupport.h>

/*! Validate that the required input messages are linked, build the algorithm's configuration from the adapter's
 stored properties, and (re)construct the embedded algorithm.
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SolarArrayReference::reset(uint64_t callTime) {
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("solarArrayReference.attNavInMsg wasn't connected.");
    }
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("solarArrayReference.attRefInMsg wasn't connected.");
    }

    const SolarArrayReferenceConfig config =
        SolarArrayReferenceConfig::create(SolarArrayAxes{this->driveAxis, this->surfaceNormal},
                                          this->alignmentThreshold,
                                          this->trackingMode,
                                          this->specifiedArrayAngle,
                                          this->offsetAngle);
    this->algorithm = std::make_unique<SolarArrayReferenceAlgorithm>(config);
}

SolarArrayReferenceConfig SolarArrayReference::toConfig() const {
    return SolarArrayReferenceConfig::create(SolarArrayAxes{this->driveAxis, this->surfaceNormal},
                                             this->alignmentThreshold,
                                             this->trackingMode,
                                             this->specifiedArrayAngle,
                                             this->offsetAngle);
}

void SolarArrayReference::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SolarArrayReference reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Reset the algorithm's retained runtime state without rebuilding its configuration. */
void SolarArrayReference::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SolarArrayReference reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! Read the navigation and reference messages, run the solar array reference law, and write the commanded array
 reference angle output message.
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SolarArrayReference::updateState(uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("SolarArrayReference reset() has not been called.");
    }

    NavAttMsgF32Payload attNavIn = this->attNavInMsg();
    AttRefMsgF32Payload attRefIn = this->attRefInMsg();

    const Eigen::Vector3f sigma_BN = cArrayToEigenVector(attNavIn.sigma_BN);
    const Eigen::Vector3f sigma_RN = cArrayToEigenVector(attRefIn.sigma_RN);
    const Eigen::Vector3f rHatIn_SB_B = cArrayToEigenVector(attNavIn.vehSunPntBdy);

    const float thetaRef = this->algorithm->update(sigma_BN, sigma_RN, rHatIn_SB_B);

    MotorAngleRefMsgF32Payload solarArrayRefOut = {};
    solarArrayRefOut.theta = thetaRef;
    this->solarArrayRefOutMsg.write(solarArrayRefOut, this->moduleID, callTime);
}
