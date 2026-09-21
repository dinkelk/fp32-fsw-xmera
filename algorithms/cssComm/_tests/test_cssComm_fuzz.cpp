#include "cssCommTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(CssCommAlgorithmFuzz, regressionTestCssComm)
    .WithDomains(fuzztest::VectorOf(fuzztest::InRange(1e-5, 1e-2)).WithSize(kMaxNumCssSensors),  // maxSensorValues
                 fuzztest::VectorOf(fuzztest::InRange(-1e1, 1e1)).WithSize(kMaxNumChebyPolys),   // chebyCoeffs
                 fuzztest::VectorOf(fuzztest::InRange(-1.5, 1.5)).WithSize(kMaxNumCssSensors));  // sensorInputRatios
