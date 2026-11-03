#include "audition_perception/classifier_observation_config.hpp"
#include <gtest/gtest.h>
using audition_perception::ClassifierObservationConfig;
TEST(ClassifierObservationConfig, DefaultsAreValid) {
  EXPECT_NO_THROW(ClassifierObservationConfig{}.validate());
}
TEST(ClassifierObservationConfig, ValidBounds) {
  EXPECT_NO_THROW((ClassifierObservationConfig{200,1000,1000}).validate());
  EXPECT_NO_THROW((ClassifierObservationConfig{10000,600000,600000}).validate());
}
TEST(ClassifierObservationConfig, RejectsOutOfRangeSampling) {
  EXPECT_THROW((ClassifierObservationConfig{199,10000,30000}).validate(),std::invalid_argument);
  EXPECT_THROW((ClassifierObservationConfig{10001,10000,30000}).validate(),std::invalid_argument);
}
TEST(ClassifierObservationConfig, RejectsShortOrOversizedThresholds) {
  EXPECT_THROW((ClassifierObservationConfig{1000,999,30000}).validate(),std::invalid_argument);
  EXPECT_THROW((ClassifierObservationConfig{1000,10000,600001}).validate(),std::invalid_argument);
}
TEST(ClassifierObservationConfig, RejectsThresholdShorterThanSampleInterval) {
  EXPECT_THROW((ClassifierObservationConfig{5000,4000,30000}).validate(),std::invalid_argument);
  EXPECT_THROW((ClassifierObservationConfig{5000,10000,4999}).validate(),std::invalid_argument);
}
