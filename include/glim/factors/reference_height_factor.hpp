#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <boost/serialization/base_object.hpp>

namespace glim
{

// Constrains only the world-frame vertical translation of one GLIM Pose3
// state.  The first implementation deliberately assumes coincident IMU and
// barometer origins; adding a lever arm later requires rotation terms here.
class ReferenceHeightFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;
  using Base::evaluateError;
  ReferenceHeightFactor() = default;

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new ReferenceHeightFactor(*this));
  }

  ReferenceHeightFactor(
    gtsam::Key current_key, double measured_height_m,
    const gtsam::SharedNoiseModel & noise_model,
    const gtsam::Vector3 & vertical_axis = gtsam::Vector3::UnitZ())
  : Base(noise_model, current_key),
    measured_height_m_(measured_height_m),vertical_axis_(vertical_axis.normalized()) {}

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & current,
    gtsam::OptionalMatrixType H_current = nullptr) const override
  {
    // Ask Pose3 for its local-coordinate translation Jacobian instead of
    // assuming a particular Pose3 retraction convention.
    gtsam::Matrix36 current_translation_jacobian;
    const double current_z = vertical_axis_.dot(current.translation(current_translation_jacobian));

    if (H_current != nullptr) {
      *H_current = vertical_axis_.transpose()*current_translation_jacobian;
    }

    gtsam::Vector1 error;
    error << current_z - measured_height_m_;
    return error;
  }

private:
  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive& archive, const unsigned int) {
    archive & boost::serialization::make_nvp("base", boost::serialization::base_object<Base>(*this));
    archive & boost::serialization::make_nvp("height", measured_height_m_);
    archive & boost::serialization::make_nvp("vertical_axis", vertical_axis_);
  }
  double measured_height_m_{0.0};
  gtsam::Vector3 vertical_axis_{gtsam::Vector3::UnitZ()};
};

}  // namespace glim
