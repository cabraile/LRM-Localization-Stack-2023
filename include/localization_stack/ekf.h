#pragma once

#include <memory>

#include <gtsam/nonlinear/ExtendedKalmanFilter.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace lrm {

using gtsam::Symbol;
using gtsam::Pose3;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::Quaternion;
using gtsam::ExtendedKalmanFilter;
using gtsam::BetweenFactor;
using gtsam::PriorFactor;

class EKF
{

public:

    EKF();

    void initialize(const Pose3 & utm_T_base_link_origin, const Vector & covariance_diagonals);

    void predict(const Pose3 & odom_T_base_link_curr, const Vector & covariance_diagonals);

    void update(const Pose3 & utm_T_base_link_curr, const Vector & covariance_diagonals);

    bool isInitialized() const;

    Pose3 getCurrentLocalState() const;

    Pose3 getOriginToUTMTransform() const;

private:

    std::shared_ptr< ExtendedKalmanFilter<Pose3> > _ekf;

    Pose3 _odom_T_base_link_prev;

    Pose3 _utm_T_base_link_origin;

    Pose3 _base_link_origin_T_base_link_curr;

    size_t _prediction_count;

    size_t _update_count;

    bool _flag_initialized;

};

}
