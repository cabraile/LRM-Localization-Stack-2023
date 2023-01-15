#include "localization_stack/ekf.h"

namespace lrm
{

EKF::EKF()
{
    _flag_initialized = false;
}

void EKF::initialize(const Pose3 & utm_T_base_link_origin, const Vector & covariance_diagonals)
{
    Symbol x_0('x',0);
    auto noise_model = gtsam::noiseModel::Diagonal::Variances(covariance_diagonals);
    _ekf = std::shared_ptr< ExtendedKalmanFilter<Pose3> >(
        new ExtendedKalmanFilter<Pose3>(x_0, Pose3(Quaternion(1.0,0.0,0.0,0.0), Vector3(0.0,0.0,0.0)), noise_model) 
    );
    _utm_T_base_link_origin = utm_T_base_link_origin;
    _prediction_count = 0;
    _flag_initialized = true;
}

void EKF::predict(const Pose3 & odom_T_base_link_curr, const Vector & covariance_diagonals)
{
    // Compute between pose
    Pose3 base_link_prev_T_odom = _odom_T_base_link_prev.inverse();
    Pose3 base_link_prev_T_base_link_curr = base_link_prev_T_odom * odom_T_base_link_curr;
    
    // Create the between pose factor
    Symbol x_prev('x', _prediction_count);
    Symbol x_curr('x', _prediction_count + 1 );
    auto noise_model = gtsam::noiseModel::Diagonal::Variances(covariance_diagonals);
    BetweenFactor<Pose3> between_pose_factor(x_prev, x_curr, base_link_prev_T_base_link_curr, noise_model);

    // Perform prediction
    _base_link_origin_T_base_link_curr = _ekf->predict(between_pose_factor);

    // Update control variables
    _odom_T_base_link_prev = odom_T_base_link_curr;
    _prediction_count++;
}

void EKF::update(const Pose3 & utm_T_base_link_curr, const Vector & covariance_diagonals)
{
    // Convert to the local frame first - numerical reason
    Pose3 base_link_origin_T_utm = _utm_T_base_link_origin.inverse();
    Pose3 base_link_origin_T_base_link_curr = base_link_origin_T_utm * utm_T_base_link_curr;

    // Perform correction
    Symbol x_curr('x', _prediction_count);
    auto noise_model = gtsam::noiseModel::Diagonal::Variances(covariance_diagonals);
    PriorFactor<Pose3> correction_factor(x_curr, base_link_origin_T_base_link_curr, noise_model);
    _base_link_origin_T_base_link_curr = _ekf->update(correction_factor);

    // Update control variables
    _update_count++;
}

bool EKF::isInitialized() const 
{
    return _flag_initialized;
}


Pose3 EKF::getCurrentLocalState() const
{
    return _base_link_origin_T_base_link_curr;
}

Pose3 EKF::getOriginToUTMTransform() const
{
    return _utm_T_base_link_origin;
}

} // namespace lrm