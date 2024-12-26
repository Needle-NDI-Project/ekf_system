#include "ekf_system/extended_kalman_filter.hpp"

namespace ekf_system
{

    ExtendedKalmanFilter::ExtendedKalmanFilter(unsigned int state_size)
        : state_size_(state_size)
    {
        // Initialize state and covariance matrices
        state_ = Vector::Zero(state_size_);
        state_covariance_ = Matrix::Identity(state_size_, state_size_);
        process_noise_ = Matrix::Identity(state_size_, state_size_);
        measurement_noise_ = Matrix::Identity(state_size_, state_size_);
        innovation_ = Vector::Zero(state_size_);
        innovation_covariance_ = Matrix::Identity(state_size_, state_size_);
    }

    void ExtendedKalmanFilter::predict(const Vector &control_input, double dt)
    {
        // Predict state using nonlinear process model
        state_ = processModel(state_, control_input, dt);

        // Calculate Jacobian of process model
        Matrix F = processJacobian(state_, control_input, dt);

        // Update state covariance
        state_covariance_ = F * state_covariance_ * F.transpose() + process_noise_;
    }

    void ExtendedKalmanFilter::update(const Vector &measurement, const Matrix &H, const Matrix &R)
    {
        // Calculate innovation
        Vector predicted_measurement = H * state_;
        innovation_ = measurement - predicted_measurement;

        // Calculate innovation covariance with provided R
        innovation_covariance_ = H * state_covariance_ * H.transpose() + R;

        // Calculate Kalman gain
        Matrix K = state_covariance_ * H.transpose() *
                   innovation_covariance_.inverse();

        // Update state
        state_ = state_ + K * innovation_;

        // Normalize quaternion part of state
        normalizeQuaternion();

        // Update covariance
        Matrix I = Matrix::Identity(state_size_, state_size_);
        state_covariance_ = (I - K * H) * state_covariance_;
    }

    void ExtendedKalmanFilter::normalizeQuaternion()
    {
        if (state_size_ >= 7)
        {                                       // If state contains quaternion
            Vector quat = state_.segment<4>(3); // Assuming quaternion starts at index 3
            double norm = quat.norm();
            if (norm > 0)
            {
                state_.segment<4>(3) = quat / norm;
            }
        }
    }

    Vector ExtendedKalmanFilter::processModelQuaternion(const Vector &state, const Vector &control, double dt) const
    {
        Vector new_state = state;

        // Update position with velocity
        if (state_size_ >= 13)
        {                                                        // Full state with velocity
            new_state.segment<3>(0) += state.segment<3>(7) * dt; // Position update

            // Convert angular velocity to quaternion rate
            Vector omega = state.segment<3>(10); // Angular velocity
            Vector quat = state.segment<4>(3);   // Current quaternion

            // Quaternion integration (simplified - should use proper quaternion integration)
            Matrix4d omega_matrix;
            omega_matrix << 0, -omega(0), -omega(1), -omega(2),
                omega(0), 0, omega(2), -omega(1),
                omega(1), -omega(2), 0, omega(0),
                omega(2), omega(1), -omega(0), 0;

            new_state.segment<4>(3) += 0.5 * omega_matrix * quat * dt;
        }

        return new_state;
    }

    void ExtendedKalmanFilter::reset(const Vector &initial_state)
    {
        if (initial_state.size() == state_size_)
        {
            state_ = initial_state;
        }
        else
        {
            state_ = Vector::Zero(state_size_);
        }
        state_covariance_ = Matrix::Identity(state_size_, state_size_);
    }

    void ExtendedKalmanFilter::setProcessNoiseCovariance(const Matrix &Q)
    {
        if (Q.rows() == state_size_ && Q.cols() == state_size_)
        {
            process_noise_ = Q;
        }
    }

    void ExtendedKalmanFilter::setMeasurementNoiseCovariance(const Matrix &R)
    {
        if (R.rows() == state_size_ && R.cols() == state_size_)
        {
            measurement_noise_ = R;
        }
    }

    void ExtendedKalmanFilter::setInitialStateCovariance(const Matrix &P)
    {
        if (P.rows() == state_size_ && P.cols() == state_size_)
        {
            state_covariance_ = P;
        }
    }

    void ExtendedKalmanFilter::setInitialState(const Vector &x0)
    {
        if (x0.size() == state_size_)
        {
            state_ = x0;
        }
    }

    // Default implementation of process model (should be overridden for specific systems)
    Matrix ExtendedKalmanFilter::processJacobian(
        const Vector &state, const Vector &control, double dt) const
    {
        return Matrix::Identity(state_size_, state_size_);
    }

    Vector ExtendedKalmanFilter::processModel(
        const Vector &state, const Vector &control, double dt) const
    {
        return state; // Simple identity model by default
    }

    Matrix ExtendedKalmanFilter::measurementJacobian(const Vector &state) const
    {
        return Matrix::Identity(state_size_, state_size_);
    }

    Vector ExtendedKalmanFilter::measurementModel(const Vector &state) const
    {
        return state; // Simple identity measurement model by default
    }

} // namespace ekf_system