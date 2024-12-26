#ifndef EKF_SYSTEM_EXTENDED_KALMAN_FILTER_HPP_
#define EKF_SYSTEM_EXTENDED_KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace ekf_system
{
    using Matrix = Eigen::MatrixXd;
    using Vector = Eigen::VectorXd;
    using Matrix4d = Eigen::Matrix<double, 4, 4>;

    class ExtendedKalmanFilter
    {
    public:
        // using Matrix = Eigen::MatrixXd;
        // using Vector = Eigen::VectorXd;

        explicit ExtendedKalmanFilter(unsigned int state_size = 13); // Default state size for pose + velocity
        virtual ~ExtendedKalmanFilter() = default;

        // Main EKF methods
        void predict(const Vector &control_input, double dt);
        void update(const Vector &measurement, const Matrix &H, const Matrix &R);
        void reset(const Vector &initial_state = Vector());

        // Setters for noise parameters
        void setProcessNoiseCovariance(const Matrix &Q);
        void setMeasurementNoiseCovariance(const Matrix &R);
        void setInitialStateCovariance(const Matrix &P);
        void setInitialState(const Vector &x0);

        // Getters for state and covariance
        Vector getState() const { return state_; }
        Matrix getStateCovariance() const { return state_covariance_; }

        // Get innovation and its covariance (for diagnostic purposes)
        Vector getInnovation() const { return innovation_; }
        Matrix getInnovationCovariance() const { return innovation_covariance_; }

        // Add quaternion handling
        void normalizeQuaternion();

        // Add specific process model implementations
        Vector processModelQuaternion(const Vector &state, const Vector &control, double dt) const;

    protected:
        // Core EKF matrices
        Vector state_;             // State vector
        Matrix state_covariance_;  // State covariance matrix
        Matrix process_noise_;     // Process noise covariance matrix
        Matrix measurement_noise_; // Measurement noise covariance matrix

        // Additional variables for diagnostics
        Vector innovation_;            // Innovation vector
        Matrix innovation_covariance_; // Innovation covariance matrix

        // Virtual methods for system model definition
        virtual Matrix processJacobian(const Vector &state, const Vector &control, double dt) const;
        virtual Vector processModel(const Vector &state, const Vector &control, double dt) const;
        virtual Matrix measurementJacobian(const Vector &state) const;
        virtual Vector measurementModel(const Vector &state) const;

    private:
        const unsigned int state_size_;
    };

} // namespace ekf_system

#endif // EKF_SYSTEM_EXTENDED_KALMAN_FILTER_HPP_