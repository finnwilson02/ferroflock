#ifndef UKF_H
#define UKF_H

#include <Eigen/Dense>
#include <string>
#include <vector>

/**
 * Unscented Kalman Filter for drone state estimation.
 * Fuses OptiTrack, IMU, and control inputs.
 * State vector: [x, y, z, vx, vy, vz, psi]
 * Designed to run predict at 100 Hz.
 */
class UKF {
public:
    /**
     * Constructor.
     */
    UKF();

    /**
     * Load parameters from JSON configuration file.
     * @param config_path Path to the configuration file.
     */
    void loadConfig(const std::string& config_path);

    /**
     * Set control input and IMU yaw rate with timestamp.
     * @param t Current timestamp in seconds.
     * @param u Control input vector (rc_lr, rc_fb, rc_ud, rc_yaw).
     * @param omega Yaw rate from IMU.
     */
    void setControlInput(double t, const Eigen::Vector4d& u, double omega);

    /**
     * Predict state forward in time. Should be called at 100 Hz.
     * @param t Current timestamp in seconds.
     */
    void predict(double t);

    /**
     * Update state with OptiTrack measurement.
     * @param t Current timestamp in seconds.
     * @param z_opti OptiTrack measurement (x, y, z, psi).
     */
    void update(double t, const Eigen::Vector4d& z_opti);

    /**
     * Get the current state estimate.
     * @return State vector [x, y, z, vx, vy, vz, psi].
     */
    Eigen::VectorXd getState() const;

private:
    // State vector [x, y, z, vx, vy, vz, psi]
    Eigen::VectorXd x_;
    
    // State covariance matrix
    Eigen::MatrixXd P_;
    
    // Process noise covariance
    Eigen::MatrixXd Q_;
    
    // Measurement noise covariance
    Eigen::MatrixXd R_;
    
    // Control input gains
    double k_fb_, k_lr_, k_ud_, k_yaw_;
    
    // UKF parameters
    double alpha_, beta_, kappa_;
    double lambda_;
    
    // Weights for mean and covariance calculation
    std::vector<double> weights_mean_;
    std::vector<double> weights_cov_;
    
    // Timestamps
    double last_predict_time_;
    double last_opti_time_;
    
    // Latest inputs
    Eigen::Vector4d control_input_; // rc_lr, rc_fb, rc_ud, rc_yaw
    double omega_; // yaw rate
    
    // Number of state variables
    int n_x_;
    
    // Number of sigma points
    int n_sigma_;
    
    // Sigma points matrix
    Eigen::MatrixXd Xsig_;
    
    /**
     * Process model - predicts state evolution.
     * @param x Current state.
     * @param dt Time step.
     * @return Next state.
     */
    Eigen::VectorXd f(const Eigen::VectorXd& x, double dt);
    
    /**
     * Measurement model - maps state to measurement space.
     * @param x Current state.
     * @return Expected measurement.
     */
    Eigen::Vector4d h(const Eigen::VectorXd& x);
    
    /**
     * Generate sigma points for UKF.
     */
    void generateSigmaPoints();
    
    /**
     * Adjust yaw measurement to handle 180-degree flips.
     * @param psi_meas Measured yaw.
     * @param psi_est Estimated yaw.
     * @return Corrected yaw measurement.
     */
    double adjustYawMeasurement(double psi_meas, double psi_est);
};

#endif // UKF_H