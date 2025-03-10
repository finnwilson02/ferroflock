#include "../include/ukf.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>

using json = nlohmann::json;

/**
 * Constructor initializes state vector, covariance matrices and UKF parameters
 * Predict at 100 Hz; IMU at 10 Hz held constant between updates
 */
UKF::UKF() {
    // State dimension
    n_x_ = 7;
    
    // Number of sigma points
    n_sigma_ = 2 * n_x_ + 1;
    
    // Initialize state vector [x, y, z, vx, vy, vz, psi]
    x_ = Eigen::VectorXd(n_x_);
    x_.setZero();
    
    // Initialize covariance matrix
    P_ = Eigen::MatrixXd(n_x_, n_x_);
    P_.setIdentity();
    
    // Initialize process noise covariance (will be overwritten by config)
    Q_ = Eigen::MatrixXd(n_x_, n_x_);
    Q_.setIdentity();
    Q_ *= 0.01; // Default process noise
    
    // Initialize measurement noise covariance (will be overwritten by config)
    R_ = Eigen::MatrixXd(4, 4);
    R_.setIdentity();
    R_ *= 0.001; // Default measurement noise
    
    // Initialize UKF parameters (will be overwritten by config)
    alpha_ = 0.001; // Small positive value, typically 1e-4 ≤ alpha ≤ 1
    beta_ = 2.0;    // Optimal for Gaussian distributions
    kappa_ = 0.0;   // Secondary scaling parameter, typically 0
    lambda_ = alpha_ * alpha_ * (n_x_ + kappa_) - n_x_;
    
    // Initialize weights
    weights_mean_ = std::vector<double>(n_sigma_);
    weights_cov_ = std::vector<double>(n_sigma_);
    
    weights_mean_[0] = lambda_ / (n_x_ + lambda_);
    weights_cov_[0] = weights_mean_[0] + (1 - alpha_ * alpha_ + beta_);
    
    for (int i = 1; i < n_sigma_; i++) {
        weights_mean_[i] = 1.0 / (2 * (n_x_ + lambda_));
        weights_cov_[i] = weights_mean_[i];
    }
    
    // Initialize sigma points matrix
    Xsig_ = Eigen::MatrixXd(n_x_, n_sigma_);
    
    // Initialize control input gains
    k_fb_ = 1.0;
    k_lr_ = 1.0;
    k_ud_ = 1.0;
    k_yaw_ = 0.5;
    
    // Initialize timestamps
    last_predict_time_ = 0.0;
    last_opti_time_ = -1.0; // Negative indicates no measurement yet
    
    // Initialize control inputs
    control_input_ = Eigen::Vector4d::Zero();
    omega_ = 0.0;
}

/**
 * Load configuration from JSON file
 */
void UKF::loadConfig(const std::string& config_path) {
    try {
        // Open and parse config file
        std::ifstream file(config_path);
        if (!file.is_open()) {
            std::cerr << "Failed to open config file: " << config_path << std::endl;
            return;
        }
        
        json config;
        file >> config;
        
        // Load control input gains
        if (config.contains("gains")) {
            k_fb_ = config["gains"].value("k_fb", 1.0);
            k_lr_ = config["gains"].value("k_lr", 1.0);
            k_ud_ = config["gains"].value("k_ud", 1.0);
            k_yaw_ = config["gains"].value("k_yaw", 0.5);
        }
        
        // Load UKF parameters
        alpha_ = config.value("alpha", 0.001);
        beta_ = config.value("beta", 2.0);
        kappa_ = config.value("kappa", 0.0);
        
        // Recalculate lambda and weights
        lambda_ = alpha_ * alpha_ * (n_x_ + kappa_) - n_x_;
        
        weights_mean_[0] = lambda_ / (n_x_ + lambda_);
        weights_cov_[0] = weights_mean_[0] + (1 - alpha_ * alpha_ + beta_);
        
        for (int i = 1; i < n_sigma_; i++) {
            weights_mean_[i] = 1.0 / (2 * (n_x_ + lambda_));
            weights_cov_[i] = weights_mean_[i];
        }
        
        // Load process noise covariance Q
        if (config.contains("Q")) {
            for (int i = 0; i < n_x_; i++) {
                for (int j = 0; j < n_x_; j++) {
                    Q_(i, j) = config["Q"][i][j];
                }
            }
        }
        
        // Load measurement noise covariance R
        if (config.contains("R")) {
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    R_(i, j) = config["R"][i][j];
                }
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
    }
}

/**
 * Store latest control input, IMU yaw rate, and update timestamp
 */
void UKF::setControlInput(double t, const Eigen::Vector4d& u, double omega) {
    control_input_ = u;
    omega_ = omega;
    // We don't update last_predict_time_ here, only in predict()
}

/**
 * Return current state estimate
 */
Eigen::VectorXd UKF::getState() const {
    return x_;
}

/**
 * Process model - predicts state evolution based on current state and control inputs
 * @param x Current state vector [x, y, z, vx, vy, vz, psi]
 * @param dt Time step in seconds
 * @return Predicted next state
 */
Eigen::VectorXd UKF::f(const Eigen::VectorXd& x, double dt) {
    Eigen::VectorXd x_next = x;
    
    // Extract state components
    double pos_x = x(0);
    double pos_y = x(1);
    double pos_z = x(2);
    double vel_x = x(3);
    double vel_y = x(4);
    double vel_z = x(5);
    double psi = x(6);
    
    // Extract control inputs
    double rc_lr = control_input_(0);  // Left-right
    double rc_fb = control_input_(1);  // Forward-backward
    double rc_ud = control_input_(2);  // Up-down
    double rc_yaw = control_input_(3); // Yaw control
    
    // Transform body-frame control inputs to lab frame
    double accel_x = k_fb_ * rc_fb * cos(psi) - k_lr_ * rc_lr * sin(psi);
    double accel_y = k_fb_ * rc_fb * sin(psi) + k_lr_ * rc_lr * cos(psi);
    double accel_z = k_ud_ * rc_ud;
    
    // yaw rate from IMU (already in lab frame)
    double yaw_rate = k_yaw_ * rc_yaw + omega_;
    
    // Update position based on velocity
    x_next(0) = pos_x + vel_x * dt;
    x_next(1) = pos_y + vel_y * dt;
    x_next(2) = pos_z + vel_z * dt;
    
    // Update velocity based on acceleration
    x_next(3) = vel_x + accel_x * dt;
    x_next(4) = vel_y + accel_y * dt;
    x_next(5) = vel_z + accel_z * dt;
    
    // Update yaw based on yaw rate
    x_next(6) = psi + yaw_rate * dt;
    
    // Normalize yaw angle to [-π, π]
    while (x_next(6) > M_PI) x_next(6) -= 2 * M_PI;
    while (x_next(6) < -M_PI) x_next(6) += 2 * M_PI;
    
    return x_next;
}

/**
 * Measurement model - maps state to expected measurement
 * @param x State vector [x, y, z, vx, vy, vz, psi]
 * @return Expected measurement [x, y, z, psi]
 */
Eigen::Vector4d UKF::h(const Eigen::VectorXd& x) {
    Eigen::Vector4d z_pred;
    
    // OptiTrack directly measures position and yaw
    z_pred(0) = x(0); // x position
    z_pred(1) = x(1); // y position
    z_pred(2) = x(2); // z position
    z_pred(3) = x(6); // yaw (psi)
    
    return z_pred;
}

/**
 * Adjust yaw measurement to handle 180-degree flips
 * @param psi_meas Measured yaw from OptiTrack
 * @param psi_est Estimated yaw from UKF
 * @return Corrected yaw measurement
 */
double UKF::adjustYawMeasurement(double psi_meas, double psi_est) {
    // Normalize both angles to [-π, π]
    while (psi_meas > M_PI) psi_meas -= 2 * M_PI;
    while (psi_meas < -M_PI) psi_meas += 2 * M_PI;
    
    while (psi_est > M_PI) psi_est -= 2 * M_PI;
    while (psi_est < -M_PI) psi_est += 2 * M_PI;
    
    // Calculate difference
    double diff = psi_meas - psi_est;
    
    // Normalize difference to [-π, π]
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    
    // If difference is large (close to π), we might be experiencing a yaw flip
    if (std::abs(diff) > 0.8 * M_PI) {
        // Add or subtract 180 degrees (π radians)
        psi_meas = psi_meas + (diff > 0 ? -M_PI : M_PI);
        
        // Normalize again
        while (psi_meas > M_PI) psi_meas -= 2 * M_PI;
        while (psi_meas < -M_PI) psi_meas += 2 * M_PI;
    }
    
    return psi_meas;
}

/**
 * Generate sigma points for the UKF prediction step
 */
void UKF::generateSigmaPoints() {
    // Calculate square root of (n_x_ + lambda_) * P_
    Eigen::MatrixXd A = P_.llt().matrixL();
    double scaling = std::sqrt(n_x_ + lambda_);
    A *= scaling;
    
    // Set first sigma point as the mean
    Xsig_.col(0) = x_;
    
    // Generate the remaining sigma points
    for (int i = 0; i < n_x_; i++) {
        Xsig_.col(i + 1) = x_ + A.col(i);
        Xsig_.col(i + 1 + n_x_) = x_ - A.col(i);
    }
}

/**
 * Predict state forward in time
 * @param t Current timestamp in seconds
 */
void UKF::predict(double t) {
    // Calculate time step
    double dt = t - last_predict_time_;
    
    // Skip prediction if dt is zero or negative
    if (dt <= 0) {
        return;
    }
    
    // Generate sigma points
    generateSigmaPoints();
    
    // Propagate sigma points through process model
    Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(n_x_, n_sigma_);
    for (int i = 0; i < n_sigma_; i++) {
        Xsig_pred.col(i) = f(Xsig_.col(i), dt);
    }
    
    // Predict state mean
    x_.setZero();
    for (int i = 0; i < n_sigma_; i++) {
        x_ += weights_mean_[i] * Xsig_pred.col(i);
    }
    
    // Predict state covariance
    P_.setZero();
    for (int i = 0; i < n_sigma_; i++) {
        Eigen::VectorXd x_diff = Xsig_pred.col(i) - x_;
        
        // Normalize yaw angle difference
        while (x_diff(6) > M_PI) x_diff(6) -= 2 * M_PI;
        while (x_diff(6) < -M_PI) x_diff(6) += 2 * M_PI;
        
        P_ += weights_cov_[i] * x_diff * x_diff.transpose();
    }
    
    // Add process noise covariance
    P_ += Q_ * dt; // Scale process noise by dt
    
    // Update sigma points with the new prediction
    Xsig_ = Xsig_pred;
    
    // Update timestamp
    last_predict_time_ = t;
}

/**
 * Update state with OptiTrack measurement
 * @param t Current timestamp in seconds
 * @param z_opti OptiTrack measurement [x, y, z, psi]
 */
void UKF::update(double t, const Eigen::Vector4d& z_opti) {
    // Skip update if measurement is stale (>0.5 seconds old)
    if (last_opti_time_ >= 0 && t - last_opti_time_ > 0.5) {
        std::cerr << "Skipping stale OptiTrack measurement" << std::endl;
        return;
    }
    
    // Create copy of measurement and adjust yaw
    Eigen::Vector4d z = z_opti;
    z(3) = adjustYawMeasurement(z_opti(3), x_(6));
    
    // Transform sigma points to measurement space
    Eigen::MatrixXd Zsig = Eigen::MatrixXd(4, n_sigma_);
    for (int i = 0; i < n_sigma_; i++) {
        Zsig.col(i) = h(Xsig_.col(i));
    }
    
    // Calculate mean predicted measurement
    Eigen::Vector4d z_pred = Eigen::Vector4d::Zero();
    for (int i = 0; i < n_sigma_; i++) {
        z_pred += weights_mean_[i] * Zsig.col(i);
    }
    
    // Calculate measurement covariance matrix S
    Eigen::MatrixXd S = Eigen::MatrixXd(4, 4);
    S.setZero();
    for (int i = 0; i < n_sigma_; i++) {
        Eigen::Vector4d z_diff = Zsig.col(i) - z_pred;
        
        // Normalize yaw angle difference
        while (z_diff(3) > M_PI) z_diff(3) -= 2 * M_PI;
        while (z_diff(3) < -M_PI) z_diff(3) += 2 * M_PI;
        
        S += weights_cov_[i] * z_diff * z_diff.transpose();
    }
    
    // Add measurement noise covariance
    S += R_;
    
    // Calculate cross correlation matrix
    Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, 4);
    Tc.setZero();
    for (int i = 0; i < n_sigma_; i++) {
        Eigen::VectorXd x_diff = Xsig_.col(i) - x_;
        // Normalize yaw angle difference
        while (x_diff(6) > M_PI) x_diff(6) -= 2 * M_PI;
        while (x_diff(6) < -M_PI) x_diff(6) += 2 * M_PI;
        
        Eigen::Vector4d z_diff = Zsig.col(i) - z_pred;
        // Normalize yaw angle difference
        while (z_diff(3) > M_PI) z_diff(3) -= 2 * M_PI;
        while (z_diff(3) < -M_PI) z_diff(3) += 2 * M_PI;
        
        Tc += weights_cov_[i] * x_diff * z_diff.transpose();
    }
    
    // Calculate Kalman gain K
    Eigen::MatrixXd K = Tc * S.inverse();
    
    // Update state mean and covariance
    Eigen::Vector4d z_diff = z - z_pred;
    
    // Normalize yaw angle difference
    while (z_diff(3) > M_PI) z_diff(3) -= 2 * M_PI;
    while (z_diff(3) < -M_PI) z_diff(3) += 2 * M_PI;
    
    // Update state
    x_ = x_ + K * z_diff;
    
    // Normalize yaw angle
    while (x_(6) > M_PI) x_(6) -= 2 * M_PI;
    while (x_(6) < -M_PI) x_(6) += 2 * M_PI;
    
    // Update covariance
    P_ = P_ - K * S * K.transpose();
    
    // Update timestamp
    last_opti_time_ = t;
}