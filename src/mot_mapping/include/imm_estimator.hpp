/**
 * @file imm_estimator.hpp
 * @brief Interacting Multiple Model (IMM) Estimator for Dynamic Object Tracking
 * @author FAPP Improvement
 * @date 2024
 * 
 * This file implements an IMM estimator that combines multiple motion models
 * to adaptively track dynamic objects with varying motion patterns.
 * 
 * Innovation: Instead of using a single Constant Velocity (CV) model,
 * we use three models: CV, Constant Acceleration (CA), and Coordinated Turn (CT)
 * to better capture diverse motion patterns in dynamic environments.
 */

#pragma once
#include <ros/ros.h>
#include <queue>
#include <Eigen/Geometry>
#include <cmath>

namespace mot_mapping {

/**
 * @brief Single motion model EKF for use within IMM
 */
struct MotionModelEKF {
    int model_type;  // 0: CV, 1: CA, 2: CT
    double dt;
    ros::Time last_update_stamp_;
    
    Eigen::MatrixXd A;  // State transition matrix
    Eigen::MatrixXd B;  // Control input matrix (for process noise)
    Eigen::MatrixXd H;  // Observation matrix
    Eigen::MatrixXd Q;  // Process noise covariance
    Eigen::MatrixXd R;  // Measurement noise covariance
    Eigen::MatrixXd P;  // State covariance
    Eigen::VectorXd x;  // State vector: [px, py, pz, vx, vy, vz, ...]
    
    double likelihood;  // Model likelihood for IMM mixing
    
    MotionModelEKF() = default;
    
    /**
     * @brief Initialize CV (Constant Velocity) model
     * State: [px, py, pz, vx, vy, vz]
     */
    void initCV(double _dt) {
        model_type = 0;
        dt = _dt;
        
        // State: [px, py, pz, vx, vy, vz]
        x.setZero(6);
        P.setIdentity(6, 6);
        
        // State transition: p' = p + v*dt, v' = v
        A.setIdentity(6, 6);
        A(0, 3) = dt;
        A(1, 4) = dt;
        A(2, 5) = dt;
        
        // Observation matrix (we observe position and velocity)
        H.setIdentity(6, 6);
        
        // Process noise
        Q.setIdentity(6, 6);
        double q_pos = 0.01;   // Position process noise
        double q_vel = 0.5;    // Velocity process noise
        Q.block<3,3>(0,0) *= q_pos;
        Q.block<3,3>(3,3) *= q_vel;
        
        // Measurement noise
        R.setIdentity(6, 6);
        R.block<3,3>(0,0) *= 0.09;  // Position measurement noise
        R.block<3,3>(3,3) *= 0.4;   // Velocity measurement noise
        
        likelihood = 1.0;
    }
    
    /**
     * @brief Initialize CA (Constant Acceleration) model
     * State: [px, py, pz, vx, vy, vz, ax, ay, az]
     */
    void initCA(double _dt) {
        model_type = 1;
        dt = _dt;
        double dt2 = dt * dt / 2.0;
        
        // State: [px, py, pz, vx, vy, vz, ax, ay, az]
        x.setZero(9);
        P.setIdentity(9, 9);
        
        // State transition: p' = p + v*dt + 0.5*a*dt^2, v' = v + a*dt, a' = a
        A.setIdentity(9, 9);
        A(0, 3) = dt; A(0, 6) = dt2;
        A(1, 4) = dt; A(1, 7) = dt2;
        A(2, 5) = dt; A(2, 8) = dt2;
        A(3, 6) = dt;
        A(4, 7) = dt;
        A(5, 8) = dt;
        
        // Observation matrix (we observe position and velocity, not acceleration)
        H.setZero(6, 9);
        H.block<6,6>(0,0).setIdentity();
        
        // Process noise
        Q.setIdentity(9, 9);
        Q.block<3,3>(0,0) *= 0.01;
        Q.block<3,3>(3,3) *= 0.1;
        Q.block<3,3>(6,6) *= 1.0;  // Higher noise for acceleration
        
        // Measurement noise
        R.setIdentity(6, 6);
        R.block<3,3>(0,0) *= 0.09;
        R.block<3,3>(3,3) *= 0.4;
        
        likelihood = 1.0;
    }
    
    /**
     * @brief Initialize CT (Coordinated Turn) model
     * State: [px, py, pz, vx, vy, vz, omega] where omega is turn rate
     */
    void initCT(double _dt) {
        model_type = 2;
        dt = _dt;
        
        // State: [px, py, pz, vx, vy, vz, omega]
        x.setZero(7);
        P.setIdentity(7, 7);
        
        // A will be computed in predict() due to nonlinearity
        A.setIdentity(7, 7);
        
        // Observation matrix
        H.setZero(6, 7);
        H.block<6,6>(0,0).setIdentity();
        
        // Process noise
        Q.setIdentity(7, 7);
        Q.block<3,3>(0,0) *= 0.01;
        Q.block<3,3>(3,3) *= 0.5;
        Q(6, 6) = 0.1;  // Turn rate noise
        
        // Measurement noise
        R.setIdentity(6, 6);
        R.block<3,3>(0,0) *= 0.09;
        R.block<3,3>(3,3) *= 0.4;
        
        likelihood = 1.0;
    }
    
    void predict() {
        if (model_type == 2) {
            // CT model - nonlinear prediction
            double omega = x(6);
            double vx = x(3), vy = x(4);
            
            if (std::abs(omega) > 1e-5) {
                double sin_wt = std::sin(omega * dt);
                double cos_wt = std::cos(omega * dt);
                
                // Update position
                x(0) += (vx * sin_wt + vy * (cos_wt - 1)) / omega;
                x(1) += (-vx * (cos_wt - 1) + vy * sin_wt) / omega;
                x(2) += x(5) * dt;  // z is linear
                
                // Update velocity
                double new_vx = vx * cos_wt - vy * sin_wt;
                double new_vy = vx * sin_wt + vy * cos_wt;
                x(3) = new_vx;
                x(4) = new_vy;
                // vz and omega unchanged
                
                // Jacobian for CT model
                A.setIdentity(7, 7);
                A(0, 3) = sin_wt / omega;
                A(0, 4) = (cos_wt - 1) / omega;
                A(1, 3) = -(cos_wt - 1) / omega;
                A(1, 4) = sin_wt / omega;
                A(2, 5) = dt;
                A(3, 3) = cos_wt;
                A(3, 4) = -sin_wt;
                A(4, 3) = sin_wt;
                A(4, 4) = cos_wt;
            } else {
                // Near-zero turn rate, use CV-like update
                x.head(3) += x.segment(3, 3) * dt;
                A.setIdentity(7, 7);
                A(0, 3) = dt;
                A(1, 4) = dt;
                A(2, 5) = dt;
            }
        } else {
            // CV and CA models - linear prediction
            x = A * x;
        }
        
        P = A * P * A.transpose() + Q;
    }
    
    void update(const Eigen::VectorXd& z) {
        // Innovation
        Eigen::VectorXd y = z - H * x;
        
        // Innovation covariance
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        
        // Kalman gain
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();
        
        // Update state and covariance
        x = x + K * y;
        int n = x.size();
        P = (Eigen::MatrixXd::Identity(n, n) - K * H) * P;
        
        // Compute likelihood for IMM mixing
        double det_S = S.determinant();
        if (det_S > 1e-10) {
            double mahal = y.transpose() * S.inverse() * y;
            likelihood = std::exp(-0.5 * mahal) / std::sqrt(std::pow(2 * M_PI, z.size()) * det_S);
        } else {
            likelihood = 1e-10;
        }
        
        last_update_stamp_ = ros::Time::now();
    }
    
    Eigen::Vector3d pos() const {
        return x.head(3);
    }
    
    Eigen::Vector3d vel() const {
        return x.segment(3, 3);
    }
    
    Eigen::Matrix3d posCov() const {
        return P.block<3, 3>(0, 0);
    }
    
    Eigen::Matrix3d velCov() const {
        return P.block<3, 3>(3, 3);
    }
};

/**
 * @brief Interacting Multiple Model (IMM) Estimator
 * 
 * Combines CV, CA, and CT models to adaptively track objects with varying motion patterns.
 * The model probabilities are updated based on innovation likelihood.
 */
struct IMMEstimator {
    typedef std::shared_ptr<IMMEstimator> Ptr;
    
    int id;
    int age, update_num;
    ros::Time last_update_stamp_;
    
    // Motion models
    MotionModelEKF model_cv;   // Constant Velocity
    MotionModelEKF model_ca;   // Constant Acceleration  
    MotionModelEKF model_ct;   // Coordinated Turn
    
    // Model probabilities
    Eigen::Vector3d mu;        // Model probabilities [P(CV), P(CA), P(CT)]
    Eigen::Matrix3d Pi;        // Model transition probability matrix
    
    // Mixing parameters
    Eigen::Matrix3d mu_ij;     // Mixing probabilities
    
    double dt;
    
    IMMEstimator(double _dt) : dt(_dt) {
        // Initialize models
        model_cv.initCV(dt);
        model_ca.initCA(dt);
        model_ct.initCT(dt);
        
        // Initial model probabilities (favor CV initially)
        mu << 0.6, 0.3, 0.1;
        
        // Model transition probabilities (diagonal-dominant for stability)
        Pi << 0.90, 0.05, 0.05,   // From CV
              0.05, 0.90, 0.05,   // From CA
              0.10, 0.10, 0.80;   // From CT
              
        age = 0;
        update_num = 0;
    }
    
    void reset(const Eigen::Vector3d& pos, int id_) {
        id = id_;
        age = 1;
        update_num = 0;
        
        // Reset all models
        model_cv.initCV(dt);
        model_ca.initCA(dt);
        model_ct.initCT(dt);
        
        // Set initial positions
        model_cv.x.head(3) = pos;
        model_ca.x.head(3) = pos;
        model_ct.x.head(3) = pos;
        
        // Reset model probabilities
        mu << 0.6, 0.3, 0.1;
        
        last_update_stamp_ = ros::Time::now();
    }
    
    /**
     * @brief IMM Mixing Step
     * Compute mixed initial conditions for each model based on current probabilities
     */
    void mixing() {
        // Compute predicted mode probabilities
        Eigen::Vector3d c_bar = Pi.transpose() * mu;
        
        // Compute mixing probabilities
        for (int j = 0; j < 3; ++j) {
            for (int i = 0; i < 3; ++i) {
                mu_ij(i, j) = Pi(i, j) * mu(i) / (c_bar(j) + 1e-10);
            }
        }
        
        // Mix states for each model
        // For simplicity, we extract common states (pos, vel) and mix them
        Eigen::Vector3d pos_cv = model_cv.pos();
        Eigen::Vector3d vel_cv = model_cv.vel();
        Eigen::Vector3d pos_ca = model_ca.pos();
        Eigen::Vector3d vel_ca = model_ca.vel();
        Eigen::Vector3d pos_ct = model_ct.pos();
        Eigen::Vector3d vel_ct = model_ct.vel();
        
        // Mixed positions and velocities for each model
        std::array<Eigen::Vector3d, 3> pos_arr = {pos_cv, pos_ca, pos_ct};
        std::array<Eigen::Vector3d, 3> vel_arr = {vel_cv, vel_ca, vel_ct};
        
        for (int j = 0; j < 3; ++j) {
            Eigen::Vector3d mixed_pos = Eigen::Vector3d::Zero();
            Eigen::Vector3d mixed_vel = Eigen::Vector3d::Zero();
            
            for (int i = 0; i < 3; ++i) {
                mixed_pos += mu_ij(i, j) * pos_arr[i];
                mixed_vel += mu_ij(i, j) * vel_arr[i];
            }
            
            // Update model states
            if (j == 0) {
                model_cv.x.head(3) = mixed_pos;
                model_cv.x.segment(3, 3) = mixed_vel;
            } else if (j == 1) {
                model_ca.x.head(3) = mixed_pos;
                model_ca.x.segment(3, 3) = mixed_vel;
            } else {
                model_ct.x.head(3) = mixed_pos;
                model_ct.x.segment(3, 3) = mixed_vel;
            }
        }
    }
    
    void predict() {
        // Mixing step
        mixing();
        
        // Predict each model
        model_cv.predict();
        model_ca.predict();
        model_ct.predict();
    }
    
    void update(const Eigen::Vector3d& z_pos, const Eigen::Vector3d& z_vel) {
        // Create measurement vector
        Eigen::VectorXd z(6);
        z << z_pos, z_vel;
        
        // Update each model
        model_cv.update(z);
        model_ca.update(z);
        model_ct.update(z);
        
        // Update model probabilities based on likelihoods
        Eigen::Vector3d c_bar = Pi.transpose() * mu;
        Eigen::Vector3d likelihoods(model_cv.likelihood, model_ca.likelihood, model_ct.likelihood);
        
        // Normalize likelihoods to prevent numerical issues
        double max_lik = likelihoods.maxCoeff();
        if (max_lik > 1e-10) {
            likelihoods /= max_lik;
        }
        
        // New model probabilities
        Eigen::Vector3d mu_new;
        for (int j = 0; j < 3; ++j) {
            mu_new(j) = likelihoods(j) * c_bar(j);
        }
        
        // Normalize
        double sum_mu = mu_new.sum();
        if (sum_mu > 1e-10) {
            mu = mu_new / sum_mu;
        }
        
        // Ensure minimum probability for model persistence
        for (int i = 0; i < 3; ++i) {
            if (mu(i) < 0.01) mu(i) = 0.01;
        }
        mu /= mu.sum();  // Re-normalize
        
        last_update_stamp_ = ros::Time::now();
        update_num++;
    }
    
    /**
     * @brief Get combined position estimate (weighted by model probabilities)
     */
    Eigen::Vector3d pos() const {
        return mu(0) * model_cv.pos() + mu(1) * model_ca.pos() + mu(2) * model_ct.pos();
    }
    
    /**
     * @brief Get combined velocity estimate (weighted by model probabilities)
     */
    Eigen::Vector3d vel() const {
        return mu(0) * model_cv.vel() + mu(1) * model_ca.vel() + mu(2) * model_ct.vel();
    }
    
    /**
     * @brief Get combined position covariance (for uncertainty-aware planning)
     */
    Eigen::Matrix3d posCov() const {
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        Eigen::Vector3d mean_pos = pos();
        
        // Add weighted covariances and spread-of-means
        std::array<Eigen::Vector3d, 3> positions = {model_cv.pos(), model_ca.pos(), model_ct.pos()};
        std::array<Eigen::Matrix3d, 3> covs = {model_cv.posCov(), model_ca.posCov(), model_ct.posCov()};
        
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d diff = positions[i] - mean_pos;
            cov += mu(i) * (covs[i] + diff * diff.transpose());
        }
        
        return cov;
    }
    
    /**
     * @brief Get the dominant motion model type
     * @return 0: CV (straight motion), 1: CA (accelerating), 2: CT (turning)
     */
    int dominantModel() const {
        int max_idx = 0;
        mu.maxCoeff(&max_idx);
        return max_idx;
    }
    
    /**
     * @brief Get model probabilities for debugging/visualization
     */
    Eigen::Vector3d modelProbabilities() const {
        return mu;
    }
};

} // namespace mot_mapping
