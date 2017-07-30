#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Constructor
 * Initializes Unscented Kalman filter.
 */
UKF::UKF() {
  
  is_initialized_ = false;      // If true, the UKF has been initialized
  use_lidar_ = true;            // If false, LIDAR measurements will be ignored (except during init)
  use_radar_ = true;            // If false, RADAR measurements will be ignored (except during init)

  time_us_ = 0.0;
  
  n_x_ = 5;                     // State dimension (px, py, v, yaw, yawd)
  n_aug_ = 7;                   // Augmented state dimension (px, py, v, yaw, yawd, nu_a, nu_yawdd)
  lambda_ = 3 - n_aug_;         // Sigma point spreading parameter
  NIS_radar_ = 0.0;
  NIS_lidar_ = 0.0;
  
  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  
  for(int i=1; i < 2 * n_aug_ + 1; i++) {     // 2n+1 weights
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }
  
  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // initial augmented sigma points
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  // initial predicted augmented sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;

  // Lidar measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Lidar measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
}

UKF::~UKF() {}

/**
 * Process measurement data.
 *
 * @param {MeasurementPackage} meas_package The latest measurement data of
 *        either radar or lidar.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER && use_lidar_) ||
      (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)) {
    
    //-------------------------------------------------------
    // Initialize UKF with first measurement
    //-------------------------------------------------------
    
    if (!is_initialized_) {
      // init timestamp
      time_us_ = meas_package.timestamp_;
      
      // init covariance matrix
      P_ << 0.15,    0, 0, 0, 0,
               0, 0.15, 0, 0, 0,
               0,    0, 1, 0, 0,
               0,    0, 0, 1, 0,
               0,    0, 0, 0, 1;
      
      if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_lidar_) {
        // init measurement with LIDAR data [x, y] ==> [px, py, v, yaw, yawd]
        x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 1, 1, 0.1;
        
      } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        // init measurement with RADAR data [rho, phi, rho_dot] ==> [px, py, v, yaw, yawd]
        float rho = meas_package.raw_measurements_(0);
        float phi = meas_package.raw_measurements_(1);
        // TODO: calculate v: float rho_dot = meas_package.raw_measurements_(2);
        
        x_ << rho * cos(phi), rho * sin(phi), 1, 1, 0.1;
      }
      
      // initialization done
      is_initialized_ = true;
      return;
    }
    
    //-------------------------------------------------------
    // UKF prediction step
    //-------------------------------------------------------
    
    float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
    time_us_ = meas_package.timestamp_;
    
    Prediction(dt);
    
    //-------------------------------------------------------
    // UKF update step
    //-------------------------------------------------------
    
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package);
    }
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 *
 * @param {double} delta_t the change in time (in seconds) between the last
 *        measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  
  // Generate augmented sigma points Xsig_aug_
  GenerateAugmentedSigmaPoints();
  
  // Predict augmented sigma points Xsig_pred_
  PredictSigmaPoints(delta_t);
  
  // Predict state vector x_ and state covariance matrix P_
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a lidar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  // extract radar measurements (r, phi, r_dot)
  VectorXd z = meas_package.raw_measurements_;
  
  //-------------------------------------------------------
  // Predict RADAR measurements
  //-------------------------------------------------------
  
  int n_z = 3;    // measurement dimension, radar can measure r, phi, and r_dot
  
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    
    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v   = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    
    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;
    
    // measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                           // r
    Zsig(1, i) = atan2(p_y, p_x);                                       // phi
    Zsig(2, i) = (p_x * v1 + p_y * v2 ) / sqrt(p_x * p_x + p_y * p_y);  // r_dot
  }
  
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  
  for (int i=0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    
    VectorXd z_diff = Zsig.col(i) - z_pred;   // residual
    
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  
  R << std_radr_ * std_radr_,                         0,                       0,
                           0, std_radphi_ * std_radphi_,                       0,
                           0,                         0, std_radrd_ * std_radrd_;
  S = S + R;

  //-------------------------------------------------------
  // Update RADAR measurements
  //-------------------------------------------------------
  
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    
    VectorXd z_diff = Zsig.col(i) - z_pred;   //residual
    
    // angle normalization
    while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
    
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    // angle normalization
    while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  // residual
  VectorXd z_diff = z - z_pred;
  
  //angle normalization
  while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
  
  // calculate NIS
  NIS_radar_ = tools_.CalculateNIS(z, z_pred, S);
  
  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}

/**
 * Generates a sigma points matrix in the format [mean, sigma point 1, 
 * sigma point 2,...].
 *
 * @param Xsig_out Calculated sigma points matrix.
 */
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {
  
  // create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
  
  // calculate square root of P
  MatrixXd A = P_.llt().matrixL();
  
  // set first column of sigma point matrix
  Xsig.col(0) = x_;
  
  // set remaining sigma points
  for (int i = 0; i < n_x_; i++)
  {
    Xsig.col(i + 1)        = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }
  
  // print result
  //std::cout << "Xsig = " << std::endl << Xsig << std::endl;
  
  *Xsig_out = Xsig;
}

/**
 * Generates the augmented sigma points matrix.
 */
void UKF::GenerateAugmentedSigmaPoints() {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  
  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;
  
  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug;
  
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  
  // print result
  //std::cout << "Xsig_aug = " << std::endl << Xsig_aug_ << std::endl;
}

/**
 * Predict the augmented sigma points matrix.
 *
 * @param {double} delta_t the change in time (in seconds) between the last
 *        measurement and this one.
 */
void UKF::PredictSigmaPoints(double delta_t) {
  
  //predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // extract values for better readability
    double p_x      = Xsig_aug_(0, i);
    double p_y      = Xsig_aug_(1, i);
    double v        = Xsig_aug_(2, i);
    double yaw      = Xsig_aug_(3, i);
    double yawd     = Xsig_aug_(4, i);
    double nu_a     = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);
    
    // predicted state values
    double px_p, py_p;
    
    // avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    } else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }
    
    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;
    
    // add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;
    
    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;
    
    // write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
  
  //print result
  //std::cout << "Xsig_pred_ = " << std::endl << Xsig_pred_ << std::endl;
}

/**
 * Predict state vector and state covariance matrix.
 */
void UKF::PredictMeanAndCovariance() {
  
  // predicted state mean
  x_.fill(0.0);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  
  //predicted state covariance matrix
  P_.fill(0.0);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {   // iterate over sigma points
    
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    //angle normalization
    while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
    
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }

  //print result
  //std::cout << "Predicted state" << std::endl;
  //std::cout << x_ << std::endl;
  //std::cout << "Predicted covariance matrix" << std::endl;
  //std::cout << P_ << std::endl;
}
