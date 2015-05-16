/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef WHEEL_VISUAL_SENSOR_MANAGER_H
#define WHEEL_VISUAL_SENSOR_MANAGER_H

#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/pose_sensor_handler/pose_sensorhandler.h>
#include <msf_updates/pose_sensor_handler/pose_measurement.h>
#include <msf_updates/odometry_sensor_handler/odometry_sensorhandler.h>
#include <msf_updates/odometry_sensor_handler/odometry_measurement.h>
#include <msf_updates/WheelVisualSensorConfig.h>

namespace msf_updates {

typedef msf_updates::WheelVisualSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class WheelVisualSensorManager : public msf_core::MSF_SensorManagerROS<
    msf_updates::EKFState> {
  typedef WheelVisualSensorManager this_T;
  typedef msf_pose_sensor::PoseSensorHandler<
      msf_updates::pose_measurement::PoseMeasurement<>, this_T> VisualPoseSensorHandler_T;
  typedef msf_odometry_sensor::OdometrySensorHandler<
      msf_updates::odometry_measurement::OdometryMeasurement, this_T> OdometryPoseSensorHandler_T;
  friend class msf_pose_sensor::PoseSensorHandler<
      msf_updates::pose_measurement::PoseMeasurement<>, this_T>;
  public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  WheelVisualSensorManager(
      ros::NodeHandle pnh = ros::NodeHandle("~/wheel_visual_sensor")) {
    imu_handler_.reset(
        new msf_core::IMUHandler_ROS<msf_updates::EKFState>(*this, "msf_core",
                                                            "imu_handler", true));

    bool distortmeas = false;  ///< Distort the pose measurements

    pose_handler_.reset(
        new VisualPoseSensorHandler_T(*this, "", "pose_sensor", distortmeas));
    AddHandler(pose_handler_);

    odometry_handler_.reset(
        new OdometryPoseSensorHandler_T(*this, "", "odometry_sensor"));
    AddHandler(odometry_handler_);


    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&this_T::Config, this, _1,
                                                    _2);
    reconf_server_->setCallback(f);
  }
  virtual ~WheelVisualSensorManager() {
  }

  virtual const Config_T& Getcfg() {
    return config_;
  }

 private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState> > imu_handler_;
  shared_ptr<VisualPoseSensorHandler_T> pose_handler_;
  shared_ptr<OdometryPoseSensorHandler_T> odometry_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_;  ///< Dynamic reconfigure server.

  /**
   * \brief Dynamic reconfigure callback.
   */
  virtual void Config(Config_T &config, uint32_t level) {
    config_ = config;
    pose_handler_->SetNoises(config.pose_noise_meas_p,
                             config.pose_noise_meas_q);
    pose_handler_->SetDelay(config.pose_delay);

    odometry_handler_->SetNoises(config.odometry_noise_meas_p,
                             config.odometry_noise_meas_y,
                             config.odometry_noise_meas_v,
                             config.odometry_noise_meas_w);
    odometry_handler_->SetDelay(config.odometry_delay);

    if ((level & msf_updates::WheelVisualSensor_INIT_FILTER)
        && config.core_init_filter == true) {
      Init(config.pose_initial_scale);
      config.core_init_filter = false;
    }

    // Init call with "set height" checkbox.
    if ((level & msf_updates::WheelVisualSensor_SET_HEIGHT)
        && config.core_set_height == true) {
      Eigen::Matrix<double, 3, 1> p = pose_handler_->GetPositionMeasurement();
      if (p.norm() == 0) {
        MSF_WARN_STREAM(
            "No measurements received yet to initialize position. Height init "
            "not allowed.");
        return;
      }
      double scale = p[2] / config.core_height;
      Init(scale);
      config.core_set_height = false;
    }
  }

  void Init(double scale) const {
    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ic, p_vc, p_wv,
        p_ib, p_ob, p_wo;
    Eigen::Matrix<double, 6, 1> z_o;
    Eigen::Quaternion<double> q, q_wv, q_ic, q_vc, q_ib, q_ob, q_wo;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // init values
    g << msf_core::constants::GRAVITY;	/// Gravity.
    b_w << 0, 0, 0;		/// Bias gyroscopes.
    b_a << 0, 0, 0;		/// Bias accelerometer.

    v << 0, 0, 0;			/// Robot velocity (IMU centered).
    w_m << 0, 0, 0;		/// Initial angular velocity.

    q.setIdentity();  // initial conditions
    p.setZero();

    q_wo.setIdentity();  // World-odometry rotation drift.
    p_wo.setZero();      // World-odometry position drift.

    P.setZero();  // Error state covariance; if zero, a default initialization in msf_core is used.

    p_vc = pose_handler_->GetPositionMeasurement();
    q_vc = pose_handler_->GetAttitudeMeasurement();

    z_o = odometry_handler_->GetOdometryMeasurement();
    p_ob = (Eigen::Matrix<double, 3, 1>() << z_o(0), z_o(1), 0.0).finished();
    double qw = sqrt(1.0 - pow(z_o(2), 2.0));
    q_ob = Eigen::Quaternion<double>(qw, 0.0, 0.0, z_o(2));

    MSF_INFO_STREAM(
        "initial measurement vision: pos:["<<p_vc.transpose()<<"] orientation: " <<STREAMQUAT(q_vc));
    MSF_INFO_STREAM(
        "initial measurement odometry: pos:["<<z_o.transpose()<<"]");

    // Check if we have already input from the measurement sensor.
    if (p_vc.norm() == 0)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize vision position - using [0 0 0]");
    if (z_o.norm() == 0)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize odometry position - using [0 0 0]");
    if (q_vc.w() == 1)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize attitude - using [1 0 0 0]");

    ros::NodeHandle pnh("~");
    pnh.param("wheel_visual_sensor/init/p/x", p[0], 0.0);
    pnh.param("wheel_visual_sensor/init/p/y", p[1], 0.0);
    pnh.param("wheel_visual_sensor/init/p/z", p[2], 0.0);

    pnh.param("wheel_visual_sensor/init/q/w", q.w(), 1.0);
    pnh.param("wheel_visual_sensor/init/q/x", q.x(), 0.0);
    pnh.param("wheel_visual_sensor/init/q/y", q.y(), 0.0);
    pnh.param("wheel_visual_sensor/init/q/z", q.z(), 0.0);
    q.normalize();
    MSF_INFO_STREAM("p: " << p.transpose());
    MSF_INFO_STREAM("q: " << STREAMQUAT(q));

    pnh.param("pose_sensor/init/p_ic/x", p_ic[0], 0.0);
    pnh.param("pose_sensor/init/p_ic/y", p_ic[1], 0.0);
    pnh.param("pose_sensor/init/p_ic/z", p_ic[2], 0.0);

    pnh.param("pose_sensor/init/q_ic/w", q_ic.w(), 1.0);
    pnh.param("pose_sensor/init/q_ic/x", q_ic.x(), 0.0);
    pnh.param("pose_sensor/init/q_ic/y", q_ic.y(), 0.0);
    pnh.param("pose_sensor/init/q_ic/z", q_ic.z(), 0.0);
    q_ic.normalize();

    MSF_INFO_STREAM("p_ic: " << p_ic.transpose());
    MSF_INFO_STREAM("q_ic: " << STREAMQUAT(q_ic));

    pnh.param("odometry_sensor/init/p_ib/x", p_ib[0], 0.0);
    pnh.param("odometry_sensor/init/p_ib/y", p_ib[1], 0.0);
    pnh.param("odometry_sensor/init/p_ib/z", p_ib[2], 0.0);

    pnh.param("odometry_sensor/init/q_ib/w", q_ib.w(), 1.0);
    pnh.param("odometry_sensor/init/q_ib/x", q_ib.x(), 0.0);
    pnh.param("odometry_sensor/init/q_ib/y", q_ib.y(), 0.0);
    pnh.param("odometry_sensor/init/q_ib/z", q_ib.z(), 0.0);
    q_ib.normalize();

    MSF_INFO_STREAM("p_ib: " << p_ib.transpose());
    MSF_INFO_STREAM("q_ib: " << STREAMQUAT(q_ib));

    q_wv = q_vc * q_ic.conjugate() * q.conjugate();

    p_wv = p - q_wv.conjugate().toRotationMatrix() * p_vc / scale
           + q.toRotationMatrix() * p_ic;


    MSF_WARN_STREAM("q " << STREAMQUAT(q));
    MSF_WARN_STREAM("q_wv " << STREAMQUAT(q_wv));

    //TODO (slynen): what if there is no initial position measurement? Then we
    // have to shift vision-world later on, before applying the first position
    // measurement.
    q_wo = q_ob * q_ib.conjugate() * q.conjugate();
    p_wo = p - q_wo.conjugate().toRotationMatrix() * p_ob
        + q.toRotationMatrix() * p_ib;
    // measurement

    a_m = q.inverse() * g;			    /// Initial acceleration.

    //TODO (slynen) Fix this.
    //we want z from vision (we did scale init), so:
//    p(2) = p_vision(2);
//    p_wv(2) = 0;
//    position_handler_->adjustGPSZReference(p(2));

    // Prepare init "measurement"
    // True means that we will also set the initial sensor readings.
    shared_ptr < msf_core::MSF_InitMeasurement<EKFState_T>
        > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->SetStateInitValue < StateDefinition_T::p > (p);
    meas->SetStateInitValue < StateDefinition_T::v > (v);
    meas->SetStateInitValue < StateDefinition_T::q > (q);
    meas->SetStateInitValue < StateDefinition_T::b_w > (b_w);
    meas->SetStateInitValue < StateDefinition_T::b_a > (b_a);
    meas->SetStateInitValue < StateDefinition_T::L
        > (Eigen::Matrix<double, 1, 1>::Constant(scale));
    meas->SetStateInitValue < StateDefinition_T::q_wv > (q_wv);
    meas->SetStateInitValue < StateDefinition_T::p_wv > (p_wv);
    meas->SetStateInitValue < StateDefinition_T::q_ic > (q_ic);
    meas->SetStateInitValue < StateDefinition_T::p_ic > (p_ic);
    meas->SetStateInitValue < StateDefinition_T::q_wo > (q_wo);
    meas->SetStateInitValue < StateDefinition_T::p_wo > (p_wo);
    meas->SetStateInitValue < StateDefinition_T::q_ib > (q_ib);
    meas->SetStateInitValue < StateDefinition_T::p_ib > (p_ib);

    SetStateCovariance(meas->GetStateCovariance());  // Call my set P function.
    meas->Getw_m() = w_m;
    meas->Geta_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // Call initialization in core.
    msf_core_->Init(meas);
  }

  // Prior to this call, all states are initialized to zero/identity.
  virtual void ResetState(EKFState_T& state) const {
    // Set scale to 1.
    Eigen::Matrix<double, 1, 1> scale;
    scale << 1.0;
    state.Set < StateDefinition_T::L > (scale);
  }
  virtual void InitState(EKFState_T& state) const {
    UNUSED(state);
  }

  virtual void CalculateQAuxiliaryStates(EKFState_T& state, double dt) const {
    const msf_core::Vector3 nqwvv = msf_core::Vector3::Constant(
        config_.pose_noise_q_wv);
    const msf_core::Vector3 npwvv = msf_core::Vector3::Constant(
        config_.pose_noise_p_wv);
    const msf_core::Vector3 nqicv = msf_core::Vector3::Constant(
        config_.pose_noise_q_ic);
    const msf_core::Vector3 npicv = msf_core::Vector3::Constant(
        config_.pose_noise_p_ic);
    const msf_core::Vector1 n_L = msf_core::Vector1::Constant(
        config_.pose_noise_scale);
    const msf_core::Vector3 nqwov = msf_core::Vector3::Constant(
        config_.odometry_noise_q_wo);
    const msf_core::Vector3 npwov = msf_core::Vector3::Constant(
        config_.odometry_noise_p_wo);
    const msf_core::Vector3 nqibv = msf_core::Vector3::Constant(
        config_.odometry_noise_q_ib);
    const msf_core::Vector3 npibv = msf_core::Vector3::Constant(
        config_.odometry_noise_p_ib);
 
    // Compute the blockwise Q values and store them with the states,
    // these then get copied by the core to the correct places in Qd.
    state.GetQBlock<StateDefinition_T::L>() = (dt * n_L.cwiseProduct(n_L))
        .asDiagonal();
    state.GetQBlock<StateDefinition_T::q_wv>() =
        (dt * nqwvv.cwiseProduct(nqwvv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_wv>() =
        (dt * npwvv.cwiseProduct(npwvv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_ic>() =
        (dt * nqicv.cwiseProduct(nqicv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_ic>() =
        (dt * npicv.cwiseProduct(npicv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_wo>() =
        (dt * nqwvv.cwiseProduct(nqwov)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_wo>() =
        (dt * npwvv.cwiseProduct(npwov)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_ib>() =
        (dt * nqicv.cwiseProduct(nqibv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_ib>() =
        (dt * npicv.cwiseProduct(npibv)).asDiagonal();
   }

  virtual void SetStateCovariance(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
          EKFState_T::nErrorStatesAtCompileTime>& P) const {
    UNUSED(P);
    // Nothing, we only use the simulated cov for the core plus diagonal for the
    // rest.
  }

  virtual void AugmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(correction);
  }

  virtual void SanityCheckCorrection(
      EKFState_T& delaystate,
      const EKFState_T& buffstate,
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {

    UNUSED(buffstate);
    UNUSED(correction);

    const EKFState_T& state = delaystate;
    if (state.Get<StateDefinition_T::L>()(0) < 0) {
      MSF_WARN_STREAM_THROTTLE(
          1,
          "Negative scale detected: " << state.Get<StateDefinition_T::L>()(0) << ". Correcting to 0.1");
      Eigen::Matrix<double, 1, 1> L_;
      L_ << 0.1;
      delaystate.Set < StateDefinition_T::L > (L_);
    }
  }
};
}
#endif  // WHEEL_VISUAL_SENSOR_MANAGER_H
