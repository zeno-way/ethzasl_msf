/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
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
#ifndef ODOMETRY_MEASUREMENT_HPP_
#define ODOMETRY_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_core/eigen_utils.h>
#include <nav_msgs/Odometry.h>

namespace msf_updates {
    namespace odometry_measurement {
        enum {
            nMeasurements = 3+3
        };

        /**
         * \brief A measurement as provided by a wheel odometry sensor
         * (x, y, yaw, vx, vy, omega_z)
         */
        typedef msf_core::MSF_Measurement<
            nav_msgs::Odometry,
            Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> OdometryMeasurementBase;
        struct OdometryMeasurement : public OdometryMeasurementBase {
            private:
                typedef OdometryMeasurementBase Measurement_t;
                typedef Measurement_t::Measurement_ptr measptr_t;

                virtual void MakeFromSensorReadingImpl(measptr_t msg) {

                    Eigen::Matrix<double, nMeasurements,
                        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
                    Eigen::Matrix<double, nMeasurements, 1> r_old;

                    H_old.setZero();

                    // Get measurement.
                    z_p_ = (Eigen::Matrix<double, nMeasurements, 1>() <<
                            msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.orientation.z,
                            msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.angular.z
                            ).finished();

                    if (fixed_covariance_)  //  take fix covariance from reconfigure GUI
                    {

                        const double s_zp = n_zp_ * n_zp_;
                        const double s_zy = n_zy_ * n_zy_;
                        const double s_zv = n_zv_ * n_zv_;
                        const double s_zw = n_zw_ * n_zw_;
                        R_ = (Eigen::Matrix<double, nMeasurements, 1>() <<
                                s_zp, s_zp, s_zy,
                                s_zv, s_zv, s_zw)
                            .finished().asDiagonal();

                    } else {  // Tke covariance from sensor.

                        Eigen::Matrix<double, 6, 6> meas_cov(&msg->pose.covariance[0]);

                        R_.block<2, 2>(0, 0) = meas_cov.block<2, 2>(0,0);
                        R_.block<1, 1>(2, 2) = meas_cov.block<1, 1>(5,5);
                        R_.block<2, 2>(3, 3) = meas_cov.block<2, 2>(0,0);
                        R_.block<1, 1>(5, 5) = meas_cov.block<1, 1>(5,5);

                        if (msg->header.seq % 100 == 0) {  // Only do this check from time to time.
                            if (R_.block<3, 3>(0, 0).determinant() < -0.01)
                                MSF_WARN_STREAM_THROTTLE(
                                        60, "The covariance matrix you provided for "
                                        "the position sensor is not positive definite");
                        }
                    }
                }
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                    Eigen::Matrix<double, nMeasurements, 1> z_p_;  /// Position measurement.
                double n_zp_;  /// Position measurement noise.
                double n_zy_;  /// Yaw measurement noise.
                double n_zv_;  /// Velocity measurement noise.
                double n_zw_;  /// Angular rate measurement noise.

                bool fixed_covariance_;
                int fixedstates_;

                typedef msf_updates::EKFState EKFState_T;
                typedef EKFState_T::StateSequence_T StateSequence_T;
                typedef EKFState_T::StateDefinition_T StateDefinition_T;
                virtual ~OdometryMeasurement() {
                }
                OdometryMeasurement(double n_zp, double n_zy, double n_zv, double n_zw,
                        bool fixed_covariance,
                        bool isabsoluteMeasurement, int sensorID, int fixedstates)
                    : OdometryMeasurementBase(isabsoluteMeasurement, sensorID),
                    n_zp_(n_zp),
                    n_zy_(n_zy),
                    n_zv_(n_zv),
                    n_zw_(n_zw),
                    fixed_covariance_(fixed_covariance),
                    fixedstates_(fixedstates) {
                    }
                virtual std::string Type() {
                    return "odometry";
                }

                virtual void CalculateH(
                        shared_ptr<EKFState_T> state_in,
                        Eigen::Matrix<double, nMeasurements,
                        msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {
                    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

                    H.setZero();

                    // Get rotation matrices.
                    Eigen::Matrix<double, 3, 3> C_wo = state.Get<StateDefinition_T::q_wo>()
                        .toRotationMatrix();
                    Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
                        .toRotationMatrix();

                    Eigen::Matrix<double, 3, 3> C_bi = state.Get<StateDefinition_T::q_ib>()
                        .conjugate().toRotationMatrix();

                    // Preprocess for elements in H matrix.
                    Eigen::Matrix<double, 3, 1> vecold;

                    vecold = (-state.Get<StateDefinition_T::p_wo>() + state.Get<StateDefinition_T::p>()
                            + C_q * state.Get<StateDefinition_T::p_ib>());
                    Eigen::Matrix<double, 3, 3> skewold = Skew(vecold);

                    Eigen::Matrix<double, 3, 1> pbi =
                        state.Get<StateDefinition_T::p_ib>();

                    Eigen::Matrix<double, 3, 3> pbi_sk = Skew(pbi);

                    Eigen::Matrix<double, 3, 3> omega_sk = Skew(state.w_m);

                    Eigen::Matrix<double, 3, 1> viw = 
                            state.Get<StateDefinition_T::v>();

                    //Eigen::Matrix<double, 3, 3> viw_sk = Skew(viw);

                    // Get indices of states in error vector.
                    enum {
                        kIdxstartcorr_p = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::p>::value,
                        kIdxstartcorr_v = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::v>::value,
                        kIdxstartcorr_q = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::q>::value,

                        kIdxstartcorr_qwo = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::q_wo>::value,
                        kIdxstartcorr_pwo = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::p_wo>::value,
                        kIdxstartcorr_qib = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::q_ib>::value,
                        kIdxstartcorr_pib = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::p_ib>::value,
                    };

                    // Read the fixed states flags.
                    bool calibposfix = (fixedstates_ & 1 << StateDefinition_T::p_ib);
                    bool calibattfix = (fixedstates_ & 1 << StateDefinition_T::q_ib);
                    bool driftwoattfix = (fixedstates_ & 1 << StateDefinition_T::q_wo);
                    bool driftwoposfix = (fixedstates_ & 1 << StateDefinition_T::p_wo);

                    // Set crosscov to zero for fixed states.
                    if (calibposfix)
                        state_in->ClearCrossCov<StateDefinition_T::p_ib>();
                    if (calibattfix)
                        state_in->ClearCrossCov<StateDefinition_T::q_ib>();
                    if (driftwoattfix)
                        state_in->ClearCrossCov<StateDefinition_T::q_wo>();
                    if (driftwoposfix)
                        state_in->ClearCrossCov<StateDefinition_T::p_wo>();

                    // Construct H matrix.
                    // Position:
                    H.block<2, 3>(0, kIdxstartcorr_p) = C_wo.block<2,3>(0,0);  // p

                    H.block<2, 3>(0, kIdxstartcorr_q) = (-C_wo * C_q * pbi_sk).block<2,3>(0,0);  // q

                    H.block<2, 3>(0, kIdxstartcorr_pib) =
                        calibposfix ?
                        Eigen::Matrix<double, 2, 3>::Zero() :
                        (C_wo * C_q).block<2,3>(0,0).eval();  //p_ib

                    H.block<2, 3>(0, kIdxstartcorr_pwo) =
                        driftwoposfix ?
                        Eigen::Matrix<double, 2, 3>::Zero() :
                        (-Eigen::Matrix<double, 3, 3>::Identity()).block<2,3>(0,0).eval();  //p_wo

                    H.block<2, 3>(0, kIdxstartcorr_qwo) =
                        driftwoattfix ?
                        Eigen::Matrix<double, 2, 3>::Zero() :
                        (-C_wo * skewold).block<2,3>(0,0).eval();  // q_wo

                    // Yaw.
                    H.block<1, 3>(2, kIdxstartcorr_q) = C_bi.block<1,3>(2,0);  // q

                    H.block<1, 3>(2, kIdxstartcorr_qwo) =
                        driftwoattfix ?
                        Eigen::Matrix<double, 1, 3>::Zero() :
                        (C_bi * C_q.transpose()).block<1,3>(2,0).eval();  // q_wo

                    H.block<1, 3>(2, kIdxstartcorr_qib) =
                        calibattfix ?
                        Eigen::Matrix<double, 1, 3>::Zero() :
                        Eigen::Matrix<double, 1, 3>::UnitZ().eval();  //q_ib

                    // Velocity
                    H.block<2, 3>(3, kIdxstartcorr_v) =  (C_bi * C_q).block<2,3>(0,0);  // v

                    H.block<2, 3>(3, kIdxstartcorr_q) =
                        (C_bi*Skew(C_q * viw)).block<2,3>(0,0);  // q

                    H.block<2, 3>(3, kIdxstartcorr_pib) = 
                        calibposfix ?
                        Eigen::Matrix<double, 2, 3>::Zero() : 
                        Skew(C_bi * C_q * viw).block<2,3>(0,0).eval(); // p_ib

                    H.block<2, 3>(3, kIdxstartcorr_qib) = 
                        calibattfix ?
                        Eigen::Matrix<double, 2, 3>::Zero() : 
                        (C_bi * omega_sk + Skew(C_bi * omega_sk * pbi)).block<2,3>(0,0).eval(); // q_ib

                    // Angular Rate
                    H.block<1,3>(5, kIdxstartcorr_qib) =
                        calibattfix ?
                        Eigen::Matrix<double, 1, 3>::Zero() : 
                        (omega_sk).block<1,3>(2,0).eval(); // q_ib

                }

                /**
                 * The method called by the msf_core to apply the measurement represented by this object.
                 */
                virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                        msf_core::MSF_Core<EKFState_T>& core) {


                    if (isabsolute_) {  // Does this measurement refer to an absolute measurement,
                        // or is is just relative to the last measurement.
                        // Get a const ref, so we can read core states
                        const EKFState_T& state = *state_nonconst_new;

                        Eigen::Matrix<double, 3, 3> omega_sk = Skew(state.w_m);

                        // init variables
                        Eigen::Matrix<double, nMeasurements,
                            msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
                        Eigen::Matrix<double, nMeasurements, 1> r_old;

                        CalculateH(state_nonconst_new, H_new);

                        // Get rotation matrices.
                        Eigen::Matrix<double, 3, 3> C_wo = state.Get<StateDefinition_T::q_wo>()
                            .conjugate().toRotationMatrix();

                        Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
                            .conjugate().toRotationMatrix();

                        // Construct residuals.
                        // Position.
                        r_old.block<2, 1>(0, 0) = z_p_.block<2,1>(0,0)
                                - ((C_wo.transpose()
                                    * (-state.Get<StateDefinition_T::p_wo>()
                                        + state.Get<StateDefinition_T::p>()
                                        + C_q.transpose() * state.Get<StateDefinition_T::p_ib>()))).block<2,1>(0,0);

                        // Yaw.
                        Eigen::Quaternion<double> q_err;
                        Eigen::Quaternion<double> z_q(cos(z_p_(2)/2.0), 0.0, 0.0, sin(z_p_(2)/2.0));
                        q_err = (state.Get<StateDefinition_T::q_wo>()
                                * state.Get<StateDefinition_T::q>()
                                * state.Get<StateDefinition_T::q_ib>()).conjugate() * z_q;
                        r_old.block<1, 1>(2, 0) = (q_err.vec() / q_err.w() * 2).block<1,1>(2,0);
                        // Odometry world yaw drift - virtual measurement to fix yaw to this world
                        // q_err = state.Get<StateQwoIdx>();

                        // r_old(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y())
                        //     / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

                        // velocity
                        r_old.block<2,1>(3,0) = z_p_.block<2,1>(3,0) -
                            (state.Get<StateDefinition_T::q_ib>().conjugate()
                                * state.Get<StateDefinition_T::q>()
                                * state.Get<StateDefinition_T::v>() +
                             state.Get<StateDefinition_T::q_ib>().conjugate()*(omega_sk*state.Get<StateDefinition_T::p_ib>())).block<2,1>(0,0);

                        // angular rate z
                        r_old.block<1,1>(5,0) = z_p_.block<1,1>(5,0) -
                            (state.Get<StateDefinition_T::q_ib>().conjugate()
                             *state.w_m).block<1,1>(2,0);

                        if (!CheckForNumeric(r_old, "r_old")) {
                            MSF_ERROR_STREAM("r_old: "<<r_old);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
                        }
                        if (!CheckForNumeric(H_new, "H_old")) {
                            MSF_ERROR_STREAM("H_old: "<<H_new);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
                        }
                        if (!CheckForNumeric(R_, "R_")) {
                            MSF_ERROR_STREAM("R_: "<<R_);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
                        }

                        // Call update step in base class.
                        this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                R_);
                    } else {
                        // Init variables: Get previous measurement.
                        shared_ptr < msf_core::MSF_MeasurementBase<EKFState_T> > prevmeas_base =
                            core.GetPreviousMeasurement(this->time, this->sensorID_);

                        if (prevmeas_base->time == msf_core::constants::INVALID_TIME) {
                            MSF_WARN_STREAM(
                                    "The previous measurement is invalid. Could not apply measurement! " "time:"<<this->time<<" sensorID: "<<this->sensorID_);
                            return;
                        }

                        // Make this a pose measurement.
                        shared_ptr<OdometryMeasurement> prevmeas = dynamic_pointer_cast
                            < OdometryMeasurement > (prevmeas_base);
                        if (!prevmeas) {
                            MSF_WARN_STREAM(
                                    "The dynamic cast of the previous measurement has failed. "
                                    "Could not apply measurement");
                            return;
                        }

                        // Get state at previous measurement.
                        shared_ptr<EKFState_T> state_nonconst_old = core.GetClosestState(
                                prevmeas->time);

                        if (state_nonconst_old->time == msf_core::constants::INVALID_TIME) {
                            MSF_WARN_STREAM(
                                    "The state at the previous measurement is invalid. Could "
                                    "not apply measurement");
                            return;
                        }

                        // Get a const ref, so we can read core states.
                        const EKFState_T& state_new = *state_nonconst_new;
                        const EKFState_T& state_old = *state_nonconst_old;

                        Eigen::Matrix<double, nMeasurements,
                            msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new,
                            H_old;
                        Eigen::Matrix<double, nMeasurements, 1> r_new, r_old;

                        CalculateH(state_nonconst_old, H_old);

                        H_old *= -1;

                        CalculateH(state_nonconst_new, H_new);

                        //TODO (slynen): check that both measurements have the same states fixed!
                        Eigen::Matrix<double, 3, 3> C_wo_old, C_wo_new;
                        Eigen::Matrix<double, 3, 3> C_q_old, C_q_new;

                        C_wo_new = state_new.Get<StateDefinition_T::q_wo>().conjugate().toRotationMatrix();
                        C_q_new = state_new.Get<StateDefinition_T::q>().conjugate()
                            .toRotationMatrix();

                        C_wo_old = state_old.Get<StateDefinition_T::q_wo>().conjugate().toRotationMatrix();
                        C_q_old = state_old.Get<StateDefinition_T::q>().conjugate()
                            .toRotationMatrix();

                        // Construct residuals.
                        // Position:
                        Eigen::Matrix<double, 2, 1> diffprobpos = ((C_wo_new.transpose()
                                    * (-state_new.Get<StateDefinition_T::p_wo>() + state_new.Get<StateDefinition_T::p>()
                                        + C_q_new.transpose() * state_new.Get<StateDefinition_T::p_ib>()))
                                - (C_wo_old.transpose()
                                    * (-state_old.Get<StateDefinition_T::p_wo>() + state_old.Get<StateDefinition_T::p>()
                                        + C_q_old.transpose() * state_old.Get<StateDefinition_T::p_ib>()))).block<2,1>(0,0);


                        Eigen::Matrix<double, 2, 1> diffmeaspos =
                            z_p_.block<2,1>(0,0) - prevmeas->z_p_.block<2,1>(0,0);

                        r_new.block<2, 1>(0, 0) = diffmeaspos - diffprobpos;

                        // Attitude:
                        Eigen::Quaternion<double> diffprobatt = (state_new.Get<StateDefinition_T::q_wo>()
                                * state_new.Get<StateDefinition_T::q>()
                                * state_new.Get<StateDefinition_T::q_ib>()).conjugate()
                            * (state_old.Get<StateDefinition_T::q_wo>()
                                    * state_old.Get<StateDefinition_T::q>()
                                    * state_old.Get<StateDefinition_T::q_ib>());

                        double diffmeasatt = z_p_(2,0) - prevmeas->z_p_(2,0);

                        double y_err = diffmeasatt - diffprobatt.z()/diffprobatt.w()*2;

                        r_new(3, 0) = y_err;
                        // Odometry world yaw drift.
                        //q_err = state_new.Get<StateQwvIdx>();

                        //r_new(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y())
                        //    / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

                        if (!CheckForNumeric(r_old, "r_old")) {
                            MSF_ERROR_STREAM("r_old: "<<r_old);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
                        }
                        if (!CheckForNumeric(H_new, "H_old")) {
                            MSF_ERROR_STREAM("H_old: "<<H_new);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
                        }
                        if (!CheckForNumeric(R_, "R_")) {
                            MSF_ERROR_STREAM("R_: "<<R_);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
                        }

                        // Call update step in base class.
                        this->CalculateAndApplyCorrectionRelative(state_nonconst_old,
                                state_nonconst_new, core, H_old,
                                H_new, r_new, R_);

                    }
                }
        };
    }  // namespace odometry_measurement
}  // namespace msf_updates

#endif  // ODOMETRY_MEASUREMENT_HPP_
