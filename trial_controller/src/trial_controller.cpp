#include "trial_controller.h"
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>
#include <franka/rate_limiting.h>
#include <qpOASES.hpp>

void myfun(const alglib::real_1d_array &k, double &func, alglib::real_1d_array &grad, void *ptr) {
        func = -100.0*k[0]-1.0*k[1]-1.0*k[2]-1.0*k[3]-1.0*k[4]-1.0*k[5]-1.0*k[6]-1.0*k[7];
        grad[0] = -100.0;
        grad[1] = -1.0;
        grad[2] = -1.0;
        grad[3] = -1.0;
        grad[4] = -1.0;
        grad[5] = -1.0;
        grad[6] = -1.0;
        grad[7] = -1.0;

}


namespace trial_controller_velocity
{
    bool TrialControllerVelocity::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
    {
        if (!node_handle.getParam("only_damping", only_damping_)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get only_damping from parameter server");
            return false;
        }
        if (!only_damping_) {
            if (!node_handle.getParam("small_mass", small_mass_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get small_mass from parameter server");
                return false;
            }
        }
            initOperations.initFrankaVelFT(robot_hw,&state_handle_,&model_handle_,&joint_handles_,FT_sensor,external_force_computation,7);
            if (!node_handle.getParam("m_tr_no_mass", m_tr_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get m_tr_no_mass from parameter server");
                return false;
            }
            if (!node_handle.getParam("m_rot_no_mass", m_rot_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get m_rot_no_mass from parameter server");
                return false;
            }
            if (!node_handle.getParam("d_tr_no_mass", d_tr_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get d_tr_no_mass from parameter server");
                return false;
            }
            if (!node_handle.getParam("d_rot_no_mass", d_rot_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get d_rot_no_mass from parameter server");
                return false;
            }
        
        


        F_ext_EE_0_lowpass_prev_.setZero();

        if (!node_handle.getParam("only_transl", only_transl_)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get only_transl from parameter server");
            return false;
        }
        
        if (!node_handle.getParam("only_x", only_x_)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get only_x from parameter server");
            return false;
        }
        
        dynamic_reconfigure_trial_controller_velocity_param_node_ =
        ros::NodeHandle("dynamic_reconfigure_trial_controller_velocity_param_node");
        dynamic_server_trial_controller_velocity_param_ = std::make_unique<dynamic_reconfigure::Server<trial_controller_velocity::trial_controller_velocity_paramConfig>>( dynamic_reconfigure_trial_controller_velocity_param_node_);
        dynamic_server_trial_controller_velocity_param_->setCallback(boost::bind(&TrialControllerVelocity::trialControllerVelocityParamCallback, this, _1, _2));
        
        return true;
    }
    
    void TrialControllerVelocity::update(const ros::Time&, const ros::Duration& period)
    {
        initOperations.check_initial_bias(FT_sensor);
        data_extraction_.started = true;
        Eigen::Matrix<double,6,1> F_ext_S_s;
        F_ext_S_s = FT_sensor.get_FT_sensor_data();
        franka::RobotState robot_state = state_handle_->getRobotState();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_d(robot_state.q_d.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_d(robot_state.dq_d.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> ddq_d(robot_state.ddq_d.data());
        Eigen::Map<Eigen::Matrix<double, 4, 4>> O_T_EE(robot_state.O_T_EE.data());
        Eigen::Matrix<double,6,1> F_ext_EE_0 = external_force_computation.computeEEPoleBaseFrameExtWrench(O_T_EE,F_ext_S_s,initOperations.bias_checked,initOperations.bias_error,FT_sensor.ecat_error);
        lockingFunction.locking_unlocking(F_ext_EE_0.head(3));
        if (only_transl_) {

            F_ext_EE_0[3] = 0;
            F_ext_EE_0[4] = 0;
            F_ext_EE_0[5] = 0;
        }
        
        Eigen::Matrix<double,7,1> dq_max_safe;
        Eigen::Matrix<double,7,1> dq_min_safe;
        Eigen::Matrix<double,7,7> B_mat = Eigen::Matrix<double,7,7>::Zero();
        std::array<double, 42> J_vector = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,state_handle_->getRobotState().q_d,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> J(J_vector.data());
        Eigen::Matrix<double,7,1> B = Eigen::Matrix<double,7,1>::Zero();
        Eigen::FullPivLU<Eigen::MatrixXd> lu(J);
        B = lu.kernel();
        Eigen::Matrix<double,2,1> velocity_limits;
        for (size_t i = 0; i < 7; i++) {
            velocity_limits = setSafeVelocities(kMaxJointVelocity[i],kMaxJointAcceleration[i],kMaxJointJerk[i],dq_d[i],ddq_d[i]);
            dq_min_safe[i] = velocity_limits[0];
            dq_max_safe[i] = velocity_limits[1];
        }
        Eigen::Matrix<double,6,1> F_ext_EE_0_lowpass;
        F_ext_EE_0_lowpass.setZero();
        Eigen::Matrix<double,7,1> dq_c;
        dq_c.setZero();
        qpOASES::real_t solution[2] = {m_tr_,d_tr_};
        qpOASES::real_t solution_delta[6] = {0,0,0,0,0,0};
        Eigen::Matrix<double,7,1> dq_c_lim = Eigen::Matrix<double,7,1>::Zero();
        bool limits_violated = false;
        double optimization_result = 99;
        double optimization_result_delta = 99;
        for (size_t i=0;i<6;++i){
            F_ext_EE_0_lowpass[i] = franka::lowpassFilter(0.001,F_ext_EE_0[i],F_ext_EE_0_lowpass_prev_[i],1000);
        }
        
        
        
        
        
        if (lockingFunction.locked_) {
            dq_c.setZero();
            for (size_t i = 0; i < 7; i++) {
                if (!(dq_c[i] <= dq_max_safe[i] && dq_c[i] >= dq_min_safe[i])) {
                    limits_violated = true;
                    dq_c_lim(i) = std::max(std::min(dq_c[i], dq_max_safe[i]), dq_min_safe[i]);
                    //lancia funzione
                    
                }
                else {
//lancia funzione 
                    dq_c_lim(i) = dq_c(i);
                }
            }
        }
        else {

                Eigen::Matrix<double,6,1> v_c;
                v_c.setZero();
                std::array<double, 42> J_vector = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,state_handle_->getRobotState().q_d,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
                Eigen::Map<Eigen::Matrix<double, 6, 7>> J(J_vector.data());
                Eigen::Matrix<double,7,1> q_next = q_d+dq_d*T_;
                std::array<double,7> q_next_array;
                for (size_t i = 0; i < 7; i++) {
                    q_next_array[i] = q_next(i);
                }
                std::array<double, 42> J_vector_next = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,q_next_array,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
                Eigen::Map<Eigen::Matrix<double, 6, 7>> J_next(J_vector_next.data());
                Eigen::Matrix<double,6,1> v_prev = J*dq_d;
                for (size_t i = 0; i < 6; i++) {
                    if (i < 3) {
                        v_c[i] = (m_tr_*v_prev[i]+F_ext_EE_0_lowpass(i)*T_)/(m_tr_+d_tr_*T_);
                    }
                    else {
                        v_c[i] = (m_rot_*v_prev[i]+F_ext_EE_0_lowpass(i)*T_)/(m_rot_+d_rot_*T_);
                    }
                }

                std::array<double, 49> B_vector = model_handle_->getMass(q_next_array,state_handle_->getRobotState().I_total,state_handle_->getRobotState().m_total,state_handle_->getRobotState().F_x_Ctotal);
                B_mat = Eigen::Map<Eigen::Matrix<double, 7, 7>>(B_vector.data());
                Eigen::Matrix<double,7,6> J_pinv = B_mat.inverse()*J_next.transpose()*(J_next*B_mat.inverse()*J_next.transpose()).inverse();
                dq_c = J_pinv*v_c;
                for (size_t i = 0; i < 7; i++) {
                    if (!(dq_c[i] <= dq_max_safe[i] && dq_c[i] >= dq_min_safe[i])) {
                        limits_violated = true;
                    }
                }
                if (limits_violated) {
                    double k1 = 1;
                    double k2 = 1;
                    qpOASES::real_t H[2*2];
                    for (size_t i = 0; i < 2; i++) {
                        for (size_t j = 0; j < 2; j++) {
                            if (i==j) {
                                if (i == 0) {
                                    H[i*2+j] = 2*k1;
                                }
                                else if (i == 1) {
                                    H[i*2+j] = 2*k2;
                                }
                            }
                            else {
                                H[i*2+j] = 0;
                            }
                        }
                    }
                    qpOASES::real_t g[2] = {-2*m_tr_,-2*d_tr_};
                    Eigen::Matrix<double,14,2> A_eigen = Eigen::Matrix<double,14,2>::Zero();
                    Eigen::Matrix<double,7,1> B = dq_max_safe-J_pinv.block(0,3,7,3)*(m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
                    Eigen::Matrix<double,7,1> C = dq_min_safe-J_pinv.block(0,3,7,3)*(m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
                    A_eigen.block(0,0,7,1) = J_pinv.block(0,0,7,3)*v_prev.head(3)-B;
                    A_eigen.block(7,0,7,1) = -J_pinv.block(0,0,7,3)*v_prev.head(3)+C;
                    A_eigen.block(0,1,7,1) = -B*T_;
                    A_eigen.block(7,1,7,1) = C*T_;
                    qpOASES::real_t A[14*2];
                    for (size_t i = 0; i < 14; i++) {
                        for (size_t j = 0; j < 2; j++) {
                            A[i*2+j] = A_eigen(i,j);
                        }
                    }
                    Eigen::Matrix<double,14,1> ubA_eigen = Eigen::Matrix<double,14,1>::Zero();
                    ubA_eigen.block(0,0,7,1) = -J_pinv.block(0,0,7,3)*F_ext_EE_0_lowpass.head(3)*T_;
                    ubA_eigen.block(7,0,7,1) = J_pinv.block(0,0,7,3)*F_ext_EE_0_lowpass.head(3)*T_;
                    qpOASES::real_t ubA[14];
                    qpOASES::real_t lbA[14];
                    for (size_t i = 0; i < 14; i++) {
                        ubA[i] = ubA_eigen[i];
                        lbA[i] = -qpOASES::INFTY;
                    }
                    int nWSR = 100;
                    qpOASES::real_t lb[2] = {m_tr_,d_tr_};
                    qpOASES::real_t ub[2] = {qpOASES::INFTY,qpOASES::INFTY};
                    qpOASES::SQProblem opt_problem( 2,14 );
                    qpOASES::returnValue return_value = opt_problem.init( H,g,A,lb,ub,lbA,ubA, nWSR,0,solution );             
                    opt_problem.getPrimalSolution(solution);
                    optimization_result = qpOASES::getSimpleStatus(return_value);
                    Eigen::Matrix<double,6,1> v_adm_lim = Eigen::Matrix<double,6,1>::Zero();
                    v_adm_lim.head(3) = (solution[0]*v_prev.head(3)+F_ext_EE_0_lowpass.head(3)*T_)/(solution[0]+solution[1]*T_);
                    v_adm_lim.tail(3) = (m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
		    dq_c_lim = J_pinv*(v_adm_lim);

                }
                else {

                    dq_c_lim = dq_c;
                }
            
        }
        Eigen::Matrix<double,7,1> q_next = q_d+dq_d*T_;
        Eigen::Matrix<double,7,1> dq_next = (q_next-q_d)/T_;
        Eigen::Matrix<double,7,1> ddq_next = (dq_next-dq_d)/T_;
        
        Eigen::VectorXd q = q_next;
        Eigen::VectorXd dq = dq_next;
        Eigen::VectorXd ddq = ddq_next;
        
                std::array<double,7> q_next_array;
            for (size_t i = 0; i < 7; i++) {
                q_next_array[i] = q_next(i);
            }
        
        Eigen::Matrix<double,6,1> v_c;
        v_c.setZero();
        std::array<double, 42> J_vector_next = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,q_next_array,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> J_next(J_vector_next.data());
        Eigen::Matrix<double,6,1> v_prev = J*dq_d;
                for (size_t i = 0; i < 6; i++) {
                    if (i < 3) {
                        v_c[i] = (m_tr_*v_prev[i]+F_ext_EE_0(i)*T_)/(m_tr_+d_tr_*T_);
                    }
                    else {
                        v_c[i] = (m_rot_*v_prev[i]+F_ext_EE_0(i)*T_)/(m_rot_+d_rot_*T_);
                    }
                }
                

        Eigen::Matrix<double,7,6> J_pinv = B_mat.inverse()*J_next.transpose()*(J_next*B_mat.inverse()*J_next.transpose()).inverse();
        Eigen::VectorXd dq_task;
        dq_task = J_pinv*v_c;
        

        
        Eigen::VectorXd dq_null;
        dq_null = null_space_computation_gpm(J_next,J_pinv ,q, dq, ddq, dq_task,deactivation,deactivation1,deactivation2,deactivation3,deactivation4,deactivation5,deactivation6,deactivation7,dq_d,ddq_d);

        
        dq_c_lim = dq_c_lim+dq_null;
        
        F_ext_EE_0_lowpass_prev_ = F_ext_EE_0_lowpass;
        for (size_t i = 0; i < 7; i++) {
            joint_handles_[i].setCommand(dq_c_lim(i));
        }
        
        Eigen::Matrix<double,2,1> u_eigen;
        u_eigen.setZero();
        std::vector<Eigen::VectorXd> custom_data(11);
        custom_data[0] = F_ext_EE_0_lowpass;
        custom_data[1] = dq_c_lim;
        custom_data[2] = dq_max_safe;
        custom_data[3] = dq_min_safe;
        Eigen::Matrix<double,49,1> B_next_vector;
        for (size_t i = 0; i < 7; i++) {
            for (size_t j = 0; j < 7; j++) {
                B_next_vector(i*7+j) = B_mat(i,j);
            }
        }
        custom_data[4] = B_next_vector;
        Eigen::Matrix<double,2,1> optimization_vars_vector;
        optimization_vars_vector[0] = solution[0];
        optimization_vars_vector[1] = solution[1];
        custom_data[5] = optimization_vars_vector;
        Eigen::Matrix<double,6,1> optimization_vars_delta_vector;
        optimization_vars_delta_vector[0] = solution_delta[0];
        optimization_vars_delta_vector[1] = solution_delta[1];
        optimization_vars_delta_vector[2] = solution_delta[2];
        optimization_vars_delta_vector[3] = solution_delta[3];
        optimization_vars_delta_vector[4] = solution_delta[4];
        optimization_vars_delta_vector[5] = solution_delta[5];
        custom_data[6] = optimization_vars_delta_vector;
        Eigen::Matrix<double,1,1> optimization_result_vector;
        optimization_result_vector[0] = optimization_result;
        custom_data[7] = optimization_result_vector;
        Eigen::Matrix<double,1,1> optimization_result_delta_vector;
        optimization_result_delta_vector[0] = optimization_result_delta;
        custom_data[8] = optimization_result_delta_vector;
        Eigen::Matrix<double,1,1> limits_violated_vector;
        limits_violated_vector[0] = limits_violated;
        custom_data[9] = limits_violated_vector;
        Eigen::Matrix<double,1,1> locked_vector;
        locked_vector[0] = lockingFunction.locked_;
        custom_data[10] = locked_vector;
        data_extraction_.update_data(&state_handle_,&model_handle_,custom_data);
    }
    
    void TrialControllerVelocity::stopping(const ros::Time&)
    {
        std::vector<std::string> custom_header_values{"F_ext_EE_0_lowpass","dq_commanded","dq_max_safe","dq_min_safe","B_next","optimization_vars","optimization_vars_delta","optimization_result","optimization_result_delta","limits_violated","locked"};
        data_extraction_.write_data_to_csv(custom_header_values);
        data_extraction_.started = false;
    }
    
    Eigen::Matrix<double,2,1> TrialControllerVelocity::setSafeVelocities(double max_velocity,double max_acceleration,double max_jerk,double last_commanded_velocity,double last_commanded_acceleration) {
        double safe_max_acceleration = std::max(std::min({
            (max_jerk / max_acceleration) * (max_velocity - last_commanded_velocity), max_acceleration,last_commanded_acceleration+max_jerk*kDeltaT}),last_commanded_acceleration-max_jerk*kDeltaT);
        double safe_min_acceleration = std::min(std::max({
            (max_jerk / max_acceleration) * (-max_velocity - last_commanded_velocity), -max_acceleration,last_commanded_acceleration-max_jerk*kDeltaT}),last_commanded_acceleration+max_jerk*kDeltaT);
        Eigen::Matrix<double,2,1> safe_velocities;
        safe_velocities.setZero();
        safe_velocities[0] = last_commanded_velocity+safe_min_acceleration*kDeltaT;
        safe_velocities[1] = last_commanded_velocity+safe_max_acceleration*kDeltaT;
        return safe_velocities;
    }
    
    void TrialControllerVelocity::trialControllerVelocityParamCallback(
    trial_controller_velocity::trial_controller_velocity_paramConfig& config,
    uint32_t /*level*/) {
        if (!dyn_params_set) {
            config.m_tr = m_tr_;
            config.m_rot = m_rot_;
            config.d_tr = d_tr_;
            config.d_rot = d_rot_;
            dyn_params_set = true;
        }
        else {
            m_tr_ = config.m_tr;
            m_rot_ = config.m_rot;
            d_tr_ = config.d_tr;
            d_rot_ = config.d_rot;
        }
    }
    
            Eigen::VectorXd TrialControllerVelocity::derivative(Eigen::VectorXd& q) {
 
        Eigen::VectorXd out(49);
        out(0) = q(0) * 1.0 / std::pow(q(0) * q(0) * 2.978194627446728e-2 + 1.0e-4, 2) * (-5.956389254893456e-2);
        out(8) = q(1) * 1.0 / std::pow(q(1) * q(1) * 8.045145909083209e-2 + 1.0e-4, 2) * (-1.609029181816642e-1);
        out(16) = q(2) * 1.0 / std::pow(q(2) * q(2) * 2.978194627446728e-2 + 1.0e-4, 2) * (-5.956389254893456e-2);
        out(24) = -(q(3) * 2.21926221959067e-1 + 3.486017094533025e-1) * 1.0 / std::pow((q(3) * 3.331112591605596e-1 + 5.232511658894071e-1), 2 + 1.0e-4);
        out(32) = q(4) * 1.0 / std::pow(q(4) * q(4) * 2.978194627446728e-2 + 1.0e-4, 2) * (-5.956389254893456e-2);
        out(40) = -(q(5) * 1.407172357506209e-1 - 2.627894377642846e-1) * 1.0 / std::pow((q(5) * (1.0e+2 / 3.77e+2) - 4.953580901856764e-1), 2 + 1.0e-4);
        out(48) = q(6) * 1.0 / std::pow(q(6) * q(6) * 2.978194627446728e-2 + 1.0e-4, 2) * (-5.956389254893456e-2);
        out.segment(1, 7).setZero();
        out.segment(9, 7).setZero();
        out.segment(17, 7).setZero();
        out.segment(26, 7).setZero();
        out.segment(34, 7).setZero();
        out.segment(42, 7).setZero();
        return out;
}
        double TrialControllerVelocity::calculateManipulability(const Eigen::MatrixXd& J) {
 
        Eigen::MatrixXd JT = J.transpose();
        Eigen::MatrixXd JJt = J * JT;
        double det = JJt.determinant();
        return std::sqrt(det);
}


        void TrialControllerVelocity::nullspace_managment_1(
        double nullspace_scaling1, double nullspace_scaling2, double nullspace_scaling3, 
        double nullspace_scaling4, double nullspace_scaling5, double nullspace_scaling6, 
        double nullspace_scaling7, double nullspace_scaling, 
        double& p1, double& p2, double& p3, double& p4, 
        double& p5, double& p6, double& p7, double& p8)
{
 
        double total_scaling = nullspace_scaling1 + nullspace_scaling2 + nullspace_scaling3 +
                          nullspace_scaling4 + nullspace_scaling5 + nullspace_scaling6 +
                          nullspace_scaling7 + nullspace_scaling;
    

        p1 = nullspace_scaling1 / total_scaling;
        p2 = nullspace_scaling2 / total_scaling;
        p3 = nullspace_scaling3 / total_scaling;
        p4 = nullspace_scaling4 / total_scaling;
        p5 = nullspace_scaling5 / total_scaling;
        p6 = nullspace_scaling6 / total_scaling;
        p7 = nullspace_scaling7 / total_scaling;
        p8 = nullspace_scaling / total_scaling;
}




        Eigen::VectorXd TrialControllerVelocity::distance_joint_limit(const Eigen::VectorXd& q) {
        Eigen::VectorXd max_position(7);
        max_position << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
        Eigen::VectorXd min_position(7);
        min_position << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        Eigen::VectorXd qm = (max_position + min_position) / 2;
        Eigen::VectorXd output(7);
    
        for (int i = 0; i < 7; ++i) {
            double dj = std::pow((q(i) - qm(i)) / (max_position(i) - min_position(i)), 2);
            output(i) = 1.0 / (dj + 1e-4);
        }

        return output;
}


        Eigen::Matrix<double,4,1> TrialControllerVelocity::setSafeValues(double max_velocity,double max_acceleration,double max_jerk,double last_commanded_velocity,double last_commanded_acceleration) {
        double safe_max_acceleration = std::max(std::min({
            (max_jerk / max_acceleration) * (max_velocity - last_commanded_velocity), max_acceleration,last_commanded_acceleration+max_jerk*kDeltaT}),last_commanded_acceleration-max_jerk*kDeltaT);
        double safe_min_acceleration = std::min(std::max({
            (max_jerk / max_acceleration) * (-max_velocity - last_commanded_velocity), -max_acceleration,last_commanded_acceleration-max_jerk*kDeltaT}),last_commanded_acceleration+max_jerk*kDeltaT);
        Eigen::Matrix<double,4,1> safe_values;
        safe_values.setZero();
        safe_values[0] = last_commanded_velocity+safe_min_acceleration*kDeltaT;
        safe_values[1] = last_commanded_velocity+safe_max_acceleration*kDeltaT;
        safe_values[2] =  safe_max_acceleration;
        safe_values[3] =  safe_min_acceleration;
        return safe_values;
    }


    
        Eigen::VectorXd TrialControllerVelocity::null_space_computation_gpm(const Eigen::MatrixXd &J,const Eigen::MatrixXd &J_pinv, Eigen::VectorXd &q, const Eigen::VectorXd &dq, 
                                    const Eigen::VectorXd &ddq, const Eigen::VectorXd &dq_task,bool deactivation,bool deactivation1,bool deactivation2,bool deactivation3,bool deactivation4,bool deactivation5,bool deactivation6,bool deactivation7,
                                    const Eigen::VectorXd &dq_d,const Eigen::VectorXd &ddq_d) {
        
        
        double initial_nullspace_value = 0.04;
        double final_nullspace_value = 0.03;
        double initial_nullspace_value1 = 19;
        double final_nullspace_value1 = 9;


        double T = 0.1; 

        double deltaq = 1e-4;
        double w0 = calculateManipulability(J);


        Eigen::VectorXd dwkin(q.size()); 

        for (int a = 0; a < q.size(); ++a) {
            Eigen::VectorXd qDq = q;
            qDq(a) += deltaq;
            
            std::array<double,7> q_d_array;
            for (size_t i = 0; i < 7; i++) {
                q_d_array[i] = qDq(i);
            }
        
        
            std::array<double, 42> J_vector_next = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,q_d_array,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> Jd(J_vector_next.data());
            

            
            dwkin(a) = calculateManipulability(Jd) - w0;
        }

        Eigen::VectorXd optimal_vector = dwkin / deltaq;
        Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
        Eigen::MatrixXd J_transpose = J.transpose();

        Eigen::VectorXd output = derivative(q);

        Eigen::VectorXd optimal_vector1(7);
        optimal_vector1 <<  output(0), output(1), output(2), output(3),output(4),output(5),output(6);
        Eigen::VectorXd optimal_vector2(7);
        optimal_vector2 << output(7), output(8), output(9), output(10),output(11),output(12),output(13);
        Eigen::VectorXd optimal_vector3(7);
        optimal_vector3 << output(14), output(15), output(16), output(17),output(18),output(19),output(20);
        Eigen::VectorXd optimal_vector4(7);
        optimal_vector4 <<output(21), output(22), output(23), output(24),output(25),output(26),output(27);
        Eigen::VectorXd optimal_vector5(7);
        optimal_vector5 <<  output(28), output(29), output(30), output(31),output(32),output(33),output(34);
        Eigen::VectorXd optimal_vector6(7);
        optimal_vector6 << output(35), output(36), output(37), output(38),output(39),output(40),output(41);
        Eigen::VectorXd optimal_vector7(7); 
        optimal_vector7 << output(42), output(43), output(44), output(45), output(46), output(47), output(48);
    
        Eigen::VectorXd v_a(7);
        v_a << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;

        Eigen::VectorXd v(7); 
        v = (I-J_pinv*J)*optimal_vector+v_a;

        Eigen::VectorXd v1(7); 
        v1 = (I-J_pinv*J)*optimal_vector1+v_a;
        
        Eigen::VectorXd v2(7); 
        v2 = (I-J_pinv*J)*optimal_vector2+v_a;

        Eigen::VectorXd v3(7); 
        v3 = (I-J_pinv*J)*optimal_vector3+v_a;

        Eigen::VectorXd v4(7); 
        v4 = (I-J_pinv*J)*optimal_vector4+v_a;

        Eigen::VectorXd v5(7); 
        v5 = (I-J_pinv*J)*optimal_vector5+v_a;

        Eigen::VectorXd v6(7); 
        v6 = (I-J_pinv*J)*optimal_vector6+v_a;

        Eigen::VectorXd v7(7); 
        v7 = (I-J_pinv*J)*optimal_vector7+v_a;
        
        Eigen::Matrix<double,7,1> dq_max_safe;
        Eigen::Matrix<double,7,1> dq_min_safe;
        Eigen::Matrix<double,7,1> ddq_max_safe;
        Eigen::Matrix<double,7,1> ddq_min_safe;
        Eigen::Matrix<double,4,1> limits;
        for (size_t i = 0; i < 7; i++) {
            limits = setSafeValues(kMaxJointVelocity[i],kMaxJointAcceleration[i],kMaxJointJerk[i],dq_d[i],ddq_d[i]);
            dq_min_safe[i] = limits[0];
            dq_max_safe[i] = limits[1];
            ddq_max_safe[i] = limits[2];
            ddq_min_safe[i] = limits[3];
        }
        
        
        Eigen::VectorXd max_velocity(7);
        Eigen::VectorXd max_jerk(7);
        
        max_velocity << 2.129003, 2.151503, 2.144003, 2.136503, 2.564003, 2.549003, 2.549003;
        max_jerk << 7499.999, 3749.999, 4999.999, 6249.999, 7499.999, 9999.999, 9999.999;

        double c1[7];
        for (int i=0; i<7; ++i){
        c1[i] = ( dq_max_safe[i]-dq_task[i]);}
        double c2[7];
        for (int i=0; i<7; ++i){
        c2[i] = (dq_min_safe[i]-dq_task[i]);}
        double c3[7];
        for (int i=0; i<7; ++i){
        c3[i] = (ddq_max_safe[i]*T-dq_task[i]+dq[i]);}
        double c4[7];
        for (int i=0; i<7; ++i){
        c4[i] = (ddq_min_safe[i]*T-dq_task[i]+dq[i]);}
        double c5[7];
        for (int i=0; i<7; ++i){
        c5[i] = ((+max_jerk[i]*T+ddq[i])*T+dq[i]-dq_task[i]);}
        double c6[7];
        for (int i=0; i<7; ++i){
        c6[i] = ((-max_jerk[i]*T+ddq[i])*T+dq[i]-dq_task[i]);}


    Eigen::VectorXd dj = distance_joint_limit(q);

    if (w0 <= initial_nullspace_value)
    {
    deactivation = true;
    }

    if (dj[0] <= initial_nullspace_value1)
    {
    deactivation1 = true;
    }

    if (dj[1] <= initial_nullspace_value1)
    {
    deactivation2 = true;
    }

    if (dj[2] <= initial_nullspace_value1)
    {
    deactivation3 = true;
    }

    if (dj[3] <= initial_nullspace_value1)
    {
    deactivation4 = true;
    }

    if (dj[4] <= initial_nullspace_value1)
    {
    deactivation5 = true;
    }

    if (dj[5] <= initial_nullspace_value1)
    {
    deactivation6 = true;
    }

    if (dj[6] <= initial_nullspace_value1)
    {
    deactivation7 = true;
    }




    double nullspace_scaling = 0;
    double nullspace_scaling1 = 0;
    double nullspace_scaling2 = 0;
    double nullspace_scaling3 = 0;
    double nullspace_scaling4 = 0;
    double nullspace_scaling5 = 0;
    double nullspace_scaling6 = 0;
    double nullspace_scaling7 = 0;


    if (deactivation == true)
    {    
                if (w0 <= final_nullspace_value){
                        nullspace_scaling = 1;
                }else if (final_nullspace_value < w0 && w0 <= initial_nullspace_value){
                        nullspace_scaling = 1 / (final_nullspace_value - initial_nullspace_value) * w0 + initial_nullspace_value / (initial_nullspace_value - final_nullspace_value);}

    }


    if (deactivation1 == true)
    {    
                if (dj[0] <= final_nullspace_value1){
                        nullspace_scaling1 = 1;
                }else if (final_nullspace_value1 < dj[0] && dj[0] <= initial_nullspace_value1){
                        nullspace_scaling1 = 1 / (final_nullspace_value1 - initial_nullspace_value1) * dj[0] + initial_nullspace_value1 / (initial_nullspace_value1 - final_nullspace_value1);}

    }

    if (deactivation2 == true)
    {    
                if (dj[1] <= final_nullspace_value1){
                        nullspace_scaling2 = 1;
                }else if (final_nullspace_value1 < dj[1] && dj[1] <= initial_nullspace_value1){
                        nullspace_scaling2 = 1 / (final_nullspace_value1 - initial_nullspace_value1) * dj[1] + initial_nullspace_value1 / (initial_nullspace_value1 - final_nullspace_value1);}

    }
    if (deactivation3 == true)
    {    
                if (dj[2] <= final_nullspace_value1){
                        nullspace_scaling3 = 1;
                }else if (final_nullspace_value1 < dj[2] && dj[2] <= initial_nullspace_value1){
                        nullspace_scaling3 = 1 / (final_nullspace_value1 - initial_nullspace_value1) * dj[2] + initial_nullspace_value1 / (initial_nullspace_value1 - final_nullspace_value1);}

    }
    if (deactivation4 == true)
    {    
                if (dj[3] <= final_nullspace_value1){
                        nullspace_scaling4 = 1;
                }else if (final_nullspace_value1 < dj[3] && dj[3] <= initial_nullspace_value1){
                        nullspace_scaling4 = 1 / (final_nullspace_value1 - initial_nullspace_value1) * dj[3] + initial_nullspace_value1 / (initial_nullspace_value1 - final_nullspace_value1);}

    }

    if (deactivation5 == true)
    {    
                if (dj[4] <= final_nullspace_value1){
                        nullspace_scaling5 = 1;
                }else if (final_nullspace_value1 < dj[4] && dj[4] <= initial_nullspace_value1){
                        nullspace_scaling5 = 1 / (final_nullspace_value1 - initial_nullspace_value1) * dj[4] + initial_nullspace_value1 / (initial_nullspace_value1 - final_nullspace_value1);}

    }

    if (deactivation6 == true)
    {    
                if (dj[5] <= final_nullspace_value1){
                        nullspace_scaling6 = 1;
                }else if (final_nullspace_value1 < dj[5] && dj[5] <= initial_nullspace_value1){
                        nullspace_scaling6 = 1 / (final_nullspace_value1 - initial_nullspace_value1) * dj[5] + initial_nullspace_value1 / (initial_nullspace_value1 - final_nullspace_value1);}

    }

    if (deactivation7 == true)
    {    
                if (dj[6] <= final_nullspace_value1){
                        nullspace_scaling7 = 1;
                }else if (final_nullspace_value1 < dj[6] && dj[6] <= initial_nullspace_value1){
                        nullspace_scaling7 = 1 / (final_nullspace_value1 - initial_nullspace_value1) * dj[6] + initial_nullspace_value1 / (initial_nullspace_value1 - final_nullspace_value1);}

    }
    double p1, p2, p3, p4, p5, p6, p7, p8;

    if (nullspace_scaling1!= 0 || nullspace_scaling2!= 0 || nullspace_scaling3!= 0 || nullspace_scaling4!= 0 || nullspace_scaling5!= 0 || nullspace_scaling6!= 0 || nullspace_scaling7!= 0 || nullspace_scaling!= 0)
    {
        nullspace_managment_1(nullspace_scaling1, nullspace_scaling2, nullspace_scaling3, 
                        nullspace_scaling4, nullspace_scaling5, nullspace_scaling6, 
                        nullspace_scaling7, nullspace_scaling, p1, p2, p3, p4, p5, p6, p7, p8);

    }

    else{
        p1 = 0;
        p2 = 0;
        p3 = 0;
        p4 = 0;
        p5 = 0;
        p6 = 0;
        p7 = 0;
        p8 = 0;}

    Eigen::VectorXd max_velocity_nullspace = p8*nullspace_scaling * max_velocity;
    Eigen::VectorXd min_velocity_nullspace = p8*nullspace_scaling * -max_velocity;
    Eigen::VectorXd max_velocity_nullspace1 = p1*nullspace_scaling1 * max_velocity;
    Eigen::VectorXd min_velocity_nullspace1 = p1*nullspace_scaling1 * -max_velocity;
    Eigen::VectorXd max_velocity_nullspace2 = p2*nullspace_scaling2 * max_velocity;
    Eigen::VectorXd min_velocity_nullspace2 = p2*nullspace_scaling2 * -max_velocity;
    Eigen::VectorXd max_velocity_nullspace3 = p3*nullspace_scaling3 * max_velocity;
    Eigen::VectorXd min_velocity_nullspace3 = p3*nullspace_scaling3 * -max_velocity;
    Eigen::VectorXd max_velocity_nullspace4 = p4*nullspace_scaling4 * max_velocity;
    Eigen::VectorXd min_velocity_nullspace4 = p4*nullspace_scaling4 * -max_velocity;
    Eigen::VectorXd max_velocity_nullspace5 = p5*nullspace_scaling5 * max_velocity;
    Eigen::VectorXd min_velocity_nullspace5 = p5*nullspace_scaling5 * -max_velocity;
    Eigen::VectorXd max_velocity_nullspace6 = p6*nullspace_scaling6 * max_velocity;
    Eigen::VectorXd min_velocity_nullspace6 = p6*nullspace_scaling6 * -max_velocity;
    Eigen::VectorXd max_velocity_nullspace7 = p7*nullspace_scaling7 * max_velocity;
    Eigen::VectorXd min_velocity_nullspace7 = p7*nullspace_scaling7 * -max_velocity;





    double c7[7];
    for (int i=0; i<7; ++i){
    c7[i] = +max_velocity_nullspace[i];}
    double c8[7];
    for (int i=0; i<7; ++i){
    c8[i] = min_velocity_nullspace[i];}
    double c9[7];
    for (int i=0; i<7; ++i){
    c9[i] = +max_velocity_nullspace1[i];}
    double c10[7];
    for (int i=0; i<7; ++i){
    c10[i] = min_velocity_nullspace1[i];}
    double c11[7];
    for (int i=0; i<7; ++i){
    c11[i] = +max_velocity_nullspace2[i];}
    double c12[7];
    for (int i=0; i<7; ++i){
    c12[i] = min_velocity_nullspace2[i];}
    double c13[7];
    for (int i=0; i<7; ++i){
    c13[i] = +max_velocity_nullspace3[i];}
    double c14[7];
    for (int i=0; i<7; ++i){
    c14[i] = min_velocity_nullspace3[i];}

    double c15[7];
    for (int i=0; i<7; ++i){
    c15[i] = +max_velocity_nullspace4[i];}
    double c16[7];
    for (int i=0; i<7; ++i){
    c16[i] = min_velocity_nullspace4[i];}
    double c17[7];
    for (int i=0; i<7; ++i){
    c17[i] = +max_velocity_nullspace5[i];}
    double c18[7];
    for (int i=0; i<7; ++i){
    c18[i] = min_velocity_nullspace5[i];}
    double c19[7];
    for (int i=0; i<7; ++i){
    c19[i] = +max_velocity_nullspace6[i];}
    double c20[7];
    for (int i=0; i<7; ++i){
    c20[i] = min_velocity_nullspace6[i];}
    double c21[7];
    for (int i=0; i<7; ++i){
    c21[i] = +max_velocity_nullspace7[i];}
    double c22[7];
    for (int i=0; i<7; ++i){
    c22[i] = min_velocity_nullspace7[i];
    }



    alglib::real_2d_array c;
    alglib::integer_1d_array ct;

    c.setlength(154, 9);
    ct.setlength(154);


    for (int i = 0; i < 7; i++) {
            c(i, 0) = v[i];
            c(i, 1) = v1[i];
            c(i, 2) = v2[i];
            c(i, 3) = v3[i];
            c(i, 4) = v4[i];
            c(i, 5) = v5[i];
            c(i, 6) = v6[i];
            c(i, 7) = v7[i];
            c(i, 8) = c1[i];
            ct(i) = -1;
        }

        for (int i = 7; i < 14; i++) {
            c(i, 0) = v[i-7];
            c(i, 1) = v1[i-7];
            c(i, 2) = v2[i-7];
            c(i, 3) = v3[i-7];
            c(i, 4) = v4[i-7];
            c(i, 5) = v5[i-7];
            c(i, 6) = v6[i-7];
            c(i, 7) = v7[i-7];
            c(i, 8) = c2[i-7];
            ct(i) = 1;
        }


        for (int i = 14; i < 21; i++) {
            c(i, 0) = v[i-14];
            c(i, 1) = v1[i-14];
            c(i, 2) = v2[i-14];
            c(i, 3) = v3[i-14];
            c(i, 4) = v4[i-14];
            c(i, 5) = v5[i-14];
            c(i, 6) = v6[i-14];
            c(i, 7) = v7[i-14];
            c(i, 8) = c3[i-14];
            ct(i) = -1;
        }
        for (int i = 21; i < 28; i++) {
            c(i, 0) = v[i-21];
            c(i, 1) = v1[i-21];
            c(i, 2) = v2[i-21];
            c(i, 3) = v3[i-21];
            c(i, 4) = v4[i-21];
            c(i, 5) = v5[i-21];
            c(i, 6) = v6[i-21];
            c(i, 7) = v7[i-21];
            c(i, 8) = c4[i-21];
            ct(i) = 1;
        }

        for (int i = 28; i < 35; i++) {
            c(i, 0) = v[i-28];
            c(i, 1) = v1[i-28];
            c(i, 2) = v2[i-28];
            c(i, 3) = v3[i-28];
            c(i, 4) = v4[i-28];
            c(i, 5) = v5[i-28];
            c(i, 6) = v6[i-28];
            c(i, 7) = v7[i-28];
            c(i, 8) = c5[i-28];
            ct(i) = -1;
        }

        for (int i = 35; i < 42; i++) {
            c(i, 0) = v[i-35];
            c(i, 1) = v1[i-35];
            c(i, 2) = v2[i-35];
            c(i, 3) = v3[i-35];
            c(i, 4) = v4[i-35];
            c(i, 5) = v5[i-35];
            c(i, 6) = v6[i-35];
            c(i, 7) = v7[i-35];
            c(i, 8) = c6[i-35];
            ct(i) = 1;
        }

    for (int i = 42; i < 49; i++) {
            c(i, 0) = v[i-42];
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = 0;
            c(i, 8) = c7[i-42];
            ct(i) = -1;
        }

        for (int i = 49; i < 56; i++) {
            c(i, 0) = v[i-49];
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = 0;
            c(i, 8) = c8[i-49];
            ct(i) = 1;
        }

        for (int i = 56; i < 63; i++) {
            c(i, 0) = 0;
            c(i, 1) = v1[i-56];
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = 0;
            c(i, 8) = c9[i-56];
            ct(i) = -1;
        }

        for (int i = 63; i < 0; i++) {
            c(i, 0) = 0;
            c(i, 1) = v1[i-63];
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = 0;
            c(i, 8) = c10[i-63];
            ct(i) = 1;
        }

        for (int i = 70; i < 77; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = v2[i-70];
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = 0;
            c(i, 8) = c11[i-70];
            ct(i) = -1;
        }

        for (int i = 77; i < 84; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = v2[i-77];
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = 0;
            c(i, 8) = c12[i-77];
            ct(i) = 1;
        }

        for (int i = 84; i < 91; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = v3[i-84];
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = 0;
            c(i, 8) = c13[i-84];
            ct(i) = -1;
        }

        for (int i = 91; i < 98; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = v3[i-91];
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = 0;
            c(i, 8) = c14[i-91];
            ct(i) = 1;
        }
        for (int i = 98; i < 105; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = v4[i-98];
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = 0;
            c(i, 8) = c15[i-98];
            ct(i) = -1;
        }
        for (int i = 105; i < 112; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = v4[i-105];
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = 0; 
            c(i, 8) = c16[i-105];
            ct(i) = 1;
        }
        for (int i = 112; i < 119; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = v5[i-112];
            c(i, 6) = 0;
            c(i, 7) = 0;
            c(i, 8) = c17[i-112];
            ct(i) = -1;
        }

        for (int i = 119; i < 126; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = v5[i-119];
            c(i, 6) = 0;
            c(i, 7) = 0;
            c(i, 8) = c18[i-119];
            ct(i) = 1;
        }

        for (int i = 126; i < 133; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = v6[i-126];
            c(i, 7) = 0;
            c(i, 8) = c19[i-126];
            ct(i) = -1;
        }

        for (int i = 133; i < 140; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = v6[i-133];
            c(i, 7) = 0;
            c(i, 8) = c20[i-133];
            ct(i) = 1;
        }

        for (int i = 140; i < 147; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = v7[i-140];
            c(i, 8) = c21[i-140];
            ct(i) = -1;
        }

        for (int i = 147; i < 154; i++) {
            c(i, 0) = 0;
            c(i, 1) = 0;
            c(i, 2) = 0;
            c(i, 3) = 0;
            c(i, 4) = 0;
            c(i, 5) = 0;
            c(i, 6) = 0;
            c(i, 7) = v7[i-147];
            c(i, 8) = c22[i-147];
            ct(i) = 1;
        }




    alglib::real_1d_array k;
    k.setlength(8);
    k[0] = 0.0000001;
    k[1] = 0.0;
    k[2] = 0.0;
    k[3] = 0.0;
    k[4] = 0.0;
    k[5] = 0.0;
    k[6] = 0.0;
    k[7] = 0.0;

    alglib::real_1d_array s;
    s.setlength(8);
    s[0] = 1;
    s[1] = 1;
    s[2] = 1;
    s[3] = 1;
    s[4] = 1;
    s[5] = 1;
    s[6] = 1;
    s[7] = 1;


    alglib::minbleicstate state;

    double epsg = 1e-12;
    double epsf = 1e-12;
    double epsx = 1e-12;
    alglib::ae_int_t maxits =  10000;


    alglib::minbleiccreate(k, state);


    alglib::minbleicsetlc(state, c, ct);

    alglib::minbleicsetscale(state, s);

    alglib::minbleicsetcond(state, epsg, epsf, epsx, maxits);




    alglib::minbleicoptguardsmoothness(state);
    alglib::minbleicoptguardgradient(state, 0.001);


    alglib::minbleicreport rep;
    alglib::minbleicoptimize(state,myfun);
    alglib::minbleicresults(state, k, rep);
    printf("%d\n", int(rep.terminationtype));
    printf("%s\n", k.tostring(8).c_str());


    Eigen::VectorXd dq_nullspace(7);
    Eigen::VectorXd dq_nullspace1(7);
    Eigen::VectorXd dq_nullspace2(7);
    Eigen::VectorXd dq_nullspace3(7);
    Eigen::VectorXd dq_nullspace4(7);
    Eigen::VectorXd dq_nullspace5(7);
    Eigen::VectorXd dq_nullspace6(7);
    Eigen::VectorXd dq_nullspace7(7);
    Eigen::VectorXd dq_nullspace_tot(7);


    if (deactivation==true)
    {
        dq_nullspace = k[0]* (I - J_pinv * J) * optimal_vector;
    }
    else {
        dq_nullspace.setZero();
    }


    if (deactivation1==true)
    {
        dq_nullspace1 = k[1]* (I - J_pinv * J) * optimal_vector1;
    }
    else {
        dq_nullspace1.setZero();
    }

    if (deactivation2==true)
    {
        dq_nullspace2 = k[2]* (I - J_pinv * J) * optimal_vector2;
    }
    else {
        dq_nullspace2.setZero();
    }

    if (deactivation3==true)
    {
        dq_nullspace3 = k[3]* (I - J_pinv * J) * optimal_vector3;
    }
    else {
        dq_nullspace3.setZero();
    }

    if (deactivation4==true)
    {
        dq_nullspace4 = k[4]* (I - J_pinv * J) * optimal_vector4;
    }
    else {
        dq_nullspace4.setZero();
    }
    if (deactivation5==true)
    {
        dq_nullspace5 = k[5]* (I - J_pinv * J) * optimal_vector5;
    }
    else {
        dq_nullspace5.setZero();
    }

    if (deactivation6==true)
    {
        dq_nullspace6 = k[6]* (I - J_pinv * J) * optimal_vector6;
    }
    else {
        dq_nullspace6.setZero();
    }

    if (deactivation7==true)
    {
        dq_nullspace7 = k[7]* (I - J_pinv * J) * optimal_vector7;
    }
    else {
        dq_nullspace7.setZero();
    }


    dq_nullspace_tot = dq_nullspace+dq_nullspace1+dq_nullspace2+dq_nullspace3+dq_nullspace4+dq_nullspace5+dq_nullspace6+dq_nullspace7;

    return dq_nullspace_tot;


    }
}

    
    


PLUGINLIB_EXPORT_CLASS(trial_controller_velocity::TrialControllerVelocity,controller_interface::ControllerBase)
