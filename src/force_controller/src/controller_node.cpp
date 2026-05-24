#include <force_controller/current_estimator.h>
#include <force_controller/impedance_controller.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <geometry_msgs/Vector3.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>

/**
 * @brief Force controller node (Module C).
 *
 * Subscribes to:
 *   /<robot_name>/joint_states       (sensor_msgs/JointState, effort from actuators[].torque)
 *   /<robot_name>/base_feedback      (kortex_driver/BaseCyclic_Feedback, 40 Hz)
 *   /force_control/active            (std_msgs/Bool)
 *
 * Publishes:
 *   /force_correction  (geometry_msgs/Vector3)
 *   /contact_state     (ContactState) — not yet, uses Vector3 for now
 *
 * Services:
 *   /force_controller/calibrate  (std_srvs/Trigger)
 */
class ForceControllerNode {
public:
    ForceControllerNode()
        : nh_()
        , pnh_("~")
        , control_rate_(100.0)
        , contact_threshold_(0.3)
        , in_contact_(false)
        , is_active_(false)
        , current_surface_normal_(Eigen::Vector3d::UnitZ())
    {
        std::string robot_name;
        pnh_.param("robot_name", robot_name, std::string("my_gen3_lite"));

        pnh_.param("desired_force", F_desired_, 1.0);
        pnh_.param("Kp", Kp_, 0.0003);
        pnh_.param("Kd", Kd_, 0.00005);
        pnh_.param("dz_max", dz_max_, 0.0005);
        pnh_.param("control_rate", control_rate_, 100.0);
        pnh_.param("contact_threshold", contact_threshold_, 0.3);

        impedance_ctrl_.setParams(Kp_, Kd_, F_desired_, dz_max_);

        // Subscribers
        sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>(
            "/" + robot_name + "/joint_states", 10,
            &ForceControllerNode::jointStateCallback, this);

        sub_base_feedback_ = nh_.subscribe<kortex_driver::BaseCyclic_Feedback>(
            "/" + robot_name + "/base_feedback", 10,
            &ForceControllerNode::baseFeedbackCallback, this);

        sub_active_ = nh_.subscribe<std_msgs::Bool>(
            "/force_control/active", 10,
            &ForceControllerNode::activeCallback, this);

        // Publisher
        pub_correction_ = nh_.advertise<geometry_msgs::Vector3>(
            "/force_correction", 10);

        // Services
        srv_calibrate_ = nh_.advertiseService(
            "/force_controller/calibrate",
            &ForceControllerNode::calibrateCallback, this);

        // Control loop timer
        control_timer_ = nh_.createTimer(
            ros::Duration(1.0 / control_rate_),
            &ForceControllerNode::controlLoop, this);

        ROS_INFO("Force controller node started.");
        ROS_INFO("  Robot: %s, Desired force: %.1f N, Rate: %.0f Hz",
                 robot_name.c_str(), F_desired_, control_rate_);
    }

private:
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
        latest_joint_state_ = *msg;
    }

    void baseFeedbackCallback(const kortex_driver::BaseCyclic_FeedbackConstPtr& msg) {
        // Store latest feedback
        latest_feedback_ = *msg;
    }

    void activeCallback(const std_msgs::BoolConstPtr& msg) {
        is_active_ = msg->data;
        if (!is_active_) {
            impedance_ctrl_.reset();
            in_contact_ = false;
        }
        ROS_INFO("Force control %s", is_active_ ? "ACTIVATED" : "DEACTIVATED");
    }

    bool calibrateCallback(std_srvs::Trigger::Request&,
                           std_srvs::Trigger::Response& res) {

        // Use current torques as bias
        if (latest_joint_state_.effort.size() >= 6) {
            std::vector<double> tau_current(
                latest_joint_state_.effort.begin(),
                latest_joint_state_.effort.begin() + 6);
            estimator_.calibrate(tau_current);
            res.success = true;
            res.message = "Force sensor calibrated.";
            ROS_INFO("Force calibration done.");
        } else {
            res.success = false;
            res.message = "No joint effort data available.";
        }
        return true;
    }

    void controlLoop(const ros::TimerEvent&) {
        if (!is_active_) {
            // Publish zero correction when inactive
            geometry_msgs::Vector3 zero_correction;
            zero_correction.x = 0.0;
            zero_correction.y = 0.0;
            zero_correction.z = 0.0;
            pub_correction_.publish(zero_correction);
            return;
        }

        // Get current joint state
        std::vector<double> q(latest_joint_state_.position.begin(),
                              latest_joint_state_.position.end());
        std::vector<double> tau(latest_joint_state_.effort.begin(),
                                latest_joint_state_.effort.end());

        if (q.size() < 6 || tau.size() < 6) {
            return;
        }

        // Estimate normal force
        double F_est = estimator_.estimateNormalForce(
            q, tau, current_surface_normal_);

        // Check contact threshold
        in_contact_ = (std::abs(F_est) > contact_threshold_);

        // Compute pose correction via virtual impedance
        double dt = 1.0 / control_rate_;
        Eigen::Vector3d correction = impedance_ctrl_.computePoseCorrection(
            F_est, current_surface_normal_, dt);

        // Publish correction
        geometry_msgs::Vector3 correction_msg;
        correction_msg.x = correction.x();
        correction_msg.y = correction.y();
        correction_msg.z = correction.z();
        pub_correction_.publish(correction_msg);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_base_feedback_;
    ros::Subscriber sub_active_;
    ros::Publisher  pub_correction_;
    ros::ServiceServer srv_calibrate_;
    ros::Timer control_timer_;

    force_controller::CurrentEstimator estimator_;
    force_controller::ImpedanceController impedance_ctrl_;

    sensor_msgs::JointState latest_joint_state_;
    kortex_driver::BaseCyclic_Feedback latest_feedback_;

    double control_rate_;
    double contact_threshold_;
    double F_desired_, Kp_, Kd_, dz_max_;
    bool in_contact_;
    bool is_active_;

    Eigen::Vector3d current_surface_normal_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");
    ForceControllerNode node;
    ros::spin();
    return 0;
}
