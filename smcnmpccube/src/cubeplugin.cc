#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>
#include <memory>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class CubePlugin : public gazebo::ModelPlugin {
public:
  CubePlugin() : ModelPlugin() {
    RCLCPP_INFO(rclcpp::get_logger("Cube_plugin"), "Starting Cube_plugin");
  }

  ~CubePlugin() {
    if (ros_thread_.joinable()) {
      ros_thread_.join();
    }
  }

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    this->model = _model;

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    node_ = rclcpp::Node::make_shared("Cube_plugin_node");
    this->rate = _sdf->HasElement("updateRate") ? _sdf->Get<double>("updateRate") : 400.0;
    this->publish_tf = _sdf->HasElement("publishTf") ? _sdf->Get<bool>("publishTf") : true;
    this->rotor_thrust_coeff = _sdf->HasElement("rotorThrustCoeff") ? _sdf->Get<double>("rotorThrustCoeff") : 0.00025;
    this->rotor_torque_coeff = _sdf->HasElement("rotorTorqueCoeff") ? _sdf->Get<double>("rotorTorqueCoeff") : 0.000075;

    RCLCPP_INFO(node_->get_logger(), "ROS2 Model Plugin Loaded with parameters!");
    sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/prop_vel", 
        10, 
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            this->ActivateCallback(msg);
        });
    ros_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    jointbackleftbottom = _model->GetJoint("pbl");
    jointbackrightbottom = _model->GetJoint("pbr");
    jointfrontleftbottom = _model->GetJoint("pfl");
    jointfrontrightbottom = _model->GetJoint("pfr");
    jointbacklefttop = _model->GetJoint("pblt");
    jointbackrighttop = _model->GetJoint("pbrt");
    jointfrontlefttop = _model->GetJoint("pflt");
    jointfrontrighttop = _model->GetJoint("pfrt");
    jointsideright = _model->GetJoint("pmr");
    jointsideleft = _model->GetJoint("pml");

    if (!jointbackleftbottom || !jointbackrightbottom || !jointfrontleftbottom || 
        !jointfrontrightbottom || !jointbacklefttop || !jointbackrighttop || 
        !jointfrontlefttop || !jointfrontrighttop || !jointsideright || !jointsideleft) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get one or more joints.");
      return;
    }

    propvel = std::vector<double>(10, 0.0);

    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&CubePlugin::OnUpdate, this));
  }

  void OnUpdate() {
    if (propvel.size() != 10) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "Propeller velocities not properly initialized.");
      return;
    }

    jointbackleftbottom->SetVelocity(0, propvel[3]);
    jointbackrightbottom->SetVelocity(0, propvel[2]);
    jointfrontleftbottom->SetVelocity(0, propvel[1]);
    jointfrontrightbottom->SetVelocity(0, propvel[0]);
    jointbacklefttop->SetVelocity(0, propvel[7]);
    jointbackrighttop->SetVelocity(0, propvel[6]);
    jointfrontlefttop->SetVelocity(0, propvel[5]);
    jointfrontrighttop->SetVelocity(0, propvel[4]);
    jointsideleft->SetVelocity(0, propvel[8]);
    jointsideright->SetVelocity(0, propvel[9]);

    double thrustflbottom = calculateThrust(propvel[3]);
    double thrustfrbottom = calculateThrust(propvel[2]);
    double thrustblbottom = calculateThrust(propvel[1]);
    double thrustbrbottom = calculateThrust(propvel[0]);
    double thrustfltop = calculateThrust(propvel[7]);
    double thrustfrtop = calculateThrust(propvel[6]);
    double thrustbltop = calculateThrust(propvel[5]);
    double thrustbrtop = calculateThrust(propvel[4]);
    double thrustsideleft = calculateThrust(propvel[8]);
    double thrustsideright = calculateThrust(propvel[9]);  

    double torqueflbottom = calculateTorque(propvel[3]);
    double torquefrbottom = calculateTorque(propvel[2]);
    double torqueblbottom = calculateTorque(propvel[1]);
    double torquebrbottom = calculateTorque(propvel[0]);
    double torquefltop = calculateTorque(propvel[7]);
    double torquefrtop = calculateTorque(propvel[6]);
    double torquebltop = calculateTorque(propvel[5]);
    double torquebrtop = calculateTorque(propvel[4]);

    applyForcesAndTorques(thrustflbottom, thrustfrbottom, thrustblbottom, thrustbrbottom, 
                         thrustfltop, thrustfrtop, thrustbltop, thrustbrtop, 
                         thrustsideleft, thrustsideright,
                         torqueflbottom, torquefrbottom, torqueblbottom, torquebrbottom, 
                         torquefltop, torquefrtop, torquebltop, torquebrtop);
  }

  void ActivateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != 10) {
      RCLCPP_WARN(node_->get_logger(), "Received incorrect number of propeller velocities. Expected 10, got %zu", msg->data.size());
      return;
    }
    propvel = msg->data;
  }

  double calculateThrust(double w) {
    return rotor_thrust_coeff * w * w;
  }

  double calculateTorque(double w) {
    return -sgn(w) * rotor_torque_coeff * w * w;
  }

private:
  rclcpp::Node::SharedPtr node_;  
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  std::thread ros_thread_;

  double rate;
  bool publish_tf;
  double rotor_thrust_coeff;
  double rotor_torque_coeff;

  gazebo::physics::ModelPtr model;
  gazebo::physics::JointPtr jointbackleftbottom;
  gazebo::physics::JointPtr jointbackrightbottom;
  gazebo::physics::JointPtr jointfrontleftbottom;
  gazebo::physics::JointPtr jointfrontrightbottom;
  gazebo::physics::JointPtr jointbacklefttop;
  gazebo::physics::JointPtr jointbackrighttop;
  gazebo::physics::JointPtr jointfrontlefttop;
  gazebo::physics::JointPtr jointfrontrighttop;
  gazebo::physics::JointPtr jointsideright;
  gazebo::physics::JointPtr jointsideleft;

  gazebo::event::ConnectionPtr updateConnection;
  std::vector<double> propvel;

  void applyForcesAndTorques(double thrustflbottom, double thrustfrbottom, double thrustblbottom, double thrustbrbottom,
                           double thrustfltop, double thrustfrtop, double thrustbltop, double thrustbrtop,
                           double thrustsideleft, double thrustsideright,
                           double torqueflbottom, double torquefrbottom, double torqueblbottom, double torquebrbottom,
                           double torquefltop, double torquefrtop, double torquebltop, double torquebrtop) {
                            
    gazebo::physics::LinkPtr linkflbottom = model->GetLink("propbackleft");
    gazebo::physics::LinkPtr linkfrbottom = model->GetLink("propbackright");
    gazebo::physics::LinkPtr linkblbottom = model->GetLink("propfrontleft");
    gazebo::physics::LinkPtr linkbrbottom = model->GetLink("propfrontright");
    gazebo::physics::LinkPtr linkfltop = model->GetLink("propbacklefttop");
    gazebo::physics::LinkPtr linkfrtop = model->GetLink("propbackrighttop");
    gazebo::physics::LinkPtr linkbltop = model->GetLink("propfrontlefttop");
    gazebo::physics::LinkPtr linkbrtop = model->GetLink("propfrontrighttop");
    gazebo::physics::LinkPtr linkmidleft = model->GetLink("propmidleft");
    gazebo::physics::LinkPtr linkmidright = model->GetLink("propmidright");
    gazebo::physics::LinkPtr base = model->GetLink("base_link");

    if (linkflbottom) linkflbottom->AddLinkForce(ignition::math::Vector3d(0, 0, thrustflbottom));
    if (linkfrbottom) linkfrbottom->AddLinkForce(ignition::math::Vector3d(0, 0, thrustfrbottom));
    if (linkblbottom) linkblbottom->AddLinkForce(ignition::math::Vector3d(0, 0, thrustblbottom));
    //motor failure
    //if (linkblbottom) linkblbottom->AddLinkForce(ignition::math::Vector3d(0, 0, 0.5 * thrustblbottom));

    if (linkbrbottom) linkbrbottom->AddLinkForce(ignition::math::Vector3d(0, 0, thrustbrbottom));
    if (linkfltop) linkfltop->AddLinkForce(ignition::math::Vector3d(0, 0, thrustfltop));
    if (linkfrtop) linkfrtop->AddLinkForce(ignition::math::Vector3d(0, 0, thrustfrtop));
    if (linkbltop) linkbltop->AddLinkForce(ignition::math::Vector3d(0, 0, thrustbltop));
    if (linkbrtop) linkbrtop->AddLinkForce(ignition::math::Vector3d(0, 0, thrustbrtop));
    if (linkmidright) linkmidright->AddLinkForce(ignition::math::Vector3d(0, 0, -thrustsideright));
    if (linkmidleft) linkmidleft->AddLinkForce(ignition::math::Vector3d(0, 0, -thrustsideleft));

    if (base) {
      double total_torque = torqueflbottom + torquefrbottom + torqueblbottom + torquebrbottom + 
                           torquefltop + torquefrtop + torquebltop + torquebrtop;
      base->AddRelativeTorque(ignition::math::Vector3d(0, 0, total_torque));
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(CubePlugin)