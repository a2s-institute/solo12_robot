#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

namespace gazebo
{
  class RobotJointController : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    transport::NodePtr node;
    transport::SubscriberPtr subscriber;
    transport::SubscriberPtr pidSubscriber; // Variable for the PID subscriber
    std::vector<std::string> jointNames; // Vector to hold joint names

  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      this->model = _model;
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      this->subscriber = this->node->Subscribe("/gazebo/solo12/robot/control", &RobotJointController::OnControlMessage, this);

      // Initialize joint names
      this->jointNames = {"FL_HAA", "FR_HAA", "HL_HAA", "HR_HAA", "FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE", "FL_KFE", "FR_KFE", "HL_KFE", "HR_KFE"};

      // PID controller parameters
      double P = 3.0, I = 0, D = 1.2;
      for (const auto& jointName : this->jointNames)
      {
        this->SetupJointControl(jointName, 0.1, P, I, D); // Adjust targetPos as needed per joint
      }

      this->pidSubscriber = this->node->Subscribe("/gazebo/solo12/robot/pid", &RobotJointController::OnPIDMessage, this);
    }

    private: void OnPIDMessage(ConstVector3dPtr &_msg)
    {
      double P = _msg->x();
      double I = _msg->y();
      double D = _msg->z();

      UpdateJointPIDs(P, I, D);
    }

    private: void UpdateJointPIDs(double P, double I, double D)
    {
      common::PID pid(P, I, D);
      for (const auto& jointName : this->jointNames)
      {
        this->model->GetJointController()->SetPositionPID(jointName, pid);
      }
    }

    private: void OnControlMessage(ConstVector3dPtr &_msg)
    {
      // Placeholder for handling control messages
      // Adapt to your specific message type and control logic
    }

    private: void SetupJointControl(const std::string &jointName, double targetPos, double P, double I, double D)
    {
      auto joint = this->model->GetJoint(jointName);
      if (!joint)
      {
        gzerr << "Joint " << jointName << " not found.\n";
        return;
      }

      common::PID pid(P, I, D);
      this->model->GetJointController()->SetPositionPID(joint->GetScopedName(), pid);
      this->model->GetJointController()->SetPositionTarget(joint->GetScopedName(), targetPos);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(RobotJointController)
}