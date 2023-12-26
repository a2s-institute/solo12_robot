#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class RobotJointController : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      // Initialize the model
      this->model = _model;

      // Define PID controller parameters
      double P = 7.0;
      double I = 0; // Updated Integral Gain
      double D = 1.2;

      // Set target positions and PID values for each joint
      SetupJointControl("FL_HAA", 0.05, P, I, D);
      SetupJointControl("FR_HAA", 0.05, P, I, D);
      SetupJointControl("HL_HAA", 0.05, P, I, D);
      SetupJointControl("HR_HAA", 0.05, P, I, D);

      SetupJointControl("FL_HFE", -0.4, P, I, D);
      SetupJointControl("FR_HFE", -0.4, P, I, D);
      SetupJointControl("HL_HFE", -0.9, P, I, D);
      SetupJointControl("HR_HFE", -0.9, P, I, D);

      SetupJointControl("FL_KFE", 1.0, P, I, D);
      SetupJointControl("FR_KFE", 1.0, P, I, D);
      SetupJointControl("HL_KFE", 1.2, P, I, D);
      SetupJointControl("HR_KFE", 1.2, P, I, D);

      // Continue with the rest of the joints as necessary
    }

  private:
    physics::ModelPtr model;

    void SetupJointControl(const std::string &jointName, double targetPos, double P, double I, double D)
    {
      auto joint = this->model->GetJoint(jointName);
      if (!joint)
      {
        gzerr << "Joint " << jointName << " not found.\n";
        return;
      }

      auto pid = common::PID(P, I, D);
      this->model->GetJointController()->SetPositionPID(joint->GetScopedName(), pid);
      this->model->GetJointController()->SetPositionTarget(joint->GetScopedName(), targetPos);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RobotJointController)
}
