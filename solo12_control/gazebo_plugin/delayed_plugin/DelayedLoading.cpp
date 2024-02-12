#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <chrono>
#include <thread>

namespace gazebo
{
  class DelayedModelLoader : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    double delaySeconds = 3.0; // Default delay time

  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model = _model;

      // Check if the delay is specified in the SDF file
      if (_sdf->HasElement("delay"))
      {
        delaySeconds = _sdf->Get<double>("delay");
      }

      // Run the delay operation in a separate thread to not block Gazebo loading
      std::thread(&DelayedModelLoader::DelayedOperation, this).detach();
    }

    void DelayedOperation()
    {
      // Initially make the model static to prevent it from falling due to gravity
      this->model->SetStatic(true);

      // Wait for the specified delay
      std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(delaySeconds)));

      // Make the model dynamic again after the delay
      this->model->SetStatic(false);
      
      // If the model needs to be repositioned or other initializations are required, do them here
      // For example, to reposition:
      // this->model->SetWorldPose(ignition::math::Pose3d(new_x, new_y, new_z, 0, 0, 0));
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(DelayedModelLoader)
}
