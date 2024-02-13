#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/msgs/joint_cmd.pb.h>
#include <gazebo/msgs/msgs.hh>



namespace gazebo {
class RobotJointController : public ModelPlugin {
private:
    physics::ModelPtr model;
    transport::NodePtr node;
    transport::SubscriberPtr subscriber;
    transport::PublisherPtr jointPub; // Publisher for joint commands
    std::vector<std::string> jointNames;
    event::ConnectionPtr updateConnection;

public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override {
        this->model = _model;
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(model->GetName());

        std::cout << "Initializing joint names and subscribing to joint control commands." << std::endl;
        InitializeJointNames();

        this->subscriber = this->node->Subscribe("/gazebo/solo12/robot/joint_control_cmd",
                                                 &RobotJointController::OnJointControlCmdMessage, this);

        // Initialize the publisher for joint states
        this->jointPub = this->node->Advertise<gazebo::msgs::JointCmd>("/gazebo/solo12/joint_states");

        std::cout << "Subscription to joint control commands complete." << std::endl;

        this->node->Subscribe("/gazebo/model_states", &RobotJointController::OnModelStatesMsg, this); //For getting joint_states feedback

        // PID controller parameters
        double P = 3.0, I = 0, D = 1.2;
        for (const auto& jointName : this->jointNames) {
            this->SetupJointControl(jointName, 0.1, P, I, D); // Adjust targetPos as needed per joint
        }
  

        // Setup to periodically publish joint positions
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&RobotJointController::OnUpdate, this, std::placeholders::_1));

  
    
    }

private:
    void InitializeJointNames() {
        this->jointNames = {"FL_HAA", "FR_HAA", "HL_HAA", "HR_HAA", 
                            "FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE", 
                            "FL_KFE", "FR_KFE", "HL_KFE", "HR_KFE"};
        std::cout << "Joint names initialized." << std::endl;
    }

    // This function is called at every simulation iteration
    void OnUpdate(const common::UpdateInfo & /*info*/) {
        gazebo::msgs::JointCmd jointStateMsg;

        for (const auto& jointName : this->jointNames) {
            auto joint = this->model->GetJoint(jointName);
            if (joint) {
                double position = joint->Position(0); // Get the position of the first axis
                // Populate the message with joint state information
                jointStateMsg.set_name(jointName);
                jointStateMsg.mutable_position()->set_target(position);
                // Publish the joint state message
                this->jointPub->Publish(jointStateMsg);
            }
        }
    }


    // Callback function for processing messages
    void OnJointControlCmdMessage(ConstGzStringPtr &msg) {
        std::string message = msg->data();
        std::cout << "Received joint control command: " << message << std::endl;

        // Split the message into parts
        std::istringstream messageStream(message);
        std::string segment;
        std::vector<std::string> seglist;

        while (std::getline(messageStream, segment, '#')) {
            seglist.push_back(segment);
        }

        if (seglist.size() != 5) { // Expecting 5 segments: JointName, targetPos, P, I, D
            std::cerr << "Error: incorrect number of parameters in command." << std::endl;
            return;
        }

        // Extract values
        std::string jointName = seglist[0];
        double targetPos, P, I, D;
        try {
            targetPos = std::stod(seglist[1]);
            P = std::stod(seglist[2]);
            I = std::stod(seglist[3]);
            D = std::stod(seglist[4]);
        } catch (const std::exception& e) {
            std::cerr << "Error parsing numerical values for " << jointName << ": " << e.what() << std::endl;
            return;
        }

        // Now apply these to the joint
        SetupJointControl(jointName, targetPos, P, I, D);
        std::cout << "Control command applied: " << jointName << " Position: " << targetPos << " PID: " << P << ", " << I << ", " << D << std::endl;
    }


    void SetupJointControl(const std::string &jointName, double targetPos, double P, double I, double D) {
        auto joint = this->model->GetJoint(jointName);
        if (!joint) {
            std::cerr << "Joint " << jointName << " not found in model." << std::endl;
            return;
        }

        common::PID pid(P, I, D);
        this->model->GetJointController()->SetPositionPID(joint->GetScopedName(), pid);
        this->model->GetJointController()->SetPositionTarget(joint->GetScopedName(), targetPos);
        std::cout << "Control applied to " << jointName << std::endl;
    }
    void RobotJointController::OnModelStatesMsg(ConstModelStatesPtr& msg) {
        // Example: iterate through the received model states
        for (int i = 0; i < msg->model_size(); ++i) {
            const auto& model = msg->model(i);
            std::cout << "Model name: " << model.name() << std::endl;
            
            // Further processing as needed
        }
};

GZ_REGISTER_MODEL_PLUGIN(RobotJointController)
}
