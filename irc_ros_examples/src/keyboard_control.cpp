#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

#include "irc_ros_msgs/msg/dio_command.hpp"
#include "irc_ros_msgs/srv/dio_command.hpp"
#include "irc_ros_msgs/msg/dio_int_command.hpp"
#include <termios.h>
#include <unistd.h>
#include <mutex>


class KeyboardCtrl : public rclcpp::Node
{
public:
  KeyboardCtrl() : Node("KeyboardCtrl")
  {

    RCLCPP_INFO(
                rclcpp::get_logger(""), "KeyboardCtrl Node started..."
            );
    enabled = false;
    reset = false;

    changeMotionTypePub_ = create_publisher<irc_ros_msgs::msg::DioIntCommand>("cpr_platform_controller/set_motionType", rclcpp::SystemDefaultsQoS());
    dioPub_ = create_publisher<irc_ros_msgs::msg::DioCommand>("cpr_platform_controller/set_outputs", rclcpp::SystemDefaultsQoS()); 
    twistPub_ = create_publisher<geometry_msgs::msg::Twist>("cpr_platform_controller/cmd_vel", rclcpp::SystemDefaultsQoS());
    jointVelocityPub_ = create_publisher<std_msgs::msg::Float64MultiArray>("joint_velocity_controller/commands", rclcpp::SystemDefaultsQoS());    

    key_press_thread_ = std::thread(&KeyboardCtrl::keyPressThread, this);
    twist_thread_ = std::thread(&KeyboardCtrl::publishTwistThread, this);

    key_press_thread_.detach();
    twist_thread_.detach();
  }

private:

    bool enabled = false; 
    bool reset;

    float lin_x = 0.0;
    float lin_y = 0.0;
    float lin_z = 0.0;
    
    float rot_x = 0.0;
    float rot_y = 0.0;
    float rot_z = 0.0;
    
    std::mutex m_mutex;

    int motionType = 0;
    
    std::array<float, 6> joint_velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};


    void PublishJointVelocities()
    {
        std_msgs::msg::Float64MultiArray jointVels;

        jointVels.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        jointVels.layout.dim[0].size = 6;

        jointVels.layout.dim[0].stride = 1;
        jointVels.layout.dim[0].label = "JointVelocities";

        jointVels.data.clear();
        jointVels.data.insert(jointVels.data.end(), joint_velocities.begin(), joint_velocities.end());

        jointVelocityPub_->publish(jointVels);
    }

    void PublishTwist()
    {
        geometry_msgs::msg::Twist twist_cmd;

        m_mutex.lock();
        twist_cmd.linear.x = lin_x;
        twist_cmd.linear.y = lin_y;
        twist_cmd.linear.z = lin_z;
        
        twist_cmd.angular.x = rot_x;
        twist_cmd.angular.y = rot_y;
        twist_cmd.angular.z = rot_z;
        m_mutex.unlock();
        twistPub_->publish(twist_cmd);
    }
    void Enable()
    {
        irc_ros_msgs::msg::DioCommand command;
        command.header = std_msgs::msg::Header();
        command.header.stamp = get_clock()->now();
        command.names.push_back("platform/enable");
        command.outputs.push_back(true);

        enabled = true;
        dioPub_->publish(command);
    }

    void Reset()
    {   
        irc_ros_msgs::msg::DioCommand command;
        command.header = std_msgs::msg::Header();
        command.header.stamp = get_clock()->now();
        command.names.push_back("platform/reset");
        
        if (reset)
        {
            command.outputs.push_back(false);
        }else
        {
            command.outputs.push_back(true);
        }

        reset = !reset;
        dioPub_->publish(command);
    }

    void Disable()
    {
        irc_ros_msgs::msg::DioCommand command;
        command.header = std_msgs::msg::Header();
        command.header.stamp = get_clock()->now();
        command.names.push_back("platform/enable");
        command.outputs.push_back(false);
        enabled = false;
        dioPub_->publish(command);
    }

    void ChangeMotionType()
    {
        irc_ros_msgs::msg::DioIntCommand command;
        command.header = std_msgs::msg::Header();
        command.header.stamp = get_clock()->now();
        command.names.push_back("platform/setMotionType");
        command.outputs.push_back(motionType);
        changeMotionTypePub_->publish(command);
    }

    void publishTwistThread()
    {
        while(rclcpp::ok())
        {
            if (motionType == 0)
            {
                PublishTwist();
            }
            else
            {
                PublishJointVelocities();
            }

        }
    }

    void keyPressThread()
    {
        struct termios oldt, newt;
        char ch;

        // Get the current terminal settings
        tcgetattr(STDIN_FILENO, &oldt);

        // Set the terminal to non-canonical mode to capture individual key presses
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (rclcpp::ok())
        {
            
        // Wait for a key press
            ch = getchar();

            RCLCPP_INFO(
                rclcpp::get_logger(""), "KeyPress: %c", ch
            );
//**************************Functional commands***************************************
            if (ch == 'r')
            {   
                m_mutex.lock();
                lin_x = 0.0;
                lin_y = 0.0;
                lin_z = 0.0;
                
                rot_x = 0.0;
                rot_y = 0.0;
                rot_z = 0.0;

                joint_velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

                m_mutex.unlock();
                
                Reset();                
            }
            if (ch == 'e')
            {   
                m_mutex.lock();
                lin_x = 0.0;
                lin_y = 0.0;
                lin_z = 0.0;
                
                rot_x = 0.0;
                rot_y = 0.0;
                rot_z = 0.0;

                joint_velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

                m_mutex.unlock();

                Enable();    
            }
            if (ch == 'w')
            {   
                m_mutex.lock();
                lin_x = 0.0;
                lin_y = 0.0;
                lin_z = 0.0;
                
                rot_x = 0.0;
                rot_y = 0.0;
                rot_z = 0.0;

                joint_velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

                m_mutex.unlock();

                Disable();
            }
            if (ch == 'q')
            {   
                m_mutex.lock();
                lin_x = 0.0;
                lin_y = 0.0;
                lin_z = 0.0;
                
                rot_x = 0.0;
                rot_y = 0.0;
                rot_z = 0.0;

                joint_velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                m_mutex.unlock();
            } 
        
            //ChangeMotion Type
            if (ch == 'm')
            {   
                m_mutex.lock();
                lin_x = 0.0;
                lin_y = 0.0;
                lin_z = 0.0;
                
                rot_x = 0.0;
                rot_y = 0.0;
                rot_z = 0.0;

                joint_velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

                m_mutex.unlock();

                PublishTwist();

                motionType++;
                if (motionType > 3)
                {
                    motionType = 0;
                }
                RCLCPP_INFO(
                    rclcpp::get_logger(""), "MotionType: %d", motionType
                );
                ChangeMotionType();
            }

//************************Positive Movement Commands*************************************
            m_mutex.lock();
            if (ch == 'a')
            {   
                if (motionType == 0)
                {   
                    lin_x += 0.5;
                }
                else
                {   
                    
                    joint_velocities[0] += 0.1;
                }
            }   
            if (ch == 's')
            {
                if (motionType == 0)
                {
                    lin_y += 0.5;
                }
                else
                {
                    joint_velocities[1] += 0.1;
                }
            }
            if (ch == 'd')
            {   
                if (motionType == 0)
                {
                    lin_z += 0.5;
                }
                else
                {
                    joint_velocities[2] += 0.1;
                }
            }
            if (ch == 'f')
            {
                if (motionType == 0)
                {
                    rot_x += 0.5;
                }
                else
                {
                    joint_velocities[3] += 0.1;
                }
            }
            if (ch == 'g')
            {
                if (motionType == 0)
                {
                    rot_y += 0.5;
                }
                else
                {
                    joint_velocities[4] += 0.1;
                }
            }

            if (ch == 'h')
            {
                if (motionType == 0)
                {
                    rot_z += 0.5;
                }
                else
                {
                    joint_velocities[5] += 0.1;
                }
            }
//*************************************NEGATIVE MOVEMENT COMMANDS***********************************
            if (ch == 'y')
            {   
                if (motionType == 0)
                {
                    lin_x -= 0.5;
                }
                else
                {
                    joint_velocities[0] -= 0.1;
                }
            }   
            if (ch == 'x')
            {
                if (motionType == 0)
                {
                    lin_y -= 0.5;
                }
                else
                {
                    joint_velocities[1] -= 0.1;
                }
            }
            //lateral
            if (ch == 'c')
            {   
                if (motionType == 0)
                {
                    lin_z -= 0.5;
                }
                else
                {
                    joint_velocities[2] -= 0.1;
                }
            }
            if (ch == 'v')
            {
                if (motionType == 0)
                {
                    rot_x -= 0.5;
                }
                else
                {
                    joint_velocities[3] -= 0.1;
                }
            }
            //rotation
            if (ch == 'b')
            {
                if (motionType == 0)
                {
                    rot_y -= 0.5;
                }
                else
                {
                    joint_velocities[4] -= 0.1;
                }
            }

            if (ch == 'n')
            {
                if (motionType == 0)
                {
                    rot_z -= 0.5;
                }
                else
                {
                    joint_velocities[5] -= 0.1;
                }
            }
            m_mutex.unlock();
        }
    }

    rclcpp::Publisher<irc_ros_msgs::msg::DioCommand>::SharedPtr dioPub_;
    rclcpp::Publisher<irc_ros_msgs::msg::DioIntCommand>::SharedPtr changeMotionTypePub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jointVelocityPub_;
    std::thread key_press_thread_;
    std::thread twist_thread_; 
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardCtrl>());
  rclcpp::shutdown();
  return 0;
}