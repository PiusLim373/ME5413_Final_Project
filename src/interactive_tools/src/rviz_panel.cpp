#include <pluginlib/class_list_macros.hpp>
#include "interactive_tools/rviz_panel.hpp"

PLUGINLIB_EXPORT_CLASS(rviz_panel::simplePanel, rviz::Panel)

namespace rviz_panel
{
    simplePanel::simplePanel(QWidget * parent)
    :   rviz::Panel(parent),
        ui_(std::make_shared<Ui::two_button>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        // Define ROS publisher
        pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

        // Connect the clicked signals to slots
        connect(ui_->pushButton_1_1, SIGNAL(clicked()), this, SLOT(on_button_1_1_clicked()));
        connect(ui_->pushButton_1_2, SIGNAL(clicked()), this, SLOT(on_button_1_2_clicked()));

        connect(ui_->pushButton_2_1, SIGNAL(clicked()), this, SLOT(on_button_2_1_clicked()));
        connect(ui_->pushButton_2_2, SIGNAL(clicked()), this, SLOT(on_button_2_2_clicked()));
        connect(ui_->pushButton_2_3, SIGNAL(clicked()), this, SLOT(on_button_2_3_clicked()));
        connect(ui_->pushButton_2_4, SIGNAL(clicked()), this, SLOT(on_button_2_4_clicked()));

        connect(ui_->pushButton_3_1, SIGNAL(clicked()), this, SLOT(on_button_3_1_clicked()));
        connect(ui_->pushButton_3_2, SIGNAL(clicked()), this, SLOT(on_button_3_2_clicked()));
        connect(ui_->pushButton_3_3, SIGNAL(clicked()), this, SLOT(on_button_3_3_clicked()));
    }

    geometry_msgs::PoseStamped simplePanel::getPoseMsgFromConfig(const std::string& name)
    {
        double x, y, yaw;
        std::string frame_id;
        nh_.getParam("/me5413_project" + name + "/x", x);
        nh_.getParam("/me5413_project" + name + "/y", y);
        nh_.getParam("/me5413_project" + name + "/yaw", yaw);
        nh_.getParam("/me5413_project/frame_id", frame_id);

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        q.normalize();

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = frame_id;
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.orientation = tf2::toMsg(q);

        return pose_msg;
    }

    // Assembly Line buttons
    void simplePanel::on_button_1_1_clicked()
    {
        ROS_INFO_STREAM("Setting Assembly Line 1 as the GOAL.");
        const auto goal_pose = getPoseMsgFromConfig("/assembly_line_1");
        this->pub_goal_.publish(goal_pose);
    }
    void simplePanel::on_button_1_2_clicked()
    {
        ROS_INFO_STREAM("Setting Assembly Line 2 as the GOAL.");
        const auto goal_pose = getPoseMsgFromConfig("/assembly_line_2");
        this->pub_goal_.publish(goal_pose);
    }

    // Packaging Area buttons
    void simplePanel::on_button_2_1_clicked()
    {
        ROS_INFO_STREAM("Setting Packaging Area 1 as the GOAL.");
        const auto goal_pose = getPoseMsgFromConfig("/packing_area_1");
        this->pub_goal_.publish(goal_pose);
    }
    void simplePanel::on_button_2_2_clicked()
    {
        ROS_INFO_STREAM("Setting Packaging Area 2 as the GOAL.");
        const auto goal_pose = getPoseMsgFromConfig("/packing_area_2");
        this->pub_goal_.publish(goal_pose);
    }
    void simplePanel::on_button_2_3_clicked()
    {
        ROS_INFO_STREAM("Setting Packaging Area 3 as the GOAL.");
        const auto goal_pose = getPoseMsgFromConfig("/packing_area_3");
        this->pub_goal_.publish(goal_pose);
    }
    void simplePanel::on_button_2_4_clicked()
    {
        ROS_INFO_STREAM("Setting Packaging Area 4 as the GOAL.");
        const auto goal_pose = getPoseMsgFromConfig("/packing_area_4");
        this->pub_goal_.publish(goal_pose);
    }

    // Delivery Vehicle buttons
    void simplePanel::on_button_3_1_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 1 as the GOAL.");
        const auto goal_pose = getPoseMsgFromConfig("/vehicle_1");
        this->pub_goal_.publish(goal_pose);
    }
    void simplePanel::on_button_3_2_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 2 as the GOAL.");
        const auto goal_pose = getPoseMsgFromConfig("/vehicle_2");
        this->pub_goal_.publish(goal_pose);
    }
    void simplePanel::on_button_3_3_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 3 as the GOAL.");
        const auto goal_pose = getPoseMsgFromConfig("/vehicle_3");
        this->pub_goal_.publish(goal_pose);
    }

    /**
     *  Save all configuration data from this panel to the given
     *  Config object. It is important here that you call save()
     *  on the parent class so the class id and panel name get saved.
     */
    void simplePanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    /**
     *  Load all configuration data for this panel from the given Config object.
     */
    void simplePanel::load(const rviz::Config & config)
    {
        rviz::Panel::load(config);
    }
} // namespace rviz_panel
