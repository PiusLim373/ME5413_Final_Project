/* rviz_panel.cpp

 * Copyright (C) 2023 SS47816

 * Rviz Panel for controling goal poses 
 
**/

#include <pluginlib/class_list_macros.hpp>
#include "interactive_tools/rviz_panel.hpp"

PLUGINLIB_EXPORT_CLASS(rviz_panel::simplePanel, rviz::Panel)

namespace rviz_panel
{
    simplePanel::simplePanel(QWidget * parent)
    :   rviz::Panel(parent),
        ui_(std::make_shared<Ui::TaskControlPanel>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        // Define ROS publisher
        this->pub_goal_ = nh_.advertise<std_msgs::String>("/rviz_panel/goal_name", 1);
        this->pub_respawn_ = nh_.advertise<std_msgs::Int16>("/rviz_panel/respawn_objects", 1);
        this->movebase_cancel_goal_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
        this->skip_forward_pub_ = nh_.advertise<std_msgs::String>("/skip_ahead", 1);
        this->clear_costmap_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        this->move_base_status_sub_ = nh_.subscribe("/move_base/status", 1, &simplePanel::moveBaseStatusCB, this);
        // sub_error_to_goal_ = nh_.subscribe("/interactive_tools/error_to_goal", 1, &GoalPublisherNode::goalPoseCallback, this);

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

        connect(ui_->pushButton_regen, SIGNAL(clicked()), this, SLOT(on_button_regen_clicked()));
        connect(ui_->pushButton_clear, SIGNAL(clicked()), this, SLOT(on_button_clear_clicked()));
        connect(ui_->pushButton_cancel_goal, SIGNAL(clicked()), this, SLOT(on_button_cancel_goal_clicked()));
        connect(ui_->pushButton_skip_forward, SIGNAL(clicked()), this, SLOT(on_button_skip_forward_clicked()));
        connect(ui_->pushButton_clear_costmap, SIGNAL(clicked()), this, SLOT(on_button_clear_costmap_clicked()));

        // Initialization
        goal_name_msg_.data = "";
    }

    // Assembly Line buttons
    void simplePanel::on_button_1_1_clicked()
    {
        ROS_INFO_STREAM("Setting Assembly Line 1 as the GOAL.");
        ui_->label_status->setText("Heading to Assembly Line 1");
        this->goal_name_msg_.data = "/assembly_line_1";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_1_2_clicked()
    {
        ROS_INFO_STREAM("Setting Assembly Line 2 as the GOAL.");
        ui_->label_status->setText("Heading to Assembly Line 2");
        this->goal_name_msg_.data = "/assembly_line_2";
        this->pub_goal_.publish(this->goal_name_msg_);
    }

    // Box buttons
    void simplePanel::on_button_2_1_clicked()
    {
        ROS_INFO_STREAM("Setting Box 1 as the GOAL.");
        ui_->label_status->setText("Heading to Box 1");
        this->goal_name_msg_.data = "/box_1";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_2_2_clicked()
    {
        ROS_INFO_STREAM("Setting Box 2 as the GOAL.");
        ui_->label_status->setText("Heading to Box 2");
        this->goal_name_msg_.data = "/box_2";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_2_3_clicked()
    {
        ROS_INFO_STREAM("Setting Box 3 as the GOAL.");
        ui_->label_status->setText("Heading to Box 3");
        this->goal_name_msg_.data = "/box_3";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_2_4_clicked()
    {
        ROS_INFO_STREAM("Setting Box 4 as the GOAL.");
        ui_->label_status->setText("Heading to Box 4");
        this->goal_name_msg_.data = "/box_4";
        this->pub_goal_.publish(this->goal_name_msg_);
    }

    // Delivery Vehicle buttons
    void simplePanel::on_button_3_1_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 1 as the GOAL.");
        ui_->label_status->setText("Heading to Vehicle 1");
        this->goal_name_msg_.data = "/vehicle_1";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_3_2_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 2 as the GOAL.");
        ui_->label_status->setText("Heading to Vehicle 2");
        this->goal_name_msg_.data = "/vehicle_2";
        this->pub_goal_.publish(this->goal_name_msg_);
    }
    void simplePanel::on_button_3_3_clicked()
    {
        ROS_INFO_STREAM("Setting Vehicle 3 as the GOAL.");
        ui_->label_status->setText("Heading to Vehicle 3");
        this->goal_name_msg_.data = "/vehicle_3";
        this->pub_goal_.publish(this->goal_name_msg_);
    }

    void simplePanel::on_button_regen_clicked()
    {
        ROS_INFO_STREAM("Respawning Random Objects");
        ui_->label_status->setText("Please select a goal pose");
        this->regen_cmd_msg_.data = 1;
        this->pub_respawn_.publish(this->regen_cmd_msg_);
    }
    void simplePanel::on_button_clear_clicked()
    {
        ROS_INFO_STREAM("Clearing Random Objects");
        ui_->label_status->setText("Please select a goal pose");
        this->regen_cmd_msg_.data = 0;
        this->pub_respawn_.publish(this->regen_cmd_msg_);
    }
    void simplePanel::on_button_cancel_goal_clicked()
    {
        ROS_INFO_STREAM("Cancelling move base goal");
        ui_->label_status->setText("Goal Cancelled, please select a goal pose");
        actionlib_msgs::GoalID cancel_goal;
        this->movebase_cancel_goal_.publish(cancel_goal);
    }
    void simplePanel::on_button_skip_forward_clicked()
    {
        ROS_INFO_STREAM("Called to skip forward a search");
        std_msgs::String skip_forward_data;
        this->skip_forward_pub_.publish(skip_forward_data);
    }
    void simplePanel::on_button_clear_costmap_clicked()
    {
        ROS_INFO_STREAM("Called to clear costmaps");
        std_srvs::Empty empty_srv;
        this->clear_costmap_client_.call(empty_srv);
    }

    void simplePanel::moveBaseStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
    {
        if (msg->status_list.size())
        {
            int status = msg->status_list.back().status;
            QString move_base_status = "";
            switch (status)
            {
                case 0:
                    move_base_status = "PENDING";
                    break;
                case 1:
                    move_base_status = "ACTIVE";
                    break;
                case 2:
                    move_base_status = "PREEMPTED";
                    break;
                case 3:
                    move_base_status = "SUCCEEDED";
                    break;
                case 4:
                    move_base_status = "ABORTED";
                    break;
                case 5:
                    move_base_status = "REJECTED";
                    break;
                case 6:
                    move_base_status = "PREEMPTING";
                    break;
                case 7:
                    move_base_status = "RECALLING";
                    break;
                case 8:
                    move_base_status = "RECALLED";
                    break;
                case 9:
                    move_base_status = "LOST";
                    break;
                default:
                    move_base_status = "Unknown";
            }
            ui_->move_base_status->setText("Move Base Status: " + move_base_status);
        }
        else
            ui_->move_base_status->setText("Move Base Status: Unknown");
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
