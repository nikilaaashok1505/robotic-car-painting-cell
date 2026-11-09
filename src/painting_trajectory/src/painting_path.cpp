#include <memory>
#include <thread>
#include <vector>
#include <exception>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"

using MoveGroup = moveit::planning_interface::MoveGroupInterface;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    options.append_parameter_override(
        "robot_description_kinematics.arm.kinematics_solver",
        "kdl_kinematics_plugin/KDLKinematicsPlugin");

    options.append_parameter_override(
        "robot_description_kinematics.arm.kinematics_solver_search_resolution",
        0.005);

    options.append_parameter_override(
        "robot_description_kinematics.arm.kinematics_solver_timeout",
        0.5);

    auto node = std::make_shared<rclcpp::Node>(
        "painting_path_executor", options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    try
    {
        MoveGroup move_group(node, "arm");

        move_group.setEndEffectorLink("tool0");
        move_group.setPoseReferenceFrame("world");
        move_group.setPlanningTime(10.0);

        RCLCPP_INFO(node->get_logger(),
            "BONNET RASTER PAINTING PATH PLANNER STARTED");

        auto current = move_group.getCurrentPose("tool0").pose;

        RCLCPP_INFO(node->get_logger(),
            "Starting tool position: X=%.3f Y=%.3f Z=%.3f",
            current.position.x,
            current.position.y,
            current.position.z);

        // Small test raster around the CURRENT tool position.
        // This is not yet the actual car bonnet trajectory.

        const double width = 0.020;       // 20 mm
        const double height = 0.020;      // 20 mm
        const double pass_spacing = 0.005; // 5 mm
        const double step = 0.005;        // 5 mm

        std::vector<geometry_msgs::msg::Pose> waypoints;

        int passes = static_cast<int>(height / pass_spacing);

        for (int i = 0; i <= passes; i++)
        {
            double y = current.position.y +
                       (i * pass_spacing);

            double x_start = current.position.x;
            double x_end = current.position.x + width;

            geometry_msgs::msg::Pose start = current;
            geometry_msgs::msg::Pose end = current;

            start.position.y = y;
            end.position.y = y;

            if (i % 2 == 0)
            {
                start.position.x = x_start;
                end.position.x = x_end;
            }
            else
            {
                start.position.x = x_end;
                end.position.x = x_start;
            }

            int segments = static_cast<int>(width / step);

            for (int j = 0; j <= segments; j++)
            {
                double ratio =
                    static_cast<double>(j) / segments;

                auto point = start;

                point.position.x =
                    start.position.x +
                    ratio * (end.position.x - start.position.x);

                waypoints.push_back(point);
            }
        }

        RCLCPP_INFO(node->get_logger(),
            "Generated %zu bonnet raster waypoints.",
            waypoints.size());

        move_group.setStartStateToCurrentState();

        moveit_msgs::msg::RobotTrajectory trajectory;

        double fraction = move_group.computeCartesianPath(
            waypoints,
            0.005,
            0.0,
            trajectory,
            true);

        RCLCPP_INFO(node->get_logger(),
            "Cartesian completion: %.2f%%",
            fraction * 100.0);

        RCLCPP_INFO(node->get_logger(),
            "Trajectory points generated: %zu",
            trajectory.joint_trajectory.points.size());

        if (fraction >= 0.99 &&
            !trajectory.joint_trajectory.points.empty())
        {
            RCLCPP_INFO(node->get_logger(),
                "SUCCESS: Raster painting trajectory generated.");

            RCLCPP_INFO(node->get_logger(),
                "Test region: 20 mm x 20 mm");

            RCLCPP_INFO(node->get_logger(),
                "Pass spacing: 5 mm");
        }
        else
        {
            RCLCPP_WARN(node->get_logger(),
                "Raster painting trajectory incomplete.");
        }

        if (fraction >= 0.99 &&
            !trajectory.joint_trajectory.points.empty())
        {
            RCLCPP_WARN(node->get_logger(),
                "EXECUTING TEST RASTER IN SIMULATION...");

            auto result = move_group.execute(trajectory);

            if (result.val == 1)
            {
                RCLCPP_INFO(node->get_logger(),
                    "SUCCESS: Robot completed the test raster!");
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(),
                    "Execution failed. MoveIt error code: %d",
                    result.val);
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(),
                "Execution cancelled: incomplete trajectory.");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(),
            "Exception: %s", e.what());
    }

    executor.cancel();

    if (spin_thread.joinable())
        spin_thread.join();

    rclcpp::shutdown();

    return 0;
}
