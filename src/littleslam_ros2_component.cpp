#include "littleslam_ros2/littleslam_ros2_component.hpp"

#include <class_loader/register_macro.hpp>
#include <chrono>
#include <memory>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>

using namespace std::chrono_literals;

namespace littleslam_ros2
{

    /**
     * @brief transform_pcl_pointcloud
     *
     * @tparam POINT_TYPE pcl::PointXYZ
     * @param tf_buffer tf2_ros::Buffer
     * @param source_frame std::string 変換元のフレーム名
     * @param target_frame std::string 変換先のフレーム名
     * @param source_cloud pcl::PointCloud<POINT_TYPE>
     * @param time rclcpp::Time
     * @return std::optional<pcl::PointCloud<POINT_TYPE>>
     */
    template <typename POINT_TYPE = pcl::PointXYZ>
    inline std::optional<pcl::PointCloud<POINT_TYPE>> transform_pcl_pointcloud(const tf2_ros::Buffer &tf_buffer, std::string source_frame,
                                                                               std::string target_frame, const pcl::PointCloud<POINT_TYPE> &source_cloud,
                                                                               const rclcpp::Time &time = rclcpp::Time(0))
    {
        std::optional<pcl::PointCloud<POINT_TYPE>> ret = std::nullopt;
        try
        {
            pcl::PointCloud<POINT_TYPE> target_cloud;
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer.lookupTransform(target_frame, source_frame, time, tf2::durationFromSec(0.0));
            Eigen::Affine3f affine_conversion(tf2::transformToEigen(transform_stamped).affine().cast<float>());
            pcl::transformPointCloud(source_cloud, target_cloud, affine_conversion);
            ret = target_cloud;
        }
        catch (tf2::LookupException &ex)
        {
            std::cerr << "[ERROR]" << ex.what() << std::endl;
        }
        catch (tf2::ExtrapolationException &ex)
        {
            std::cerr << "[ERROR]" << ex.what() << std::endl;
        }
        return ret;
    }

    Littleslam::Littleslam()
        : Node("littleslam"), broadcaster_(this), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
    {
        declare_parameter("use_odom", false);
        get_parameter("use_odom", use_odom_);

        RCLCPP_INFO(this->get_logger(), "use_odom: %d", use_odom_);

        fc_.setSlamFrontEnd(sf_);
        fc_.makeFramework();
        if (use_odom_)
        {
            fc_.customizeI();
        }
        else
            fc_.customizeG();
        auto scan_callback =
            [this](const typename sensor_msgs::msg::LaserScan::SharedPtr msg) -> void
        {
            Scan2D scan2d;
            if (make_scan2d(scan2d, msg))
                sf_->process(scan2d);
        };

        laser_sub_ =
            create_subscription<sensor_msgs::msg::LaserScan>("scan", 100,
                                                             scan_callback);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 100, [this](const typename nav_msgs::msg::Odometry::SharedPtr msg) -> void
            { odom_ = *msg; });

        icp_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("icp_map", 10);
        in_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("in_cloud", 10);

        path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);

        current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);

        timer_ = create_wall_timer(10ms, [this]()
                                   { broadcast_littleslam(); });
        // timer_ = create_wall_timer(100ms, [this]()
        //                            { broadcast_littleslam(); });
    }

    double Littleslam::get_yaw(const geometry_msgs::msg::Quaternion &geometry_quat)
    {
        tf2::Quaternion tf_quat;
        tf2::convert(geometry_quat, tf_quat);
        tf2::Matrix3x3 m(tf_quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    bool Littleslam::make_scan2d(Scan2D &scan2d, const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {

        if (use_odom_)
        {
            if (odom_)
            {
                scan2d.pose.tx = odom_->pose.pose.position.x;
                scan2d.pose.ty = odom_->pose.pose.position.y;
                scan2d.pose.th = RAD2DEG(get_yaw(odom_->pose.pose.orientation));
                scan2d.pose.calRmat();
            }
            // tf2::Stamped<tf2::Transform> tr;

            // try{
            //     builtin_interfaces::msg::Time time_stamp = scan->header.stamp;
            //     tf2::TimePoint time_point = tf2::TimePoint(
            //         std::chrono::seconds(time_stamp.sec) +
            //         std::chrono::nanoseconds(time_stamp.nanosec));
            //     tf2::TimePoint time_out;

            //     geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
            //         "odom", "base_link", rclcpp::Time(0), tf2::durationFromSec(0.0));

            //     tf2::fromMsg(tf, tr);
            // }
            // catch (tf2::TransformException &ex) {
            //     RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
            //     return false;
            // }
            // scan2d.pose.tx = tr.getOrigin().x();
            // scan2d.pose.ty = tr.getOrigin().y();
            // scan2d.pose.th = RAD2DEG(tf2::impl::getYaw(tr.getRotation()));
            // scan2d.pose.calRmat();
        }
        else
        {
            if (map_ == nullptr)
            {
                sf_->process(scan2d);
                map_ = sf_->getPointCloudMap();
            }
            Pose2D curPose = map_->getLastPose();
            scan2d.pose = curPose;
        }
        sensor_msgs::msg::PointCloud2 get_cloud;
        projector_.projectLaser(*scan, get_cloud);
        get_cloud.header.stamp = this->now();
        in_cloud_pub_->publish(get_cloud);
        pcl::PointCloud<pcl::PointXYZ> now_cloud;
        pcl::fromROSMsg(get_cloud, now_cloud);
        std::optional<pcl::PointCloud<pcl::PointXYZ>> tf_cloud = transform_pcl_pointcloud(
            tf_buffer_, scan->header.frame_id, "base_link", now_cloud);
        if (!tf_cloud.has_value())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform point cloud");
            return false;
        }
        scan2d.lps.clear();
        for (size_t i = 0; i < tf_cloud.value().points.size(); ++i)
        {
            LPoint2D lp;
            lp.x = tf_cloud.value().points[i].x;
            lp.y = tf_cloud.value().points[i].y;
            scan2d.lps.push_back(lp);
        }
        return true;
    }

    void Littleslam::broadcast_littleslam()
    {
        map_ = sf_->getPointCloudMap();

        pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

        msg->header.frame_id = "map";
        msg->height = msg->width = 1;
        for (auto lp : map_->globalMap)
            msg->points.push_back(pcl::PointXYZ(lp.x, lp.y, 0));
        msg->width = msg->points.size();

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = this->now();
        pcl::toROSMsg(*msg, cloud);
        icp_map_pub_->publish(cloud);

        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        pose_.header.frame_id = "map";
        pose_.header.stamp = this->now();
        for (auto pos : map_->poses)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = pos.tx;
            pose.pose.position.y = pos.ty;
            pose.pose.position.z = 0;
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(0.0, 0.0, DEG2RAD(pos.th));
            geometry_msgs::msg::Quaternion quat_msg;
            quat_msg = tf2::toMsg(quat_tf);
            pose.pose.orientation = quat_msg;
            path.poses.push_back(pose);
            if (pos.tx == map_->lastPose.tx &&
                pos.ty == map_->lastPose.ty &&
                pos.th == map_->lastPose.th)
            {
                pose.header.frame_id = "map";
                pose.header.stamp = this->now();
                pose_ = pose;
                current_pose_pub_->publish(pose);
            }
        }
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header = pose_.header;
        transform_stamped.child_frame_id = "base_link";
        transform_stamped.transform.translation.x = pose_.pose.position.x;
        transform_stamped.transform.translation.y = pose_.pose.position.y;
        transform_stamped.transform.translation.z = pose_.pose.position.z;
        transform_stamped.transform.rotation = pose_.pose.orientation;
        broadcaster_.sendTransform(transform_stamped);
        path_pub_->publish(path);
    }

} // namespace littleslam_ros2

CLASS_LOADER_REGISTER_CLASS(littleslam_ros2::Littleslam, rclcpp::Node)
