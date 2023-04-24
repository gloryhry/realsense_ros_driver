/*
 * _______________#########_______________________
 * ______________############_____________________
 * ______________#############____________________
 * _____________##__###########___________________
 * ____________###__######_#####__________________
 * ____________###_#######___####_________________
 * ___________###__##########_####________________
 * __________####__###########_####_______________
 * ________#####___###########__#####_____________
 * _______######___###_########___#####___________
 * _______#####___###___########___######_________
 * ______######___###__###########___######_______
 * _____######___####_##############__######______
 * ____#######__#####################_#######_____
 * ____#######__##############################____
 * ___#######__######_#################_#######___
 * ___#######__######_######_#########___######___
 * ___#######____##__######___######_____######___
 * ___#######________######____#####_____#####____
 * ____######________#####_____#####_____####_____
 * _____#####________####______#####_____###______
 * ______#####______;###________###______#________
 * ________##_______####________####______________
 *
 * @Author: Glory Huang
 * @Date: 2023-04-18 20:27:11
 * @LastEditors: Glory Huang
 * @LastEditTime: 2023-04-19 00:12:00
 * @Page: https://xjtuglory.ml
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#ifndef REALSENSE_DRIVER_H_
#define REALSENSE_DRIVER_H_

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
// #include <example.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#define Align_To_Color 0
#define Align_To_Infrared 1

class Camera
{
public:
    Camera() {}
    Camera(int number_, std::string serial_, bool color_, bool depth_, bool infrared_, bool pointcloud_, int align_, ros::NodeHandle nh_)
    {
        nh = nh_;
        camera_number = number_;
        camera_serial_number = serial_;
        color = color_;
        depth = depth_;
        infrared = infrared_;
        pointcloud = pointcloud_;
        align_to = align_;
        pipe_config.enable_device(camera_serial_number);
        if (color)
        {
            pipe_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        }
        if (depth)
        {
            pipe_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
        }
        if (infrared)
        {
            pipe_config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
        }

        profile = pipe.start(pipe_config);
        // if (depth)
        // {
        //     // auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        //     // const auto depth_intrinsics = depth_stream.get_intrinsics();
        //     float depth_scale = get_depth_scale(profile.get_device());
        //     std::cout << "Camera" << camera_number << " depth_scale = " << depth_scale << std::endl;
        // }
        // if (color)
        // {
        //     auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        //     auto color_intrinsics = color_stream.get_intrinsics();
        // }
        // if (infrared)
        // {
        //     auto infrared_stream = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
        //     auto infrared_intrinsics = infrared_stream.get_intrinsics();
        // }

        // 选择彩色图像数据流来作为对齐对象

        // 对齐的是彩色图，所以彩色图是不变的
        if (align_to == Align_To_Color)
        {
            aligned = RS2_STREAM_COLOR;
        }
        if (align_to == Align_To_Infrared)
        {
            aligned = RS2_STREAM_INFRARED;
        }
        // 将深度图对齐到RGB图
        align = rs2::align(aligned);
    }
    void set_Camera(int number_, std::string serial_, bool color_, bool depth_, bool infrared_, bool pointcloud_, int align_, ros::NodeHandle nh_)
    {
        nh = nh_;
        camera_number = number_;
        camera_serial_number = serial_;
        color = color_;
        depth = depth_;
        infrared = infrared_;
        pointcloud = pointcloud_;
        align_to = align_;
        pipe_config.enable_device(camera_serial_number);
        if (depth)
        {
            pipe_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        }
        if (color)
        {
            pipe_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
        }
        if (infrared)
        {
            pipe_config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
        }

        profile = pipe.start(pipe_config);
        // if (depth)
        // {
        //     // auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        //     // const auto depth_intrinsics = depth_stream.get_intrinsics();
        //     float depth_scale = get_depth_scale(profile.get_device());
        //     std::cout << "Camera" << camera_number << " depth_scale = " << depth_scale << std::endl;
        // }
        // if (color)
        // {
        //     auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        //     auto color_intrinsics = color_stream.get_intrinsics();
        // }
        // if (infrared)
        // {
        //     auto infrared_stream = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
        //     auto infrared_intrinsics = infrared_stream.get_intrinsics();
        // }

        // 选择彩色图像数据流来作为对齐对象
        // 对齐的是彩色图，所以彩色图是不变的
        if (align_to == Align_To_Color)
        {
            aligned = RS2_STREAM_COLOR;
        }
        if (align_to == Align_To_Infrared)
        {
            aligned = RS2_STREAM_INFRARED;
        }
        // 将深度图对齐到RGB图
        align = rs2::align(aligned);
    }
    void set_color_pub(std::string color_topic)
    {
        color_pub = nh.advertise<sensor_msgs::Image>(color_topic, 1);
        color_init = true;
    }
    void set_depth_pub(std::string depth_topic)
    {
        depth_pub = nh.advertise<sensor_msgs::Image>(depth_topic, 1);
        depth_init = true;
    }
    void set_infrared_pub(std::string infrared_topic)
    {
        infrared_pub = nh.advertise<sensor_msgs::Image>(infrared_topic, 1);
        infrared_init = true;
    }
    void set_pointcloud_pub(std::string pointcloud_topic)
    {
        pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1);
        pointcloud_init = true;
    }
    ~Camera() {}

public:
    ros::NodeHandle nh;
    int align_to = 0;
    bool color = true, depth = false, infrared = false, pointcloud = false;
    int camera_number;
    std::string camera_serial_number;
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config pipe_config;
    rs2::pipeline_profile profile;
    rs2_stream aligned;
    rs2::align align = rs2::align(RS2_STREAM_COLOR);
    cv::Mat color_raw;
    cv::Mat depth_raw;
    cv::Mat infrared_raw;
    ros::Publisher color_pub;
    ros::Publisher depth_pub;
    ros::Publisher infrared_pub;
    ros::Publisher pointcloud_pub;
    bool color_init = false, depth_init = false, infrared_init = false, pointcloud_init;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

public:
    float get_depth_scale(rs2::device dev)
    {
        // 遍历设备的传感器
        for (rs2::sensor &sensor : dev.query_sensors())
        {
            // 检查传感器是否是深度传感器
            if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
                return dpt.get_depth_scale();
        }
        throw std::runtime_error("Device does not have a depth sensor");
    }

    bool profile_changed(const std::vector<rs2::stream_profile> &current, const std::vector<rs2::stream_profile> &prev)
    {
        for (auto &&sp : prev)
        {
            // if previous profile is in current ( maybe just added another)
            auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile &current_sp)
                                    { return sp.unique_id() == current_sp.unique_id(); });
            if (itr == std::end(current))
            {
                return true;
            }
        }
        return false;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points &points, cv::Mat color_raw)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        cloud->width = sp.width();
        cloud->height = sp.height();
        cloud->is_dense = false;
        cloud->points.resize(points.size());
        auto ptr = points.get_vertices();
        auto tex = points.get_texture_coordinates();
        cv::Vec3b vc3;
        for (auto &p : cloud->points)
        {
            p.x = ptr->x;
            p.y = ptr->y;
            p.z = ptr->z;
            vc3 = color_raw.at<cv::Vec3b>((tex->v) * (color_raw.rows), (tex->u) * (color_raw.cols));
            p.r = vc3.val[0];
            p.g = vc3.val[1];
            p.b = vc3.val[2];
            ptr++;
            tex++;
        }

        return cloud;
    }

    int rs_imshow()
    {
        try
        {
            while (ros::ok())
            {
                rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
                ros::Time now_time = ros::Time::now();
                // 正在对齐深度图到其他图像流，我们要确保对齐的图像流不发生改变
                if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
                {
                    // 如果profile发生改变，则更新align对象，重新获取深度图像像素到长度单位的转换比例
                    profile = pipe.get_active_profile();
                    align = rs2::align(aligned);
                    // float depth_scale = get_depth_scale(profile.get_device());
                }
                // 获取对齐后的帧
                auto processed = align.process(data);

                // 尝试获取对齐后的深度图像帧和其他帧
                rs2::frame aligned_color_frame, aligned_depth_frame, aligned_infrared_frame;
                if (color)
                {
                    aligned_color_frame = processed.get_color_frame(); // RGB图
                }
                if (depth)
                {
                    aligned_depth_frame = processed.get_depth_frame(); // 深度图
                }
                if (infrared)
                {
                    aligned_infrared_frame = processed.get_infrared_frame(); // 红外图
                }
                if (depth && pointcloud)
                {
                    // Generate the pointcloud and texture mappings
                    points = pc.calculate(aligned_depth_frame);
                    // Tell pointcloud object to map to this color frame
                    pc.map_to(aligned_color_frame);
                }

                // Query frame size (width and height)
                int color_w, color_h, depth_w, depth_h, infrared_w, infrared_h;
                if (aligned_color_frame)
                {
                    color_w = aligned_color_frame.as<rs2::video_frame>().get_width();
                    color_h = aligned_color_frame.as<rs2::video_frame>().get_height();
                    color_raw = cv::Mat(cv::Size(color_w, color_h), CV_8UC3, (void *)aligned_color_frame.get_data(), cv::Mat::AUTO_STEP);
                }
                if (aligned_depth_frame)
                {
                    depth_w = aligned_depth_frame.as<rs2::video_frame>().get_width();
                    depth_h = aligned_depth_frame.as<rs2::video_frame>().get_height();
                    depth_raw = cv::Mat(cv::Size(depth_w, depth_h), CV_16UC1, (void *)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
                }
                if (aligned_infrared_frame)
                {
                    infrared_w = aligned_infrared_frame.as<rs2::video_frame>().get_width();
                    infrared_h = aligned_infrared_frame.as<rs2::video_frame>().get_height();
                    infrared_raw = cv::Mat(cv::Size(infrared_w, infrared_h), CV_8UC1, (void *)aligned_infrared_frame.get_data());
                }

                std_msgs::Header header;
                header.stamp = now_time;

                if (color && color_init)
                {
                    header.frame_id = "camera" + std::to_string(camera_number) + "_color_optical_frame";
                    sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(header, "rgb8", color_raw).toImageMsg();
                    color_pub.publish(*color_msg);
                }
                if (depth && depth_init)
                {
                    header.frame_id = "camera" + std::to_string(camera_number) + "_depth_optical_frame";
                    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(header, "mono16", depth_raw).toImageMsg();
                    depth_pub.publish(*depth_msg);
                }
                if (infrared && infrared_init)
                {
                    header.frame_id = "camera" + std::to_string(camera_number) + "_infrared_optical_frame";
                    sensor_msgs::ImagePtr infrared_msg = cv_bridge::CvImage(header, "mono8", infrared_raw).toImageMsg();
                    infrared_pub.publish(*infrared_msg);
                }
                if (depth && pointcloud && pointcloud_init)
                {
                    cloud = points_to_pcl(points, color_raw);
                    header.frame_id = "camera" + std::to_string(camera_number) + "_depth_frame";
                    sensor_msgs::PointCloud2 camera_pointcloud;
                    pcl::toROSMsg(*cloud, camera_pointcloud);
                    camera_pointcloud.header = header;
                    pointcloud_pub.publish(camera_pointcloud);
                }
            }
            return EXIT_SUCCESS;
        }
        catch (const rs2::error &e)
        {
            std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }
};

#endif