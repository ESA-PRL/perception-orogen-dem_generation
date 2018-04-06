#ifndef DEM_GENERATION_TASK_TASK_HPP
#define DEM_GENERATION_TASK_TASK_HPP

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <base/samples/Frame.hpp>
#include "dem_generation/TaskBase.hpp"
#include <frame_helper/FrameHelper.h>

#include <pcl/point_cloud.h>

#include <dem_generation/dem_generation.hpp>

#include <velodyne_lidar/pointcloudConvertHelper.hpp>
#include <velodyne_lidar/MultilevelLaserScan.h>

#include <telemetry_telecommand/Messages.hpp>

#include <unistd.h>

namespace dem_generation {

    class Task : public TaskBase
    {
    friend class TaskBase;
    protected:

        DEM myDEM;
        base::samples::DistanceImage distance_image;
        std::string camera_name, save_directory;
        base::samples::Pointcloud rock_pointcloud;
        pcl::PointCloud<pcl::PointXYZ> input_pointcloud;
        velodyne_lidar::MultilevelLaserScan input_laser_scan;
        std::vector<Eigen::Vector3d> points;
        std::vector<telemetry_telecommand::messages::Telecommand> telecommand_vec;
        telemetry_telecommand::messages::Telemetry telemetry;
        bool save_single_frame, save_both_frames, save_distance, save_dem, save_pc;
        int sync_count;
        frame_helper::FrameHelper left_conv; // used to rectify camera frames
        base::samples::frame::Frame leftFrameTarget;
        int getConversionCode(RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame);

    public:
        Task(std::string const& name = "dem_generation::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);
        ~Task();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();

        void setProducts();
        void writeTelemetry(std::string productPath,
                telemetry_telecommand::messages::ProductType type,
                base::Time timestamp);
        void generateTelemetryFromFrame();
        void generateTelemetryFromPC();

        // copied from GIcp.cpp / hpp
        void toPCLPointCloud(const ::base::samples::Pointcloud & pc, pcl::PointCloud< pcl::PointXYZ >& pcl_pc, double density = 1.0);
    };
}

#endif
