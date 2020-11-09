#pragma once

#include <frame_helper/FrameHelper.h>
#include <pcl/point_cloud.h>
#include <unistd.h>
#include <velodyne_lidar/MultilevelLaserScan.h>
#include <base-logging/Logging.hpp>
#include <base/samples/Frame.hpp>
#include <dem_generation/dem_generation.hpp>
#include <telemetry_telecommand/Messages.hpp>
#include <velodyne_lidar/pointcloudConvertHelper.hpp>
#include "dem_generation/TaskBase.hpp"

namespace dem_generation
{

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

    // Used to rectify camera frames
    frame_helper::FrameHelper frame_helper;

    base::samples::frame::Frame leftFrameTarget;

    // Returns the conversion code to be used in OpenCV's cvtColor(..) to
    // convert from the color mode of the passed frame (pointer) to BGR.
    // Returns -1 if requested mode is not supported.
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

    // This function reads the stored telecommands and sets the
    // image/stereo/distance/pointcloud/dem variables that will be used to process
    // and save the products
    void setProducts();

    // Fills telemetry fields and writes the mesage on the port
    void writeTelemetry(std::string productPath,
                        telemetry_telecommand::messages::ProductType type,
                        base::Time timestamp);

    // Generates the demanded telemetry starting from a frame
    // to generate pc, dem must be generated as well.
    // dist_frame is the only one that does not need color frame to be saved.
    // saving pc without dem will still require the color to be saved
    void generateTelemetryFromFrame();

    // Generates the demanded telemetry starting from a point cloud
    void generateTelemetryFromPc();

    // copied from GIcp.cpp / hpp
    void toPclPointCloud(const ::base::samples::Pointcloud& pc,
                         pcl::PointCloud<pcl::PointXYZ>& pcl_pc,
                         double density = 1.0);
};
}
