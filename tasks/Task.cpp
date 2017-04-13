/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace dem_generation;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
        
    frame_helper::CameraCalibration calib = _cameraCalibration.get();    
	myDEM.setCameraParameters(calib.width,calib.height,calib.cx,calib.cy, calib.fx, calib.fy); // TODO PUT PARAMETERS
	camera_name = _camera_name.get();
	save_directory = _save_directory.get();   
	
	myDEM.setFileDestination(_save_directory.get(), _camera_name.get());
	
	Eigen::Vector4d a = _pointcloud_cut_min.get();
	Eigen::Vector4d b = _pointcloud_cut_max.get();
	myDEM.setPcFiltersParameters(a.cast<float>(), b.cast<float>(), _leaf_size.get(), _k_points.get());
     
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
	RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> leftFrame, rightFrame;

	// receive normal frame
	if(_distance_frame.connected())
	{
		if(_distance_frame.read(distance_image) == RTT::NewData)
		{
			// convert frame to opencv format
			_left_frame_rect.read(leftFrame);
			cv::Mat dst = frame_helper::FrameHelper::convertToCvMat(*leftFrame);
			cv::Mat dst2 = dst;
			
			// generate products
			myDEM.setTimestamp(distance_image.time.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_"));
			myDEM.saveDistanceFrame(distance_image.data);	
			myDEM.setColorFrame(dst, dst2); // to opencv format. Basically set up so that first it is read to internal variable, then converted to opencv and sent over to my library
			myDEM.distance2pointCloud(distance_image.data);
			myDEM.pointCloud2Mesh();
			myDEM.savePointCloud(true);

			// write product outputs paths
			_distance_frame_path.write(myDEM.getDistanceFramePath());
			_mesh_path.write(myDEM.getMeshPath());
			_image_left_path.write(myDEM.getImageLeftPath());

		}
	}
	
	// receive point cloud
	if(_pointcloud.connected())
	{
		if(_pointcloud.read(rock_pointcloud) == RTT::NewData)
		{
			// convert point cloud to pcl format
			this->toPCLPointCloud(rock_pointcloud, input_pointcloud);
			
			// convert intensity frame to opencv format
			_left_frame_rect.read(leftFrame);
			cv::Mat dst = frame_helper::FrameHelper::convertToCvMat(*leftFrame);
			cv::Mat dst2 = dst;
			
			// generate products
			myDEM.setTimestamp(rock_pointcloud.time.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_"));
			myDEM.setColorFrame(dst, dst2);
			myDEM.setPointCloud(input_pointcloud);
			myDEM.pointCloud2Mesh();
			myDEM.savePointCloud(true);
			
			// write product outputs paths
			_distance_frame_path.write(myDEM.getDistanceFramePath());
			_mesh_path.write(myDEM.getMeshPath());
			_image_left_path.write(myDEM.getImageLeftPath());
		}
	}
	
	// receive laser scans
	if(_laser_scans.connected())
	{
		if(_laser_scans.read(input_laser_scan) == RTT::NewData)
		{
			// convert laser scan to 3d points vector
			velodyne_lidar::ConvertHelper::convertScanToPointCloud(input_laser_scan, points);

			// fill pcl pointcloud using lidar 3d points
			input_pointcloud.points.resize(points.size());			
			for(unsigned int i = 0; i<input_pointcloud.size(); i++)
			{
				input_pointcloud.points[i].x = (float)points[i][0]/10.0;
				input_pointcloud.points[i].y = (float)points[i][1]/10.0;
				input_pointcloud.points[i].z = (float)points[i][2]/10.0;
			}
			
			// convert intensity frame to opencv format
			_left_frame_rect.read(leftFrame);
			cv::Mat dst = frame_helper::FrameHelper::convertToCvMat(*leftFrame);
			cv::Mat dst2 = dst;
			
			// generate products
			myDEM.setTimestamp(input_laser_scan.time.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_"));
			myDEM.setColorFrame(dst, dst2);
			myDEM.setPointCloud(input_pointcloud);
			myDEM.pointCloud2Mesh();
			myDEM.savePointCloud(false); // do not save filtered pointcloud

			// write product outputs paths
			_distance_frame_path.write(myDEM.getDistanceFramePath());
			_mesh_path.write(myDEM.getMeshPath());
			_image_left_path.write(myDEM.getImageLeftPath());
		}
	}
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

// copied from GIcp.cpp / hpp
void Task::toPCLPointCloud(const ::base::samples::Pointcloud & pc, pcl::PointCloud< pcl::PointXYZ >& pcl_pc, double density)
{
    pcl_pc.clear();
    std::vector<bool> mask;
    unsigned sample_count = (unsigned)(density * pc.points.size());

    if(density <= 0.0 || pc.points.size() == 0)
    {
        return;
    }
    else if(sample_count >= pc.points.size())
    {
        mask.resize(pc.points.size(), true);
    }
    else
    {
        mask.resize(pc.points.size(), false);
        unsigned samples_drawn = 0;

        while(samples_drawn < sample_count)
        {
            unsigned index = rand() % pc.points.size();
            if(mask[index] == false)
            {
                mask[index] = true;
                samples_drawn++;
            }
        }
    }

    for(size_t i = 0; i < pc.points.size(); ++i)
    {
        if(mask[i])
        {
            if (base::isnotnan<base::Point>(pc.points[i]))
            {
                /** Depth info **/
                pcl_pc.push_back(pcl::PointXYZ(pc.points[i].x(), pc.points[i].y(), pc.points[i].z()));
            }
        }
    }

    /** All data points are finite (no NaN or Infinite) **/
    pcl_pc.is_dense = true;
}
