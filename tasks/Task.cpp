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

	if(_distance_frame.read(distance_image) == RTT::NewData)
	{
		myDEM.setTimestamp(distance_image.time.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_"));
		myDEM.saveDistanceFrame(distance_image.data);
		_distance_frame_path.write(myDEM.getDistanceFramePath());
		
		_left_frame_rect.read(leftFrame);
		cv::Mat dst = frame_helper::FrameHelper::convertToCvMat(*leftFrame);
		cv::Mat dst2 = dst;
		if ( (camera_name=="FLOC_STEREO") || (camera_name=="RLOC_STEREO") )
		{
			_right_frame_rect.read(rightFrame);
			dst2 = frame_helper::FrameHelper::convertToCvMat(*rightFrame);
		}
		myDEM.setColorFrame(dst, dst2); // to opencv format. Basically set up so that first it is read to internal variable, then converted to opencv and sent over to my library
		myDEM.distance2pointCloud(distance_image.data);

		myDEM.pointCloud2Mesh();
		
		myDEM.savePointCloud();

		_mesh_path.write(myDEM.getMeshPath());
		_image_left_path.write(myDEM.getImageLeftPath());
		
		if ( (camera_name=="FLOC_STEREO") || (camera_name=="RLOC_STEREO") )
		{
			_image_right_path.write(myDEM.getImageRightPath());
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
