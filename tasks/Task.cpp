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
		myDEM.saveDistanceFrame(distance_image.data);

		
		_left_frame_rect.read(leftFrame);
		cv::Mat dst = frame_helper::FrameHelper::convertToCvMat(*leftFrame);
		cv::Mat dst2 = dst;
		if ( (camera_name=="FLOC") || (camera_name=="RLOC") )
		{
			_right_frame_rect.read(rightFrame);
			dst2 = frame_helper::FrameHelper::convertToCvMat(*rightFrame);
		}
		myDEM.setColorFrame(dst, dst2); // to opencv format. Basically set up so that first it is read to internal variable, then converted to opencv and sent over to my library
		myDEM.distance2pointCloud(distance_image.data);

		myDEM.pointCloud2Mesh();

		_mesh_path.write(myDEM.getMeshPath());
		_image_left_path.write(myDEM.getImageLeftPath());
		
		if ( (camera_name=="FLOC") || (camera_name=="RLOC") )
		{
			_image_right_path.write(myDEM.getImageRightPath());
		}
		_distance_frame_path.write(myDEM.getDistanceFramePath());
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
