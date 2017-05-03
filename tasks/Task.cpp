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
	myDEM.setCameraParameters(calib.width,calib.height,calib.cx,calib.cy, calib.fx, calib.fy);
	
	// set camera name and producer source
	camera_name = _camera_name.get();
	if(camera_name == "MAST")
		telemetry.productSource = telemetry_telecommand::messages::MAST;   
	else if(camera_name == "LIDAR")
		telemetry.productSource = telemetry_telecommand::messages::LIDAR;
	else if(camera_name == "TOF")
		telemetry.productSource = telemetry_telecommand::messages::TOF;
	else if(camera_name == "HAZCAM")
		telemetry.productSource = telemetry_telecommand::messages::HAZCAM;
	else if(camera_name == "REAR")
		telemetry.productSource = telemetry_telecommand::messages::REAR;
	else if(camera_name == "FRONT")
		telemetry.productSource = telemetry_telecommand::messages::FRONT;
	else
	{
		std::cout << " ERROR!! Unexisting or unsupported producer type/camera_name" << std::endl;
		return false; // wrong config
	}
	
	save_directory = _save_directory.get();   
	
	myDEM.setFileDestination(_save_directory.get(), _camera_name.get());
	
	Eigen::Vector4d a = _pointcloud_cut_min.get();
	Eigen::Vector4d b = _pointcloud_cut_max.get();
	myDEM.setPcFiltersParameters(a.cast<float>(), b.cast<float>(), _leaf_size.get(), _k_points.get());
	
	sync_count = 0;
     
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
   
	// New telecommand, time to process
	if(_telecommands_in.read(telecommand_vec) == RTT::NewData)
	{
		// set which data should be processed and saved
		this->setProducts();	
		
		// receive point cloud (tof is connected)
		if(_pointcloud.connected())
		{
			if(_pointcloud.read(rock_pointcloud) == RTT::NewData)
			{
				// convert point cloud to pcl format
				this->toPCLPointCloud(rock_pointcloud, input_pointcloud);
				
				// generate telemetry
				this->generateTelemetryFromPC();
			}
		}
		
		// receive laser scans (lidar is connected)
		else if(_laser_scans.connected())
		{
			if(_laser_scans.read(input_laser_scan) == RTT::NewData)
			{
				// convert laser scan to 3d points vector
				velodyne_lidar::ConvertHelper::convertScanToPointCloud(input_laser_scan, points);

				// fill pcl pointcloud using lidar 3d points
				input_pointcloud.points.resize(points.size());			
				for(unsigned int i = 0; i<input_pointcloud.size(); i++)
				{
					input_pointcloud.points[i].x = (float)points[i][0];
					input_pointcloud.points[i].y = (float)points[i][1];
					input_pointcloud.points[i].z = (float)points[i][2];
				}
				
				// generate telemetry
				this->generateTelemetryFromPC();
			}
		}
		
		// received only distance frame (stereocamera is connected)
		else if(_distance_frame.connected())
		{
			// only frame is needed
			if(!save_dem && !save_pc && !save_distance)
			{
				this->generateTelemetryFromFrame();
			}
			// if not, we need to wait for the distance frame coming from stereo (TO BE CHECKED IF IT WORKS PROPERLY)
			else
			{
				while(_distance_frame.read(distance_image) != RTT::NewData)
					usleep(1000);
				
				this->generateTelemetryFromFrame();
			}
		}
		else
			std::cout << "WARNING!! Nothing is connected to " << camera_name << " associated DEM generation component" << std::endl;
	
		// new process finished, write out sync port
		_sync_out.write(sync_count);
		sync_count++;
	
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

// This function eads the stored telecommands and sets the 
// image/distance/pointcloud/dem variables that will be used to process
// and save the products
void Task::setProducts()
{
	save_frame = false;
	save_distance = false;
	save_dem = false;
	save_pc = false;
	
	int tSize = telecommand_vec.size();
	for(int i = 0; i < tSize; i++)
	{
		if(telecommand_vec[i].productType == telemetry_telecommand::messages::IMAGE)
			save_frame = true;
		
		if(telecommand_vec[i].productType == telemetry_telecommand::messages::DISTANCE)
			save_distance = true;
			
		if(telecommand_vec[i].productType == telemetry_telecommand::messages::POINT_CLOUD)
			save_pc = true;
			
		if(telecommand_vec[i].productType == telemetry_telecommand::messages::DEM)
			save_dem = true;
			
	}
}

// fills telemetry fields and writes the mesage on the port
void Task::writeTelemetry(std::string productPath,
						telemetry_telecommand::messages::ProductType type,
						base::Time timestamp)
{
	telemetry.productPath = productPath;
	telemetry.type = type;
	telemetry.timestamp = timestamp;
	_telemetry_out.write(telemetry);
}

// generate the demanded telemetry starting from a frame
// to generate pc, dem must be generated as well. dist_frame is 
// the only one that does not need color frame to be saved. 
// saving pc without dem will still require the color to be saved
void Task::generateTelemetryFromFrame()
{
	RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> leftFrame, rightFrame;


	// convert frame to opencv format with proper color encoding (standard opencv is BGR)
	_left_frame_rect.read(leftFrame);
	cv::Mat dst, dst2;
	if(leftFrame->getFrameMode() == base::samples::frame::MODE_BAYER_RGGB ||
		leftFrame->getFrameMode() == base::samples::frame::MODE_BAYER_GRBG ||
		leftFrame->getFrameMode() == base::samples::frame::MODE_BAYER_BGGR ||
		leftFrame->getFrameMode() == base::samples::frame::MODE_BAYER_GBRG)
	{
		base::samples::frame::Frame rock_dst, frame_buffer3;
		rock_dst.init(leftFrame->getWidth(),leftFrame->getHeight(),leftFrame->getDataDepth(),base::samples::frame::MODE_RGB,-1);
		frame_buffer3.init( leftFrame->getWidth(), leftFrame->getHeight(), leftFrame->getDataDepth(),rock_dst.getFrameMode(),-1);
		frame_helper::FrameHelper::convertBayerToRGB24(leftFrame->getImageConstPtr(),frame_buffer3.getImagePtr(),leftFrame->getWidth(),leftFrame->getHeight(),leftFrame->frame_mode); 
		dst = frame_helper::FrameHelper::convertToCvMat(rock_dst);
		cv::cvtColor(frame_helper::FrameHelper::convertToCvMat(frame_buffer3),dst,cv::COLOR_RGB2BGR);
    }
    else if(leftFrame->getFrameMode() == base::samples::frame::MODE_RGB) // already in RGB mode
    {
		dst = frame_helper::FrameHelper::convertToCvMat(*leftFrame);
		cv::cvtColor(dst, dst, cv::COLOR_RGB2BGR);
	}
	else
		std::cerr << "[DEM GENERATION OROGEN]Color frame with non supported encoding, add conversion here " << leftFrame->getFrameMode() << "\n";
	
	dst2 = dst;
	myDEM.setTimestamp(leftFrame->time.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_"));

	// unless only distance frame is recquired, save color frame
	//if(save_frame || save_dem || save_pc || save_distance)
	myDEM.setColorFrame(dst, dst2); // to opencv format. Basically set up so that first it is read to internal variable, then converted to opencv and sent over to my library

	// if frame or dem are recquired, send frame as telemetry
	if(save_frame || save_dem || save_distance)
	{
		this->writeTelemetry(myDEM.getImageLeftPath(),
			telemetry_telecommand::messages::IMAGE,
			leftFrame->time);
	}

	// if wanted, save distance and send as telemetry
	if(save_distance)
	{
		myDEM.saveDistanceFrame(distance_image.data);	
		this->writeTelemetry(myDEM.getDistanceFramePath(),
			telemetry_telecommand::messages::DISTANCE,
			leftFrame->time);
	}

	// generate dem
	if(save_dem)
	{
		myDEM.distance2pointCloud(distance_image.data);
		myDEM.filterPointCloud();
		myDEM.pointCloud2Mesh(true); // generate from filtered pointcloud
		this->writeTelemetry(myDEM.getMeshPath(),
			telemetry_telecommand::messages::DEM,
			leftFrame->time);
	}

	// save pc and send as telemetry
	if(save_pc)
	{
		myDEM.filterPointCloud();
		myDEM.savePointCloud(true); // save filtered pointcloud
		this->writeTelemetry(myDEM.getPointCloudPath(),
			telemetry_telecommand::messages::POINT_CLOUD,
			leftFrame->time);
	}
}

// generate the demanded telemetry starting from a point cloud
void Task::generateTelemetryFromPC()
{
	RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> leftFrame, rightFrame;

	// convert intensity frame to opencv format
	_left_frame_rect.read(leftFrame);
	cv::Mat dst = frame_helper::FrameHelper::convertToCvMat(*leftFrame);
	cv::Mat dst2 = dst;

	// generate products
	myDEM.setTimestamp(leftFrame->time.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_"));
	
	// unless only distance frame is recquired, save color frame
	//if(save_frame || save_dem || save_pc || save_distance)
		myDEM.setColorFrame(dst, dst2);
		
	// if frame or dem are recquired, send frame as telemetry
	if(save_frame || save_dem || save_distance)
	{
		this->writeTelemetry(myDEM.getImageLeftPath(),
			telemetry_telecommand::messages::IMAGE,
			leftFrame->time);
	}
	
	// if wanted, save distance and send as telemetry
	if(save_distance)
	{
		_distance_frame.read(distance_image);
		myDEM.saveDistanceFrame(distance_image.data);	
		this->writeTelemetry(myDEM.getDistanceFramePath(),
			telemetry_telecommand::messages::DISTANCE,
			leftFrame->time);
	}
	
	// input pointcloud
	if(save_pc || save_dem)
		myDEM.setPointCloud(input_pointcloud);

	// generate dem and send as telemetry
	if(save_dem)
	{
		myDEM.filterPointCloud();
		myDEM.pointCloud2Mesh(true); // generate from filtered pointcloud
		this->writeTelemetry(myDEM.getMeshPath(),
			telemetry_telecommand::messages::DEM,
			leftFrame->time);
	}
	
	// save pointcloud and send as telemetry
	if(save_pc)
	{
		myDEM.filterPointCloud();
		myDEM.savePointCloud(true); // save filtered pointcloud
		this->writeTelemetry(myDEM.getPointCloudPath(),
			telemetry_telecommand::messages::POINT_CLOUD,
			leftFrame->time);
	}
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
