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
	
	myDEM.compressProducts(_enable_compression.get(), _compression_level.get());
	
	left_conv.setCalibrationParameter(calib);


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
			this->generateTelemetryFromFrame();
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
	save_single_frame = false;
	save_both_frames = false;
	save_distance = false;
	save_dem = false;
	save_pc = false;
	
	int tSize = telecommand_vec.size();
	for(int i = 0; i < tSize; i++)
	{
		if(telecommand_vec[i].productType == telemetry_telecommand::messages::IMAGE)
			save_single_frame = true;

		if(telecommand_vec[i].productType == telemetry_telecommand::messages::STEREO)
			save_both_frames = true;
		
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

	// no need to check newdata because it is sent before te tc for sure
	_left_frame_rect.read(leftFrame);
	std::cout << "left frame time " << leftFrame->time << std::endl;

    // right frame is sent the same way as the left frame,
    // without intermediate processing, so we should not have
    // to wait for the frame to arrive.
    if(save_both_frames)
    {
        _right_frame_rect.read(rightFrame);
        std::cout << "right frame time " << rightFrame->time << std::endl;
    }

	// if not only frame we need to wait for the distance frame coming from stereo (TO BE CHECKED IF IT WORKS PROPERLY)
	if(save_dem || save_pc || save_distance)
	{
		bool sync = false;
		int wait_count = 0;
		// for some reason, same distance ireturn READ::NEW twice
		// hand syynchro is then required
		while(!sync)
		{
			int64_t time_diff = distance_image.time.toMicroseconds() - leftFrame->time.toMicroseconds();
			if (time_diff > 0) // distance_image in the future (should never happen)
			{
				sync = true; // should not happens, but at least program not stuck
				std::cout << "WARNING FROM DEM_GENERATION, DISTANCE FRAME COMING FROM THE FUTURE TO KILL US!!\n";
			}
			else if(time_diff <0) // must wait for new distance image
			{
				_distance_frame.read(distance_image);
				usleep(1000);
			}
			else // ok, in sync
			{
				sync = true;
			}
			wait_count++;
			if(wait_count > 5000) // 5 seconds wait for a distance frame
			{
				std::cout << "WARNING FROM DEM_GENERATION, TOOK TOO MUCH TIME TO RECEIVE A DISTANCE FRAME THERE IS A SERIOUS ISSUE SOMEWHERE\n";
				return;
			}
		}
		std::cout << "distance time " << distance_image.time << std::endl;
	}

	// convert frame to opencv format with proper color encoding (standard opencv is BGR)
	cv::Mat dst, dst2;
	base::samples::frame::Frame rock_dst, frame_buffer3;
	rock_dst.init(leftFrame->getWidth(),leftFrame->getHeight(),leftFrame->getDataDepth(),base::samples::frame::MODE_RGB,-1);

	if(leftFrame->getFrameMode() == base::samples::frame::MODE_BAYER_RGGB ||
		leftFrame->getFrameMode() == base::samples::frame::MODE_BAYER_GRBG ||
		leftFrame->getFrameMode() == base::samples::frame::MODE_BAYER_BGGR ||
		leftFrame->getFrameMode() == base::samples::frame::MODE_BAYER_GBRG)
	{
		frame_buffer3.init( leftFrame->getWidth(), leftFrame->getHeight(), leftFrame->getDataDepth(),rock_dst.getFrameMode(),-1);
		frame_helper::FrameHelper::convertBayerToRGB24(leftFrame->getImageConstPtr(),frame_buffer3.getImagePtr(),leftFrame->getWidth(),leftFrame->getHeight(),leftFrame->frame_mode);
		left_conv.convert( rock_dst, rock_dst, 0, 0, frame_helper::INTER_LINEAR, true );	// rectification of frame
		dst = frame_helper::FrameHelper::convertToCvMat(rock_dst);
		cv::cvtColor(frame_helper::FrameHelper::convertToCvMat(frame_buffer3),dst,cv::COLOR_RGB2BGR);
    }
    else if(leftFrame->getFrameMode() == base::samples::frame::MODE_RGB) // already in RGB mode
    {
		left_conv.convert( *leftFrame, rock_dst, 0, 0, frame_helper::INTER_LINEAR, true );	// rectification of frame
		dst = frame_helper::FrameHelper::convertToCvMat(rock_dst);
		cv::cvtColor(dst, dst, cv::COLOR_RGB2BGR);
	}
	else
    {
		std::cerr << "[DEM GENERATION OROGEN] Color frame with non supported encoding, add conversion here " << leftFrame->getFrameMode() << "\n";
    }
	
	dst2 = dst;
	myDEM.setTimestamp(leftFrame->time.toString(base::Time::Milliseconds,"%Y%m%d_%H%M%S_"));

	// unless only distance frame is recquired, save color frame
	//if(save_single_frame || save_dem || save_pc || save_distance)
	myDEM.setColorFrame(dst, dst2); // to opencv format. Basically set up so that first it is read to internal variable, then converted to opencv and sent over to my library


    // same for right frame
    // TODO move to function
	if(rightFrame->getFrameMode() == base::samples::frame::MODE_BAYER_RGGB ||
		rightFrame->getFrameMode() == base::samples::frame::MODE_BAYER_GRBG ||
		rightFrame->getFrameMode() == base::samples::frame::MODE_BAYER_BGGR ||
		rightFrame->getFrameMode() == base::samples::frame::MODE_BAYER_GBRG)
	{
		frame_buffer3.init( rightFrame->getWidth(), rightFrame->getHeight(), rightFrame->getDataDepth(),rock_dst.getFrameMode(),-1);
		frame_helper::FrameHelper::convertBayerToRGB24(rightFrame->getImageConstPtr(),frame_buffer3.getImagePtr(),rightFrame->getWidth(),rightFrame->getHeight(),rightFrame->frame_mode);
		left_conv.convert( rock_dst, rock_dst, 0, 0, frame_helper::INTER_LINEAR, true );	// rectification of frame
		dst = frame_helper::FrameHelper::convertToCvMat(rock_dst);
		cv::cvtColor(frame_helper::FrameHelper::convertToCvMat(frame_buffer3),dst,cv::COLOR_RGB2BGR);
    }
    else if(rightFrame->getFrameMode() == base::samples::frame::MODE_RGB) // already in RGB mode
    {
		left_conv.convert( *rightFrame, rock_dst, 0, 0, frame_helper::INTER_LINEAR, true );	// rectification of frame
		dst = frame_helper::FrameHelper::convertToCvMat(rock_dst);
		cv::cvtColor(dst, dst, cv::COLOR_RGB2BGR);
	}
	else
    {
		std::cerr << "[DEM GENERATION OROGEN] Color frame with non supported encoding, add conversion here " << rightFrame->getFrameMode() << "\n";
    }
	
	dst2 = dst;

	// unless only distance frame is recquired, save color frame
	//if(save_single_frame || save_dem || save_pc || save_distance)
	myDEM.setColorFrame(dst, dst2); // to opencv format. Basically set up so that first it is read to internal variable, then converted to opencv and sent over to my library

	// As it is now, distance shal be sent before the color frame
	if(save_distance)
	{
		myDEM.saveDistanceFrame(distance_image.data);	
		this->writeTelemetry(myDEM.getDistanceFramePath(),
			telemetry_telecommand::messages::DISTANCE,
			leftFrame->time);
	}
	
	// if frame or dem are recquired, send frame as telemetry
	if(save_single_frame || save_dem || save_distance)
	{
		this->writeTelemetry(myDEM.getImageLeftPath(),
			telemetry_telecommand::messages::IMAGE,
			leftFrame->time);
	}

    // this case is treated separately in order to set the mode to STEREO for
    // both images
	if(save_both_frames)
	{
		this->writeTelemetry(myDEM.getImageLeftPath(),
			telemetry_telecommand::messages::STEREO_LEFT,
			leftFrame->time);
		this->writeTelemetry(myDEM.getImageRightPath(),
			telemetry_telecommand::messages::STEREO_RIGHT,
			leftFrame->time);
	}

	// generate dem
	if(save_dem)
	{
		myDEM.distance2pointCloud(distance_image.data);
		myDEM.filterPointCloud();
		int error = myDEM.pointCloud2Mesh(true); // generate from filtered pointcloud
		if(!error)
		{
			this->writeTelemetry(myDEM.getMeshPath(),
				telemetry_telecommand::messages::DEM,
				leftFrame->time);
		}
	}

	// save pc and send as telemetry
	if(save_pc)
	{
		myDEM.distance2pointCloud(distance_image.data);
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
	//if(save_single_frame || save_dem || save_pc || save_distance)
		myDEM.setColorFrame(dst, dst2);
		
	// As it is now, distance shal be sent before the color frame
	// if wanted, save distance and send as telemetry
	if(save_distance)
	{
		_distance_frame.read(distance_image);
		myDEM.saveDistanceFrame(distance_image.data);	
		this->writeTelemetry(myDEM.getDistanceFramePath(),
			telemetry_telecommand::messages::DISTANCE,
			leftFrame->time);
	}
		
	// if frame or dem are recquired, send frame as telemetry
	if(save_single_frame || save_dem || save_distance)
	{
		this->writeTelemetry(myDEM.getImageLeftPath(),
			telemetry_telecommand::messages::IMAGE,
			leftFrame->time);
	}
	
	// input pointcloud
	if(save_pc || save_dem)
		myDEM.setPointCloud(input_pointcloud);

	// generate dem and send as telemetry
	if(save_dem)
	{
		myDEM.filterPointCloud();
		int error = myDEM.pointCloud2Mesh(true); // generate from filtered pointcloud
		if(!error)
		{
			this->writeTelemetry(myDEM.getMeshPath(),
				telemetry_telecommand::messages::DEM,
				leftFrame->time);
		}
	}
	
	// save pointcloud and send as telemetry
	if(save_pc)
	{
		//myDEM.filterPointCloud();
		myDEM.savePointCloud(false); // save filtered pointcloud
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
