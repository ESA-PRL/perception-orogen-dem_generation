/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef DEM_GENERATION_TASK_TASK_HPP
#define DEM_GENERATION_TASK_TASK_HPP

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <base/samples/frame.h>
#include "dem_generation/TaskBase.hpp"
#include <frame_helper/FrameHelper.h>

#include <pcl/point_cloud.h>

#include <dem_generation/dem_generation.hpp>

#include <velodyne_lidar/pointcloudConvertHelper.hpp>
#include <velodyne_lidar/MultilevelLaserScan.h>

#include <telemetry_telecommand/Messages.hpp>

#include <unistd.h>


namespace dem_generation {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the dem_generation namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','dem_generation::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
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
		bool save_frame, save_distance, save_dem, save_pc; // save bools



    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "dem_generation::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
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

