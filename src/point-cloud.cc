// Copyright 2021, 2022, CNRS, Airbus SAS

// Author: Florent Lamiraux

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include <boost/format.hpp>
#include <hpp/agimus/point-cloud.hh>

#include <ros/node_handle.h>

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <hpp/fcl/octree.h>

#include <hpp/pinocchio/joint.hh>

#include <hpp/core/problem.hh>

#include <hpp/manipulation/problem-solver.hh>
#include <hpp/manipulation/graph/graph.hh>

#ifdef CLIENT_TO_GEPETTO_VIEWER
#include <gepetto/viewer/corba/client.hh>
#endif

namespace hpp {
  namespace agimus {

#ifdef CLIENT_TO_GEPETTO_VIEWER
    static void toGepettoTransform (const Transform3f& in,
				    gepetto::corbaserver::Transform out)
    {
      Transform3f::Quaternion q (in.rotation());
      out[3] = (float)q.x();
      out[4] = (float)q.y();
      out[5] = (float)q.z();
      out[6] = (float)q.w();
      for(int i=0; i<3; i++)
        out[i] = (float)in.translation() [i];
    }
#endif

    PointCloud::~PointCloud()
    {
      shutdownRos();
    }

    bool PointCloud::initializeRosNode (const std::string& name, bool anonymous)
    {
      if (!ros::isInitialized()) {
        // Call ros init
        int option = ros::init_options::NoSigintHandler |
	  (anonymous ? ros::init_options::AnonymousName : 0);
        int argc = 0;
        ros::init (argc, NULL, name, option);
      }
      bool ret = false;
      if (!handle_) {
        handle_ = new ros::NodeHandle();
        ret = true;
      }
      return ret;
    }

    void PointCloud::shutdownRos ()
    {
      if (!handle_) return;
      boost::mutex::scoped_lock lock(mutex_);
      if (handle_) delete handle_;
      handle_ = NULL;
    }

    bool PointCloud::getPointCloud
    (const std::string& octreeFrame, const std::string& topic,
     const std::string& sensorFrame, value_type resolution,
     const vector_t& configuration, value_type timeOut)
    {
      if (!handle_)
        throw std::logic_error ("Initialize ROS first");
      // create subscriber to topic
      waitingForData_ = false;
      ros::Subscriber subscriber = handle_->subscribe
	(topic, 1, &PointCloud::pointCloudCb, this);

      waitingForData_ = true;
      value_type begin = ros::Time::now().toSec();
      while (waitingForData_) {
	value_type now = ros::Time::now().toSec();
	if (now - begin >= timeOut) {
	  break;
	}
	ros::spinOnce();
	ros::Duration(1e-2).sleep();
      }
      if (waitingForData_) {
	// timeout reached
	waitingForData_ = false;
	return false;
      }
      // build octree
      hpp::fcl::OcTreePtr_t octree(hpp::fcl::makeOctree(points_, resolution));
      attachOctreeToRobot(octree, octreeFrame, sensorFrame, configuration);
      return true;
    }

    void checkFields(const std::vector<sensor_msgs::PointField>& fields)
    {
      // Check that number of fields is at least 3
      if (fields.size() < 3){
	std::ostringstream os;
	os << "Wrong number of fields. Expected at least 3, got "
	   << fields.size() << ".";
	throw std::invalid_argument(os.str());
      }
      // Check that first 3 fields correspond to x, y, and z
      if (fields[0].name != "x"){
	std::ostringstream os;
	os << "Wrong field. Expected \"x\", got \""
	   << fields[0].name << "\".";
	throw std::invalid_argument(os.str());
      }
      if (fields[1].name != "y"){
	std::ostringstream os;
	os << "Wrong field. Expected \"y\", got \""
	   << fields[1].name << "\".";
	throw std::invalid_argument(os.str());
      }
      if (fields[2].name != "z"){
	std::ostringstream os;
	os << "Wrong field. Expected \"z\", got \""
	   << fields[2].name << "\".";
	throw std::invalid_argument(os.str());
      }
    }

    void PointCloud::pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& data)
    {
      if (!waitingForData_) return;
      waitingForData_ = false;
      checkFields(data->fields);
      uint32_t iPoint = 0;
      const uint8_t* ptr = &(data->data[0]);
      points_.resize(data->height * data->width, 3);
      for (uint32_t row=0; row < data->height; ++row) {
	for (uint32_t col=0; col < data->width; ++col) {
	  points_(iPoint, 0) = (double)(*(const float*)
					(ptr+data->fields[0].offset));
	  points_(iPoint, 1) = (double)(*(const float*)
					(ptr+data->fields[1].offset));
	  points_(iPoint, 2) = (double)(*(const float*)
					(ptr+data->fields[2].offset));
	  ++iPoint;
	  ptr+=data->point_step;
	}
      }
    }

    PointCloud::PointCloud(const ProblemSolverPtr_t& ps):
      problemSolver_ (ps),
      waitingForData_(false),
      handle_(0x0)
      {}

    void PointCloud::attachOctreeToRobot
    (const OcTreePtr_t& octree, const std::string& octreeFrame,
     const std::string& sensorFrame, const vector_t& configuration)
    {
      // Compute forward kinematics for input configuration
      const DevicePtr_t& robot (problemSolver_->robot());
      if(!robot){
	throw std::logic_error
	  ("There is no robot in the ProblemSolver instance");
      }
      robot->currentConfiguration(configuration);
      robot->computeFramesForwardKinematics();
      Frame sf(robot->getFrameByName(sensorFrame));
      Frame of(robot->getFrameByName(octreeFrame));
      // Compute pose of sensor in joint frame
      Transform3f wMs(sf.currentTransformation());
      Transform3f wMo(of.currentTransformation());
      Transform3f oMs(wMo.inverse() * wMs);
      std::string name(octreeFrame + std::string("/octree"));
      // Add a GeometryObject to the GeomtryModel
      ::pinocchio::Frame pinOctreeFrame(robot->model().frames[of.index()]);
      ::pinocchio::GeometryObject go
	  (name,std::numeric_limits<FrameIndex>::max(), pinOctreeFrame.parent,
	   octree, pinOctreeFrame.placement*oMs);
      robot->geomModel().addGeometryObject(go);
      // Invalidate constraint graph to force reinitialization before using
      // PathValidation instances stored in the edges.
      manipulation::graph::GraphPtr_t graph(problemSolver_->constraintGraph());
      if (graph) graph->invalidate();
      // Initialize problem to take into account new object.
      if (problemSolver_->problem())
	problemSolver_->resetProblem();
      // Display point cloud in gepetto-gui.
      displayOctree(octree, octreeFrame, oMs);
    }
#ifdef CLIENT_TO_GEPETTO_VIEWER
    bool PointCloud::displayOctree
    (const OcTreePtr_t& octree, const std::string& octreeFrame,
     const Transform3f& Moctree)
    {
      std::string prefix("robot/");
      // Connect to gepetto-gui without raising exception
      gepetto::viewer::corba::connect(0x0, true);
      gepetto::corbaserver::GraphicalInterface_var gui
	(gepetto::viewer::corba::gui());
      std::string groupName(prefix + octreeFrame + std::string("/octree"));
      gui->createGroup(groupName.c_str());
      //gui->addToGroup(nodeName.c_str(), octreeFrame.c_str());
      gepetto::corbaserver::Transform pose;
      toGepettoTransform(Moctree, pose);
      gui->applyConfiguration(groupName.c_str(), pose);
      std::vector<boost::array<value_type, 6> > boxes(octree->toBoxes());
      std::size_t boxId = 0;
      pose[3] = pose[4] = pose[5] = 0; pose[6]=1;
	for (auto box : boxes){
	value_type x(box[0]);
	value_type y(box[1]);
	value_type z(box[2]);
	value_type size(box[3]);
	std::string name((boost::format("box_%1%") % boxId).str());
	float white[4]={1.,1.,1.,1.};
	gui->addBox(name.c_str(), (float)size, (float)size, (float)size, white);
	gui->addToGroup(name.c_str(), groupName.c_str());
	pose[0] = (float)x; pose[1] = (float)y; pose[2] = (float)z;
	gui->applyConfiguration(name.c_str(), pose);
	boxId++;
      }
      return true;
    }
#else
    bool PointCloud::displayOctree
    (const OcTreePtr_t& /*octree*/, const std::string& /*octreeFrame*/,
     const Transform3f& /*Moctree*/)
    {
      return true;
    }
#endif

  } // namespace agimus
} // namespace hpp