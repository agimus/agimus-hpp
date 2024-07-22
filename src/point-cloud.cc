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

#include <hpp/util/debug.hh>

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

    typedef ::pinocchio::GeometryObject GeometryObject;
    typedef ::pinocchio::CollisionPair CollisionPair;

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

    bool PointCloud::buildPointCloud(
      const std::string& octreeFrame, const std::string& topic,
      const std::string& sensorFrame, value_type resolution,
      const vector_t& configuration, value_type timeOut,
      bool newPointCloud)
    {
      if (!handle_)
        throw std::logic_error ("Initialize ROS first");
      octreeFrame_ = octreeFrame;
      sensorFrame_ = sensorFrame;
      newPointCloud_ = newPointCloud;
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
        ROS_ERROR_STREAM("Timeout reached while waiting for topic " << topic);
        return false;
      }
      // Express point cloud in octreeFrame frame
      movePointCloud(octreeFrame, sensorFrame, configuration);

      // build octree
      octree_ = hpp::fcl::makeOctree(pointsInLinkFrame_, resolution);
      attachOctreeToRobot(octreeFrame);
      return true;
    }

    void PointCloud::removeOctree(const std::string& octreeFrame)
    {
      std::string name(octreeFrame + std::string("/octree"));
      const DevicePtr_t& robot (problemSolver_->robot());
      // Remove octree from pinocchio model
      if (robot->geomModel().existGeometryName(name)) {
        robot->geomModel().removeGeometryObject(name);
      } else return;
      robot->createGeomData();
      // Invalidate constraint graph to force reinitialization before using
      // PathValidation instances stored in the edges.
      manipulation::graph::GraphPtr_t graph(problemSolver_->constraintGraph());
      if (graph) graph->invalidate();
      // Initialize problem to take into account new object.
      if (problemSolver_->problem())
        problemSolver_->resetProblem();
      if (display_){
        // Remove point cloud from gepetto-gui.
        undisplayOctree(octreeFrame);
      }
      octree_.reset();
    }

    void PointCloud::setDistanceBounds(value_type min, value_type max)
    {
      minDistance_ = min; maxDistance_ = max;
    }

    void PointCloud::setDisplay(bool flag)
    {
      display_ = flag;
    }

    /// Refresh octree in gepetto-viewer
    void PointCloud::refreshOctree()
    {
      if (octreeFrame_.empty()) return;
      if (display_){
        // Remove point cloud from gepetto-gui.
        undisplayOctree(octreeFrame_);
        displayOctree(octreeFrame_);
      }
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

    bool PointCloud::filterPoint(uint32_t pointcloud_id, uint32_t point_id)
    {
      // Keep point only if included in distance interval
      value_type m2(minDistance_*minDistance_);
      value_type M2(maxDistance_*maxDistance_);
      if ((m2 > pointsInSensorFrame_[pointcloud_id].row(point_id).squaredNorm())
          || (pointsInSensorFrame_[pointcloud_id].row(point_id).squaredNorm() > M2)) {
            return false;
      }
      if (filterBehindPlan_)
      {
        // Keep point only if in front of the object
        const DevicePtr_t& robot (problemSolver_->robot());
        if(!robot){
          throw std::logic_error
            ("There is no robot in the ProblemSolver instance");
        }
        Frame of(robot->getFrameByName(octreeFrame_)); // object frame
        Transform3f wMo(of.currentTransformation());
        Frame cf(robot->getFrameByName(sensorFrame_)); // camera_frame
        Transform3f wMc(cf.currentTransformation());
        vector3_t wP = wMo.actOnEigenObject(plaquePoint_); // plan point in world frame
        vector3_t cP = wMc.inverse().actOnEigenObject(wP); // plan point in camera frame
        vector3_t wNormal = wMo.rotation()  * plaqueNormalVector_; // plan normal in world frame
        vector3_t cNormal = wMc.inverse().rotation() * wNormal; // plan normal in camera frame
        vector3_t tmp(pointsInSensorFrame_[pointcloud_id].row(point_id));
        vector3_t to_point = tmp - cP;
        if (to_point.dot(cNormal)
            < objectPlanMargin_) {
          return false;
        }
      }
      return true;
    }

    void PointCloud::pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& data)
    {
      if (!waitingForData_) return;
      waitingForData_ = false;
      checkFields(data->fields);

      uint32_t pc_id = 0;
      if (newPointCloud_) {
        pointsInSensorFrame_.clear();
      }
      else {
        pc_id = (uint32_t)pointsInSensorFrame_.size();
      }
      uint32_t iPoint = 0;
      const uint8_t* ptr = &(data->data[0]);
      pointsInSensorFrame_.push_back(PointMatrix_t ());
      pointsInSensorFrame_[pc_id].resize(data->height * data->width, 3);

      for (uint32_t row=0; row < data->height; ++row) {
        for (uint32_t col=0; col < data->width; ++col) {
          pointsInSensorFrame_[pc_id](iPoint, 0) = (double)(*(const float*)
                (ptr+data->fields[0].offset));
          pointsInSensorFrame_[pc_id](iPoint, 1) = (double)(*(const float*)
                (ptr+data->fields[1].offset));
          pointsInSensorFrame_[pc_id](iPoint, 2) = (double)(*(const float*)
                (ptr+data->fields[2].offset));
          // Keep only wanted points
          if (filterPoint(pc_id, iPoint))
            ++iPoint;
          ptr+=data->point_step;
        }
      }
      ROS_INFO_STREAM("Used "  << iPoint << " out of "
                      << data->height * data->width
                      << " to build point cloud.");
      pointsInSensorFrame_[pc_id].conservativeResize(iPoint, 3);
    }

    PointCloud::PointCloud(const ProblemSolverPtr_t& ps):
      problemSolver_ (ps),
      waitingForData_(false),
      handle_(0x0), minDistance_(0), maxDistance_
      (std::numeric_limits<value_type>::infinity()), display_(true),
      filterBehindPlan_(false),
      objectPlanMargin_(0),
      newPointCloud_(false)
      {}

    void PointCloud::movePointCloud(const std::string& octreeFrame,
				    const std::string& sensorFrame,
				    const vector_t& configuration)
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
      // Move the points from the latest added pointsInSensorFrame_ element
      size_type before_size = 0;
      if (newPointCloud_) { // The existing points are replaced by the new points
        pointsInLinkFrame_.resize(pointsInSensorFrame_.back().rows(), 3);

      }
      else { // The existing points are kept and the new points are added
        before_size = pointsInLinkFrame_.rows();
        size_type new_size = before_size + pointsInSensorFrame_.back().rows();
        pointsInLinkFrame_.conservativeResize(new_size,3);
      }
      Transform3f M(pinOctreeFrame.placement*oMs);
      for (size_type r=0; r < pointsInSensorFrame_.back().rows(); ++r){
        vector3_t x(pointsInSensorFrame_.back().row(r));
        pointsInLinkFrame_.row(before_size + r) = M.actOnEigenObject(x);
      }
    }

    void PointCloud::attachOctreeToRobot(const std::string& octreeFrame)
    {
      const DevicePtr_t& robot (problemSolver_->robot());
      const Frame& of(robot->getFrameByName(octreeFrame));
      JointIndex octreeJointId(of.pinocchio().parent);
      std::string name(octreeFrame + std::string("/octree"));
      // Add a GeometryObject to the GeomtryModel
      ::pinocchio::Frame pinOctreeFrame(robot->model().frames[of.index()]);
      // Before adding octree, remove previously inserted one
      if (robot->geomModel().existGeometryName(name)) {
	      robot->geomModel().removeGeometryObject(name);
      }
      ::pinocchio::GeometryObject octreeGo
	        (name,std::numeric_limits<FrameIndex>::max(), pinOctreeFrame.parent,
	        octree_, Transform3f::Identity());
      GeomIndex octreeGeomId(robot->geomModel().addGeometryObject(octreeGo));
      // Add collision pairs with all objects not attached to the octree joint.
      for (std::size_t geomId=0; geomId <
	     robot->geomModel().geometryObjects.size(); ++geomId){
	      const GeometryObject& go(robot->geomModel().geometryObjects[geomId]);
        if(go.parentJoint != octreeJointId){
          assert(octreeGeomId < robot->geomModel().geometryObjects.size());
          assert(geomId < robot->geomModel().geometryObjects.size());
          robot->geomModel().addCollisionPair(CollisionPair(octreeGeomId,
                        geomId));
        }
      }
      robot->createGeomData();
      // Invalidate constraint graph to force reinitialization before using
      // PathValidation instances stored in the edges.
      manipulation::graph::GraphPtr_t graph(problemSolver_->constraintGraph());
      if (graph) graph->invalidate();
      // Initialize problem to take into account new object.
      if (problemSolver_->problem())
	      problemSolver_->resetProblem();
      if (display_){
        // Display point cloud in gepetto-gui.
        undisplayOctree(octreeFrame);
        displayOctree(octreeFrame);
      }
    }

#ifdef CLIENT_TO_GEPETTO_VIEWER

    bool PointCloud::displayOctree(const std::string& octreeFrame)
    {
      std::string prefix("robot/");
      // Connect to gepetto-gui without raising exception
      gepetto::viewer::corba::connect(0x0, true);
      gepetto::corbaserver::GraphicalInterface_var gui
	(gepetto::viewer::corba::gui());
      std::string nodeName(prefix + octreeFrame + std::string("/octree"));
      octree_->exportAsObjFile("/tmp/octree.obj");
      try {
	// If node already exists, remove it
	if (gui->nodeExists(nodeName.c_str())){
	  gui->deleteNode(nodeName.c_str(), true);
	  hppDout(error, "failed to load file /tmp/octree.obj in viewer");
	}
	if (!gui->addMesh(nodeName.c_str(), "/tmp/octree.obj")){
	  hppDout(error, "failed to load file /tmp/octree.obj in viewer");
	  return false;
	}
	gepetto::corbaserver::Transform pose={0,0,0,0,0,0,1};
	gui->applyConfiguration(nodeName.c_str(), pose);
      } catch (const gepetto::Error& exc) {
	throw std::runtime_error(exc.msg);
      }
      return true;
    }

    bool PointCloud::undisplayOctree(const std::string& octreeFrame)
    {
      std::string prefix("robot/");
      // Connect to gepetto-gui without raising exception
      gepetto::viewer::corba::connect(0x0, true);
      gepetto::corbaserver::GraphicalInterface_var gui
	(gepetto::viewer::corba::gui());
      std::string nodeName(prefix + octreeFrame + std::string("/octree"));
      try {
	// If node already exists, remove it
	if (gui->nodeExists(nodeName.c_str())){
	  gui->removeObjectFromCache(nodeName.c_str());
	  gui->deleteNode(nodeName.c_str(), true);
          return true;
	}
        return false;
      } catch (const gepetto::Error& exc) {
	throw std::runtime_error(exc.msg);
      }
    }

#else
    bool PointCloud::displayOctree
    (const std::string& /*octreeFrame*/)
    {
      return true;
    }
    bool PointCloud::undisplayOctree(const std::string& /*octreeFrame*/)
    {
      return true;
    }
#endif

  } // namespace agimus
} // namespace hpp
