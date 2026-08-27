#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/storage.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_node");
namespace mtc = moveit::task_constructor;

static const std::string ARM_GROUP   = "fr3_arm";
static const std::string HAND_GROUP  = "fr3_hand";
static const std::string HAND_FRAME  = "fr3_hand_tcp";
static const std::string WORLD_FRAME = "world";
static const std::string OBJECT_ID   = "target_box";

// Scene geometry — keep in sync with fr3_world.sdf
// Table: center x=0.55, z=0.425, size 0.4x0.8x0.85 → top at z=0.85
// Box:   center x=0.55, z=0.875 (on table top: 0.85 + 0.025)
static const double TABLE_X  = 0.55;
static const double TABLE_Z  = 0.4;
static const double TABLE_DX = 0.4;
static const double TABLE_DY = 0.8;
static const double TABLE_DZ = 0.8;
static const double BOX_X    = 0.55;
static const double BOX_Z    = 0.825;
static const double BOX_SIZE = 0.05;

class PickPlaceNode
{
public:
  PickPlaceNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("pick_place_node", options) }
  {}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  {
    return node_->get_node_base_interface();
  }

  void setupPlanningScene()
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = WORLD_FRAME;
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {TABLE_DX, TABLE_DY, TABLE_DZ};
    table.pose.position.x = TABLE_X;
    table.pose.position.y = 0.0;
    table.pose.position.z = TABLE_Z;
    table.pose.orientation.w = 1.0;
    table.operation = table.ADD;

    moveit_msgs::msg::CollisionObject box;
    box.id = OBJECT_ID;
    box.header.frame_id = WORLD_FRAME;
    box.primitives.resize(1);
    box.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    box.primitives[0].dimensions = {BOX_SIZE, BOX_SIZE, BOX_SIZE};
    box.pose.position.x = BOX_X;
    box.pose.position.y = 0.0;
    box.pose.position.z = BOX_Z;
    box.pose.orientation.w = 1.0;
    box.operation = box.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(table);
    objects.push_back(box);
    psi.applyCollisionObjects(objects);
    RCLCPP_INFO(LOGGER, "Planning scene: table x=%.2f z=%.2f, box x=%.2f z=%.2f",
      TABLE_X, TABLE_Z, BOX_X, BOX_Z);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  void doTask()
  {
    task_ = createTask();

    try {
      task_.init();
    } catch (mtc::InitStageException& e) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task init failed: " << e);
      return;
    }

    if (!task_.plan(10)) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      return;
    }

    RCLCPP_INFO(LOGGER, "Task planned successfully! Executing...");
    task_.introspection().publishSolution(*task_.solutions().front());

    const auto* solution_seq =
      dynamic_cast<const mtc::SolutionSequence*>(task_.solutions().front().get());
    if (!solution_seq) {
      RCLCPP_ERROR(LOGGER, "Solution is not a SolutionSequence");
      return;
    }

    auto action_client =
      rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
        node_, "/execute_trajectory");
    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(LOGGER, "ExecuteTrajectory action server not available");
      return;
    }

    int traj_count = 0;
    for (const auto* sub_sol : solution_seq->solutions()) {
      const auto* sub_traj = dynamic_cast<const mtc::SubTrajectory*>(sub_sol);
      if (!sub_traj || !sub_traj->trajectory() || sub_traj->trajectory()->empty())
        continue;

      moveit_msgs::msg::RobotTrajectory traj_msg;
      sub_traj->trajectory()->getRobotTrajectoryMsg(traj_msg);

      // Strip finger joints — no gripper controller available
      moveit_msgs::msg::RobotTrajectory filtered_msg;
      filtered_msg.joint_trajectory.header = traj_msg.joint_trajectory.header;

      std::vector<size_t> arm_indices;
      for (size_t i = 0; i < traj_msg.joint_trajectory.joint_names.size(); ++i) {
        const auto& name = traj_msg.joint_trajectory.joint_names[i];
        if (name.find("finger") == std::string::npos) {
          arm_indices.push_back(i);
          filtered_msg.joint_trajectory.joint_names.push_back(name);
        }
      }

      if (arm_indices.empty()) {
        RCLCPP_INFO(LOGGER, "Skipping gripper-only trajectory");
        continue;
      }

      for (const auto& point : traj_msg.joint_trajectory.points) {
        trajectory_msgs::msg::JointTrajectoryPoint fp;
        fp.time_from_start = point.time_from_start;
        for (size_t idx : arm_indices) {
          if (!point.positions.empty())    fp.positions.push_back(point.positions[idx]);
          if (!point.velocities.empty())   fp.velocities.push_back(point.velocities[idx]);
          if (!point.accelerations.empty())fp.accelerations.push_back(point.accelerations[idx]);
        }
        filtered_msg.joint_trajectory.points.push_back(fp);
      }

      moveit_msgs::action::ExecuteTrajectory::Goal goal;
      goal.trajectory = filtered_msg;

      RCLCPP_INFO(LOGGER, "Executing sub-trajectory %d...", ++traj_count);
      auto future = action_client->async_send_goal(goal);
      auto goal_handle = future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(LOGGER, "Goal %d rejected", traj_count);
        return;
      }

      auto result = action_client->async_get_result(goal_handle).get();
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(LOGGER, "Sub-trajectory %d failed", traj_count);
        return;
      }
      RCLCPP_INFO(LOGGER, "Sub-trajectory %d done", traj_count);
    }
    RCLCPP_INFO(LOGGER, "Pick and place complete!");
  }

private:
  mtc::Task createTask()
  {
    mtc::Task task;
    task.stages()->setName("fr3_pick_and_place");
    task.loadRobotModel(node_);

    task.setProperty("group",    ARM_GROUP);
    task.setProperty("eef",      HAND_GROUP);
    task.setProperty("ik_frame", HAND_FRAME);

    auto sampling_planner     = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner= std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner    = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.3);
    cartesian_planner->setMaxAccelerationScalingFactor(0.3);
    cartesian_planner->setStepSize(0.01);

    // Stage 1: Current State
    mtc::Stage* current_state_ptr = nullptr;
    {
      auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_ptr = s.get();
      task.add(std::move(s));
    }

    // Stage 2: Open gripper
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
      s->setGroup(HAND_GROUP);
      s->setGoal("open");
      task.add(std::move(s));
    }

    // Stage 3: Move to pick (Connector)
    {
      auto s = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, sampling_planner}});
      s->setTimeout(30.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(s));
    }

    // Stage 4: Pick container
    // Geometry: box top = 0.900, fingers 4.9cm below TCP → TCP at grasp = 0.949
    // grasp_frame_transform.z = 0.05 positions IK frame at fingertip level
    mtc::Stage* attach_object_stage = nullptr;
    {
      auto pick = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(pick->properties(), {"eef", "group", "ik_frame"});
      pick->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

      // Approach: backward-solved from grasp pose along gripper Z
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
        s->properties().set("marker_ns", std::string("approach"));
        s->properties().set("link", HAND_FRAME);
        s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        s->setMinMaxDistance(0.01, 0.15);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = HAND_FRAME;
        vec.vector.z = 1.0;
        s->setDirection(vec);
        pick->insert(std::move(s));
      }

      // GenerateGraspPose + ComputeIK
      // setIgnoreCollisions(true): IK finds valid joint configs even near table
      // The approach Cartesian path handles actual collision-free motion
      // grasp_frame_transform.z = 0.05: positions grasp point at fingertip level
      // (TCP is 4.9cm above fingertips; box top at z=0.900; TCP at grasp z=0.949)
      {
        auto grasp_pose = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        grasp_pose->properties().configureInitFrom(mtc::Stage::PARENT);
        grasp_pose->properties().set("marker_ns", std::string("grasp_pose"));
        grasp_pose->setPreGraspPose("open");
        grasp_pose->setObject(OBJECT_ID);
        grasp_pose->setAngleDelta(M_PI / 4);
        grasp_pose->setMonitoredStage(current_state_ptr);
        grasp_pose->setRotationAxis(Eigen::Vector3d(0, 0, 1));

        Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
        grasp_frame_transform.translation().z() = 0.05;

        auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>(
          "grasp pose IK", std::move(grasp_pose));
        ik_wrapper->setMaxIKSolutions(8);
        ik_wrapper->setMinSolutionDistance(1.0);
        ik_wrapper->setIKFrame(grasp_frame_transform, HAND_FRAME);
        ik_wrapper->setIgnoreCollisions(true);
        ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        pick->insert(std::move(ik_wrapper));
      }

      // Allow hand-object collision so gripper can close on box
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
        s->allowCollisions(OBJECT_ID,
          task.getRobotModel()->getJointModelGroup(HAND_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(), true);
        pick->insert(std::move(s));
      }

      // Close gripper
      {
        auto s = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
        s->setGroup(HAND_GROUP);
        s->setGoal("close");
        pick->insert(std::move(s));
      }

      // Attach object
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
        s->attachObject(OBJECT_ID, HAND_FRAME);
        attach_object_stage = s.get();
        pick->insert(std::move(s));
      }

      // Lift: +Z world frame, min 3cm (limited by table clearance)
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
        s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        s->setMinMaxDistance(0.03, 0.3);
        s->setIKFrame(HAND_FRAME);
        s->properties().set("marker_ns", std::string("lift_object"));
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = WORLD_FRAME;
        vec.vector.z = 1.0;
        s->setDirection(vec);
        pick->insert(std::move(s));
      }

      task.add(std::move(pick));
    }

    // Stage 5: Move to place (Connector)
    {
      auto s = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{
          {ARM_GROUP,  sampling_planner},
          {HAND_GROUP, interpolation_planner}});
      s->setTimeout(10.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(s));
    }

    // Stage 6: Place container
    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
      place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

      // GeneratePlacePose + ComputeIK: place 0.3m to the side
      {
        auto place_pose = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        place_pose->properties().configureInitFrom(mtc::Stage::PARENT);
        place_pose->properties().set("marker_ns", std::string("place_pose"));
        place_pose->setObject(OBJECT_ID);

        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = WORLD_FRAME;
        target.pose.position.x = BOX_X;
        target.pose.position.y = 0.3;
        target.pose.position.z = BOX_Z;
        target.pose.orientation.w = 1.0;
        place_pose->setPose(target);
        place_pose->setMonitoredStage(attach_object_stage);

        auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>(
          "place pose IK", std::move(place_pose));
        ik_wrapper->setMaxIKSolutions(4);
        ik_wrapper->setMinSolutionDistance(1.0);
        ik_wrapper->setIKFrame(HAND_FRAME);
        ik_wrapper->setIgnoreCollisions(true);
        ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        place->insert(std::move(ik_wrapper));
      }

      // Open gripper
      {
        auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
        s->setGroup(HAND_GROUP);
        s->setGoal("open");
        place->insert(std::move(s));
      }

      // Forbid hand-object collision after release
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
        s->allowCollisions(OBJECT_ID,
          task.getRobotModel()->getJointModelGroup(HAND_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(), false);
        place->insert(std::move(s));
      }

      // Detach object
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        s->detachObject(OBJECT_ID, HAND_FRAME);
        place->insert(std::move(s));
      }

      // Retreat
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
        s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        s->setMinMaxDistance(0.03, 0.3);
        s->setIKFrame(HAND_FRAME);
        s->properties().set("marker_ns", std::string("retreat"));
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = WORLD_FRAME;
        vec.vector.x = -0.3;
        s->setDirection(vec);
        place->insert(std::move(s));
      }

      task.add(std::move(place));
    }

    // Stage 7: Return home
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
      s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      s->setGoal("ready");
      task.add(std::move(s));
    }

    return task;
  }

  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<PickPlaceNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->getNodeBaseInterface());

  auto spin_thread = std::make_unique<std::thread>(
    [&executor]() { executor.spin(); });

  rclcpp::sleep_for(std::chrono::seconds(3));

  node->setupPlanningScene();
  node->doTask();

  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}