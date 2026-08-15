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

// ─────────────────────────────────────────────────────────────────────
// Robot configuration — FR3
// ─────────────────────────────────────────────────────────────────────
static const std::string ARM_GROUP   = "fr3_arm";
static const std::string HAND_GROUP  = "fr3_hand";
static const std::string HAND_FRAME  = "fr3_hand_tcp";
static const std::string WORLD_FRAME = "world";
static const std::string OBJECT_ID   = "target_box";

// ─────────────────────────────────────────────────────────────────────
// Scene geometry — must match fr3_world.sdf exactly
// Table: center x=0.6, z=0.2, size 0.4x0.8x0.4 → top at z=0.4
// Box:   center x=0.55, z=0.425 (sits on table top: 0.4 + 0.025)
// ─────────────────────────────────────────────────────────────────────
static const double TABLE_X    = 0.6;
static const double TABLE_Z    = 0.15;
static const double TABLE_DX   = 0.4;
static const double TABLE_DY   = 0.8;
static const double TABLE_DZ   = 0.3;

static const double BOX_X      = 0.55;
static const double BOX_Z      = 0.7;
static const double BOX_SIZE   = 0.05;

// ─────────────────────────────────────────────────────────────────────
// PickPlaceNode
// ─────────────────────────────────────────────────────────────────────
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

  // ── Add collision objects to MoveIt planning scene ────────────────
  void setupPlanningScene()
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    // Table — matches fr3_world.sdf geometry exactly
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

    // Target box — sits on table top
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
    RCLCPP_INFO(LOGGER, "Planning scene: table at x=%.2f z=%.2f, box at x=%.2f z=%.2f",
      TABLE_X, TABLE_Z, BOX_X, BOX_Z);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // ── Execute the pick-and-place task ───────────────────────────────
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

    // Cast solution to SolutionSequence to iterate sub-trajectories
    const auto* solution_seq =
      dynamic_cast<const mtc::SolutionSequence*>(task_.solutions().front().get());
    if (!solution_seq) {
      RCLCPP_ERROR(LOGGER, "Solution is not a SolutionSequence");
      return;
    }

    // Use ExecuteTrajectory action — ExecuteTaskSolutionCapability
    // not available in binary Jazzy MTC install
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

      // Build filtered trajectory with arm joints only (no finger joints)
      moveit_msgs::msg::RobotTrajectory traj_msg;
      sub_traj->trajectory()->getRobotTrajectoryMsg(traj_msg);

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
        trajectory_msgs::msg::JointTrajectoryPoint filtered_point;
        filtered_point.time_from_start = point.time_from_start;
        for (size_t idx : arm_indices) {
          if (!point.positions.empty())
            filtered_point.positions.push_back(point.positions[idx]);
          if (!point.velocities.empty())
            filtered_point.velocities.push_back(point.velocities[idx]);
          if (!point.accelerations.empty())
            filtered_point.accelerations.push_back(point.accelerations[idx]);
        }
        filtered_msg.joint_trajectory.points.push_back(filtered_point);
      }

      moveit_msgs::action::ExecuteTrajectory::Goal goal;
      goal.trajectory = filtered_msg;

      RCLCPP_INFO(LOGGER, "Executing sub-trajectory %d (%zu arm joints, %zu waypoints)...",
        ++traj_count,
        filtered_msg.joint_trajectory.joint_names.size(),
        filtered_msg.joint_trajectory.points.size());

      auto future = action_client->async_send_goal(goal);
      auto goal_handle = future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(LOGGER, "Goal %d rejected", traj_count);
        return;
      }

      auto result_future = action_client->async_get_result(goal_handle);
      auto result = result_future.get();
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(LOGGER, "Sub-trajectory %d failed", traj_count);
        return;
      }
      RCLCPP_INFO(LOGGER, "Sub-trajectory %d done", traj_count);
    }
    RCLCPP_INFO(LOGGER, "Pick and place complete!");
  }

private:
  // ── Build the MTC task ────────────────────────────────────────────
  mtc::Task createTask()
  {
    mtc::Task task;
    task.stages()->setName("fr3_pick_and_place");
    task.loadRobotModel(node_);

    // Task-level properties inherited by all stages
    task.setProperty("group", ARM_GROUP);
    task.setProperty("eef", HAND_GROUP);
    task.setProperty("ik_frame", HAND_FRAME);

    // ── Solvers ───────────────────────────────────────────────────
    // OMPL — free-space arm motion
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    // JointInterpolation — simple gripper open/close
    auto interpolation_planner =
      std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    // Cartesian — straight-line end-effector motion
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.3);
    cartesian_planner->setMaxAccelerationScalingFactor(0.3);
    cartesian_planner->setStepSize(0.01);

    // ── Stage 1: Current State (Generator) ───────────────────────
    // Saves pointer — monitored by GenerateGraspPose
    mtc::Stage* current_state_ptr = nullptr;
    {
      auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_ptr = s.get();
      task.add(std::move(s));
    }

    // ── Stage 2: Open gripper ─────────────────────────────────────
    {
      auto s = std::make_unique<mtc::stages::MoveTo>(
        "open gripper", interpolation_planner);
      s->setGroup(HAND_GROUP);
      s->setGoal("open");
      task.add(std::move(s));
    }

    // ── Stage 3: Move to pick (Connector) ─────────────────────────
    // Bridges home pose to the grasp pose inside the pick container
    {
      auto s = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, sampling_planner}});
      s->setTimeout(30.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(s));
    }

    // ── Stage 4: Pick container ───────────────────────────────────
    // Approach (backward ←) | GenerateGraspPose+IK (Generator) | grasp + lift (forward →)
    mtc::Stage* attach_object_stage = nullptr;
    {
      auto pick = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(pick->properties(), {"eef", "group", "ik_frame"});
      pick->properties().configureInitFrom(mtc::Stage::PARENT,
                                           {"eef", "group", "ik_frame"});

      // Approach: straight-line along gripper Z — solved BACKWARD from grasp pose
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>(
          "approach object", cartesian_planner);
        s->properties().set("marker_ns", std::string("approach"));
        s->properties().set("link", HAND_FRAME);
        s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        s->setMinMaxDistance(0.001, 0.15);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = WORLD_FRAME;
        vec.vector.x = 0.0;
        vec.vector.y = 0.0;
        vec.vector.z = -1.0;
        s->setDirection(vec);
        pick->insert(std::move(s));
      }

      // GenerateGraspPose + ComputeIK
      // GenerateGraspPose generates candidate orientations around the object
      // ComputeIK converts each Cartesian pose to joint angles
      // setIgnoreCollisions(true) allows IK to find solutions even when
      // the hand is near the table — approach will handle actual collision avoidance
      {
        auto grasp_pose = std::make_unique<mtc::stages::GenerateGraspPose>(
          "generate grasp pose");
        grasp_pose->properties().configureInitFrom(mtc::Stage::PARENT);
        grasp_pose->properties().set("marker_ns", std::string("grasp_pose"));
        grasp_pose->setPreGraspPose("open");
        grasp_pose->setObject(OBJECT_ID);
        grasp_pose->setAngleDelta(M_PI / 4);  // 8 candidates every 45 degrees
        grasp_pose->setMonitoredStage(current_state_ptr);
        grasp_pose->setRotationAxis(Eigen::Vector3d(0, 0, 1));  // rotate around Z

        // Grasp frame: no offset — plan directly to object center
        Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
        grasp_frame_transform.translation().z() = 0.15;

        auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>(
          "grasp pose IK", std::move(grasp_pose));
        ik_wrapper->setMaxIKSolutions(2);
        ik_wrapper->setMinSolutionDistance(1.0);
        ik_wrapper->setIKFrame(grasp_frame_transform, HAND_FRAME);
        ik_wrapper->setIgnoreCollisions(true);  // ignore table collision during IK
        ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT,
                                                   {"eef", "group"});
        ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                                   {"target_pose"});
        pick->insert(std::move(ik_wrapper));
      }

      // Allow collision between hand and object so gripper can close on it
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (hand,object)");
        s->allowCollisions(OBJECT_ID,
          task.getRobotModel()
            ->getJointModelGroup(HAND_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(),
          true);
        pick->insert(std::move(s));
      }

      // Close gripper
      {
        auto s = std::make_unique<mtc::stages::MoveTo>(
          "close gripper", interpolation_planner);
        s->setGroup(HAND_GROUP);
        s->setGoal("close");
        pick->insert(std::move(s));
      }

      // Attach object — box moves with gripper from here onward
      // Save pointer so GeneratePlacePose can monitor this stage
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "attach object");
        s->attachObject(OBJECT_ID, HAND_FRAME);
        attach_object_stage = s.get();
        pick->insert(std::move(s));
      }

      // Lift: straight-line +Z in world frame — solved FORWARD after attach
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>(
          "lift object", cartesian_planner);
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

    // ── Stage 5: Move to place (Connector) ───────────────────────
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

    // ── Stage 6: Place container ──────────────────────────────────
    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
      place->properties().configureInitFrom(mtc::Stage::PARENT,
                                            {"eef", "group", "ik_frame"});

      // GeneratePlacePose + ComputeIK
      // Place 0.3m to the side of the object in the world frame
      {
        auto place_pose = std::make_unique<mtc::stages::GeneratePlacePose>(
          "generate place pose");
        place_pose->properties().configureInitFrom(mtc::Stage::PARENT);
        place_pose->properties().set("marker_ns", std::string("place_pose"));
        place_pose->setObject(OBJECT_ID);

        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = WORLD_FRAME;
        target.pose.position.x = BOX_X;
        target.pose.position.y = 0.3;   // 30cm to the side
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
        ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT,
                                                   {"eef", "group"});
        ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                                   {"target_pose"});
        place->insert(std::move(ik_wrapper));
      }

      // Open gripper to release
      {
        auto s = std::make_unique<mtc::stages::MoveTo>(
          "open gripper", interpolation_planner);
        s->setGroup(HAND_GROUP);
        s->setGoal("open");
        place->insert(std::move(s));
      }

      // Forbid collision between hand and object after release
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "forbid collision (hand,object)");
        s->allowCollisions(OBJECT_ID,
          task.getRobotModel()
            ->getJointModelGroup(HAND_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(),
          false);
        place->insert(std::move(s));
      }

      // Detach object from gripper
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "detach object");
        s->detachObject(OBJECT_ID, HAND_FRAME);
        place->insert(std::move(s));
      }

      // Retreat: move away from placed object
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>(
          "retreat", cartesian_planner);
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

    // ── Stage 7: Return home ──────────────────────────────────────
    {
      auto s = std::make_unique<mtc::stages::MoveTo>(
        "return home", sampling_planner);
      s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      s->setGoal("ready");
      task.add(std::move(s));
    }

    return task;
  }

  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
};

// ─────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────
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