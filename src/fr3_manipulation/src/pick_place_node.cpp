#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
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
// Robot configuration — FR3 specific
// ─────────────────────────────────────────────────────────────────────
static const std::string ARM_GROUP   = "fr3_arm";
static const std::string HAND_GROUP  = "fr3_hand";
static const std::string HAND_FRAME  = "fr3_hand_tcp";
static const std::string WORLD_FRAME = "world";
static const std::string OBJECT_ID   = "target_box";

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
  // Box + table must exist in MoveIt scene (not just Gazebo)
  // so MTC can reason about them during planning
  void setupPlanningScene()
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    // Table — must match Gazebo table in fr3_world.sdf
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = WORLD_FRAME;
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {0.4, 0.8, 0.7};
    table.pose.position.x = 0.85;
    table.pose.position.y = 0.0;
    table.pose.position.z = 0.35;
    table.pose.orientation.w = 1.0;
    table.operation = table.ADD;

    // Target box
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "add target box");
      moveit_msgs::msg::CollisionObject box;
      box.id = OBJECT_ID;
      box.header.frame_id = WORLD_FRAME;
      box.primitives.resize(1);
      box.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
      box.primitives[0].dimensions = {0.05, 0.05, 0.05};
      box.pose.position.x = 0.75;
      box.pose.position.y = 0.0;
      box.pose.position.z = 0.875;
      box.pose.orientation.w = 1.0;
      box.operation = box.ADD;
    

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(table);
    objects.push_back(box);
    psi.applyCollisionObjects(objects);
    RCLCPP_INFO(LOGGER, "Planning scene updated with table and box");
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

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
      return;
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

    // Task-level properties — inherited by all stages
    task.setProperty("group", ARM_GROUP);
    task.setProperty("eef", HAND_GROUP);
    task.setProperty("ik_frame", HAND_FRAME);

    // ── Solvers ───────────────────────────────────────────────────
    // OMPL — free-space motion (arm)
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);

    // Joint interpolation — simple gripper open/close motions
    auto interpolation_planner =
      std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    // Cartesian — straight-line end-effector motion (approach/lift/retreat)
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.3);
    cartesian_planner->setMaxAccelerationScalingFactor(0.3);
    cartesian_planner->setStepSize(0.01);

    // ── Stage 1: Current State (Generator) ───────────────────────
    // Saves pointer — GenerateGraspPose monitors it for start config
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
    // Bridges current state to the grasp pose generated inside pick container
    {
      auto s = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, sampling_planner}});
      s->setTimeout(10.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(s));
    }

    // ── Stage 4: Pick container (SerialContainer) ─────────────────
    // Contains: approach → GenerateGraspPose+IK → allow collision →
    //           close gripper → attach → lift
    // The GenerateGraspPose is a Generator — it anchors the backward
    // solving so approach is computed right-to-left (← direction)
    mtc::Stage* attach_object_stage = nullptr;  // monitored by place pose generator
    {
      auto pick = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(pick->properties(), {"eef", "group", "ik_frame"});
      pick->properties().configureInitFrom(mtc::Stage::PARENT,
                                           {"eef", "group", "ik_frame"});

      // Approach: straight-line Z in hand frame — solved BACKWARD from grasp pose
      // The direction vector.z=1 means move in +Z of gripper frame
      // Solved backward: MTC computes what pose leads to a valid grasp
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>(
          "approach object", cartesian_planner);
        s->properties().set("marker_ns", std::string("approach"));
        s->properties().set("link", HAND_FRAME);
        s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        s->setMinMaxDistance(0.1, 0.15);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = HAND_FRAME;
        vec.vector.z = 1.0;  // approach direction in gripper frame
        s->setDirection(vec);
        pick->insert(std::move(s));
      }

      // GenerateGraspPose + ComputeIK
      // GenerateGraspPose generates candidate grasp orientations around the object
      // ComputeIK wraps it to solve IK for each candidate — this is the Generator
      // that anchors the entire pick container's bidirectional solve
      {
        auto grasp_pose = std::make_unique<mtc::stages::GenerateGraspPose>(
          "generate grasp pose");
        grasp_pose->properties().configureInitFrom(mtc::Stage::PARENT);
        grasp_pose->properties().set("marker_ns", std::string("grasp_pose"));
        grasp_pose->setPreGraspPose("open");
        grasp_pose->setObject(OBJECT_ID);
        grasp_pose->setAngleDelta(M_PI / 12);  // try every 15 degrees
        grasp_pose->setMonitoredStage(current_state_ptr);

        // IK frame transform — rotate gripper to approach from above
        Eigen::Isometry3d grasp_frame_transform;
        Eigen::Quaterniond q =
          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
        grasp_frame_transform.linear() = q.matrix();
        grasp_frame_transform.translation().z() = 0.1;

        // ComputeIK: wraps GenerateGraspPose and solves IK for each candidate
        auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>(
          "grasp pose IK", std::move(grasp_pose));
        ik_wrapper->setMaxIKSolutions(8);
        ik_wrapper->setMinSolutionDistance(1.0);
        ik_wrapper->setIKFrame(grasp_frame_transform, HAND_FRAME);
        ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT,
                                                   {"eef", "group"});
        ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                                   {"target_pose"});
        pick->insert(std::move(ik_wrapper));
      }

      // Allow collision between hand and object so gripper can close on it
      // Without this, MTC rejects all grasp solutions as colliding
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

      // Attach object — from here MTC treats box as part of robot
      // Save pointer so place pose generator can monitor this stage
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
        s->setMinMaxDistance(0.1, 0.3);
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
    // Bridges post-lift state to place pose
    // Uses both arm (OMPL) and hand (interpolation) planners
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

    // ── Stage 6: Place container (SerialContainer) ────────────────
    // Contains: GeneratePlacePose+IK → open gripper → forbid collision →
    //           detach → retreat
    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
      place->properties().configureInitFrom(mtc::Stage::PARENT,
                                            {"eef", "group", "ik_frame"});

      // GeneratePlacePose + ComputeIK
      // Place target: 0.5m to the side of the object in the object frame
      // Monitors attach_object_stage to know how object is held
      {
        auto place_pose = std::make_unique<mtc::stages::GeneratePlacePose>(
          "generate place pose");
        place_pose->properties().configureInitFrom(mtc::Stage::PARENT);
        place_pose->properties().set("marker_ns", std::string("place_pose"));
        place_pose->setObject(OBJECT_ID);

        // Target pose relative to the object frame
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = WORLD_FRAME;
        target.pose.position.x = 0.55;
        target.pose.position.y = 0.3;
        target.pose.position.z = 0.875;
        target.pose.orientation.w = 1.0;
        place_pose->setPose(target);
        place_pose->setMonitoredStage(attach_object_stage);

        auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>(
          "place pose IK", std::move(place_pose));
        ik_wrapper->setMaxIKSolutions(4);
        ik_wrapper->setMinSolutionDistance(1.0);
        ik_wrapper->setIKFrame(HAND_FRAME);  // use hand frame, not object frame
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

      // Retreat: move in -X direction away from placed object
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>(
          "retreat", cartesian_planner);
        s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        s->setMinMaxDistance(0.1, 0.3);
        s->setIKFrame(HAND_FRAME);
        s->properties().set("marker_ns", std::string("retreat"));
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = WORLD_FRAME;
        vec.vector.x = -0.5;
        s->setDirection(vec);
        place->insert(std::move(s));
      }

      task.add(std::move(place));
    }

    // ── Stage 7: Return to ready pose ────────────────────────────
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

  // MultiThreadedExecutor required for MTC — needs concurrent callback processing
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>(
    [&executor, &node]() {
      executor.add_node(node->getNodeBaseInterface());
      executor.spin();
      executor.remove_node(node->getNodeBaseInterface());
    });

  // Give move_group time to fully initialise
  rclcpp::sleep_for(std::chrono::seconds(3));

  node->setupPlanningScene();
  node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}