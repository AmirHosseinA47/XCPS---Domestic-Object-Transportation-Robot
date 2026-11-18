// Runs the kitchen behaviour tree against the Gazebo simulation and explains what happened.
//
//   ros2 run kitchen_bt kitchen_bt_runner --scenario set     (or: clean)
//        [--tree FILE] [--log-dir DIR] [--groot]
//
// Every action node calls one robot skill on /kitchen_skills/execute (kitchen_skills.py) and
// records the skill's answer: what was done and why, or why it failed. The run produces
//   <log-dir>/<scenario>_<time>.jsonl            every node status change and skill answer
//   <log-dir>/<scenario>_<time>.btlog            for Groot2's log replay
//   <log-dir>/<scenario>_<time>_explanation.txt  the summary printed at the end
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/contrib/json.hpp>
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <kitchen_interfaces/srv/skill.hpp>
#include <rclcpp/rclcpp.hpp>

using json = nlohmann::json;
using Skill = kitchen_interfaces::srv::Skill;
using Clock = std::chrono::steady_clock;

// ------------------------------------------------------------------------------------------
// Explanation log

struct SkillStep {
  double time_s;
  std::string path, node, skill, message;
  bool success;
};

class ExplanationLog {
 public:
  explicit ExplanationLog(const std::filesystem::path& jsonl) : out_(jsonl), start_(Clock::now()) {}

  double elapsed() const {
    return std::chrono::duration<double>(Clock::now() - start_).count();
  }

  void write(const json& entry) {
    json e = entry;
    e["t"] = std::round(elapsed() * 10) / 10;
    out_ << e.dump() << "\n";
    out_.flush();
  }

  void skill(const std::string& path, const std::string& node, const std::string& skill,
             const json& params, bool success, const std::string& message) {
    steps_.push_back({elapsed(), path, node, skill, message, success});
    write({{"event", "skill"}, {"node", node}, {"path", path}, {"skill", skill},
           {"params", params}, {"success", success}, {"message", message}});
    std::cout << std::fixed << std::setprecision(0) << "[" << std::setw(4) << elapsed() << " s] "
              << (success ? "ok    " : "FAILED") << " " << node << ": " << message << std::endl;
  }

  const std::vector<SkillStep>& steps() const { return steps_; }

 private:
  std::ofstream out_;
  Clock::time_point start_;
  std::vector<SkillStep> steps_;
};

// Records every status change of every node (JSONL) and the final status of each subtree.
class TransitionLogger : public BT::StatusChangeLogger {
 public:
  TransitionLogger(const BT::Tree& tree, std::shared_ptr<ExplanationLog> log)
      : BT::StatusChangeLogger(tree.rootNode()), log_(std::move(log)) {}

  void callback(BT::Duration, const BT::TreeNode& node, BT::NodeStatus prev,
                BT::NodeStatus status) override {
    if (status == BT::NodeStatus::IDLE) {
      return;
    }
    log_->write({{"event", "status"}, {"node", node.name()}, {"path", node.fullPath()},
                 {"type", node.registrationName()}, {"from", BT::toStr(prev)},
                 {"to", BT::toStr(status)}});
    if (node.registrationName() == "SubTree") {
      subtrees_[node.name()] = status;
    }
  }
  void flush() override {}

  const std::map<std::string, BT::NodeStatus>& subtrees() const { return subtrees_; }

 private:
  std::shared_ptr<ExplanationLog> log_;
  std::map<std::string, BT::NodeStatus> subtrees_;
};

// ------------------------------------------------------------------------------------------
// Action nodes: each calls one skill

struct Context {
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<Skill>::SharedPtr client;
  std::shared_ptr<ExplanationLog> log;
};

std::vector<double> parseNumbers(const std::string& text) {
  std::vector<double> values;
  std::stringstream ss(text);
  std::string part;
  while (std::getline(ss, part, ';')) {
    values.push_back(std::stod(part));
  }
  return values;
}

class SkillAction : public BT::StatefulActionNode {
 public:
  SkillAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Context> ctx,
              std::string skill)
      : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)), skill_(std::move(skill)) {}

 protected:
  virtual json params() { return json::object(); }
  virtual void onResult(const json&) {}

  template <typename T>
  T input(const std::string& port) {
    auto value = getInput<T>(port);
    if (!value) {
      throw BT::RuntimeError("missing input [", port, "]: ", value.error());
    }
    return value.value();
  }

  BT::NodeStatus onStart() override {
    try {
      params_ = params();
    } catch (const std::exception& e) {
      ctx_->log->skill(fullPath(), name(), skill_, {}, false, std::string("bad input: ") + e.what());
      return BT::NodeStatus::FAILURE;
    }
    if (!ctx_->client->service_is_ready()) {
      ctx_->log->skill(fullPath(), name(), skill_, params_, false,
                       "the robot skills service is not running (kitchen_skills.py)");
      return BT::NodeStatus::FAILURE;
    }
    auto request = std::make_shared<Skill::Request>();
    request->skill = skill_;
    request->params_json = params_.dump();
    future_ = ctx_->client->async_send_request(request).share();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    auto response = future_.get();
    ctx_->log->skill(fullPath(), name(), skill_, params_, response->success, response->message);
    if (!response->success) {
      return BT::NodeStatus::FAILURE;
    }
    onResult(json::parse(response->result_json.empty() ? "{}" : response->result_json, nullptr, false));
    return BT::NodeStatus::SUCCESS;
  }

  // A running skill cannot be interrupted; the tree only halts nodes that are not running here.
  void onHalted() override {}

 private:
  std::shared_ptr<Context> ctx_;
  std::string skill_;
  json params_;
  rclcpp::Client<Skill>::SharedFuture future_;
};

class MoveTorso : public SkillAction {
 public:
  MoveTorso(const std::string& n, const BT::NodeConfig& c, std::shared_ptr<Context> ctx)
      : SkillAction(n, c, ctx, "move_torso") {}
  static BT::PortsList providedPorts() { return {BT::InputPort<double>("height", "metres, 0-0.33")}; }
  json params() override { return {{"height", input<double>("height")}}; }
};

class ParkArms : public SkillAction {
 public:
  ParkArms(const std::string& n, const BT::NodeConfig& c, std::shared_ptr<Context> ctx)
      : SkillAction(n, c, ctx, "park_arms") {}
  static BT::PortsList providedPorts() { return {}; }
};

class NavigateTo : public SkillAction {
 public:
  NavigateTo(const std::string& n, const BT::NodeConfig& c, std::shared_ptr<Context> ctx)
      : SkillAction(n, c, ctx, "navigate") {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("place", "standing pose name from scene.yaml")};
  }
  json params() override { return {{"to", input<std::string>("place")}}; }
};

class LookAt : public SkillAction {
 public:
  LookAt(const std::string& n, const BT::NodeConfig& c, std::shared_ptr<Context> ctx)
      : SkillAction(n, c, ctx, "look_at") {}
  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("target", "x;y;z (map)")}; }
  json params() override {
    auto p = parseNumbers(input<std::string>("target"));
    if (p.size() != 3) {
      throw BT::RuntimeError("target needs x;y;z");
    }
    return {{"x", p[0]}, {"y", p[1]}, {"z", p[2]}};
  }
};

class DetectObject : public SkillAction {
 public:
  DetectObject(const std::string& n, const BT::NodeConfig& c, std::shared_ptr<Context> ctx)
      : SkillAction(n, c, ctx, "detect") {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("object"), BT::OutputPort<std::string>("position", "x;y;z")};
  }
  json params() override { return {{"object", input<std::string>("object")}}; }
  void onResult(const json& r) override {
    if (r.contains("x")) {
      std::ostringstream s;
      s << std::fixed << std::setprecision(3) << r["x"].get<double>() << ";" << r["y"].get<double>()
        << ";" << r["z"].get<double>();
      setOutput("position", s.str());
    }
  }
};

// Re-reads where the robot is before looking again. In the lab, the head was aimed with a stale
// robot pose, which made detection fail now and then; here the next LookAt uses this fresh pose.
class RefreshRobotPose : public SkillAction {
 public:
  RefreshRobotPose(const std::string& n, const BT::NodeConfig& c, std::shared_ptr<Context> ctx)
      : SkillAction(n, c, ctx, "state") {}
  static BT::PortsList providedPorts() { return {}; }
};

class PickObject : public SkillAction {
 public:
  PickObject(const std::string& n, const BT::NodeConfig& c, std::shared_ptr<Context> ctx)
      : SkillAction(n, c, ctx, "pick") {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("object"), BT::OutputPort<std::string>("arm", "l or r")};
  }
  json params() override { return {{"object", input<std::string>("object")}}; }
  void onResult(const json& r) override {
    if (r.contains("arm")) {
      setOutput("arm", r["arm"].get<std::string>());
    }
  }
};

class PlaceObject : public SkillAction {
 public:
  PlaceObject(const std::string& n, const BT::NodeConfig& c, std::shared_ptr<Context> ctx)
      : SkillAction(n, c, ctx, "place") {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("object"), BT::InputPort<std::string>("goal", "x;y (map)"),
            BT::InputPort<std::string>("surface", "counter or island")};
  }
  json params() override {
    auto g = parseNumbers(input<std::string>("goal"));
    if (g.size() < 2) {
      throw BT::RuntimeError("goal needs x;y");
    }
    return {{"object", input<std::string>("object")}, {"x", g[0]}, {"y", g[1]},
            {"surface", input<std::string>("surface")}};
  }
};

class CheckObjectAt : public SkillAction {
 public:
  CheckObjectAt(const std::string& n, const BT::NodeConfig& c, std::shared_ptr<Context> ctx)
      : SkillAction(n, c, ctx, "object_pose") {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("object"), BT::InputPort<std::string>("goal", "x;y (map)"),
            BT::InputPort<double>("tolerance", 0.05, "metres")};
  }
  json params() override {
    auto g = parseNumbers(input<std::string>("goal"));
    if (g.size() < 2) {
      throw BT::RuntimeError("goal needs x;y");
    }
    return {{"object", input<std::string>("object")}, {"x", g[0]}, {"y", g[1]},
            {"tolerance", input<double>("tolerance")}};
  }
};

// ------------------------------------------------------------------------------------------
// The explanation printed at the end

// The innermost subtree a node belongs to, from its path ("transport_milk::7/detect_object::12").
std::string subtreeOf(const std::string& path, const std::map<std::string, BT::NodeStatus>& subtrees) {
  std::stringstream ss(path);
  std::string segment, found = "main tree";
  while (std::getline(ss, segment, '/')) {
    auto name = segment.substr(0, segment.find("::"));
    if (subtrees.count(name)) {
      found = name;
    }
  }
  return found;
}

std::string explain(const std::string& scenario, BT::NodeStatus result, const ExplanationLog& log,
                    const std::map<std::string, BT::NodeStatus>& subtrees) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(0);
  out << "What did the robot do? (" << (scenario == "set" ? "set the table" : "clean the table") << ")\n";
  std::string current;
  for (const auto& s : log.steps()) {
    if (!s.success) {
      continue;
    }
    auto part = subtreeOf(s.path, subtrees);
    if (part != current) {
      out << "  " << part << ":\n";
      current = part;
    }
    out << "    [" << std::setw(4) << s.time_s << " s] " << s.message << "\n";
  }
  out << "\nDid anything go wrong, and why?\n";
  bool any = false;
  for (const auto& s : log.steps()) {
    if (s.success) {
      continue;
    }
    any = true;
    auto part = subtreeOf(s.path, subtrees);
    auto it = subtrees.find(part);
    bool recovered = it != subtrees.end() && it->second == BT::NodeStatus::SUCCESS;
    out << "  - " << part << " / " << s.node << " at " << s.time_s << " s: " << s.message << "\n"
        << "    -> " << (recovered ? "recovered: the tree's fallback tried again and the step succeeded"
                                   : "not recovered: the task stopped here") << "\n";
  }
  if (!any) {
    out << "  Nothing went wrong.\n";
  }
  out << "\nResult: " << (result == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE") << " after "
      << log.elapsed() << " s\n";
  for (const auto& s : log.steps()) {
    if (s.skill == "object_pose") {
      out << "  " << (s.success ? "ok  " : "FAIL") << " " << s.message << "\n";
    }
  }
  return out.str();
}

// ------------------------------------------------------------------------------------------

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  std::string scenario = "set";
  std::string tree_file = ament_index_cpp::get_package_share_directory("kitchen_bt") +
                          "/behavior_trees/kitchen_tasks.xml";
  std::filesystem::path log_dir = std::filesystem::path(std::getenv("HOME")) / "kitchen_bt_runs";
  bool groot = false;
  for (size_t i = 1; i < args.size(); ++i) {
    if (args[i] == "--scenario" && i + 1 < args.size()) {
      scenario = args[++i];
    } else if (args[i] == "--tree" && i + 1 < args.size()) {
      tree_file = args[++i];
    } else if (args[i] == "--log-dir" && i + 1 < args.size()) {
      log_dir = args[++i];
    } else if (args[i] == "--groot") {
      groot = true;
    } else {
      std::cerr << "usage: kitchen_bt_runner [--scenario set|clean] [--tree FILE] [--log-dir DIR] [--groot]\n";
      return 2;
    }
  }
  if (scenario != "set" && scenario != "clean") {
    std::cerr << "--scenario must be set or clean\n";
    return 2;
  }

  auto node = rclcpp::Node::make_shared("kitchen_bt");
  auto ctx = std::make_shared<Context>();
  ctx->node = node;
  ctx->client = node->create_client<Skill>("/kitchen_skills/execute");
  std::cout << "waiting for the robot skills (/kitchen_skills/execute) ..." << std::endl;
  if (!ctx->client->wait_for_service(std::chrono::seconds(120))) {
    std::cerr << "kitchen_skills.py is not running: ros2 launch kitchen_gazebo kitchen_tasks.launch.py\n";
    return 1;
  }

  std::filesystem::create_directories(log_dir);
  char stamp[32];
  auto now = std::time(nullptr);
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
  auto stem = log_dir / (scenario + "_" + stamp);
  ctx->log = std::make_shared<ExplanationLog>(stem.string() + ".jsonl");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<MoveTorso>("MoveTorso", ctx);
  factory.registerNodeType<ParkArms>("ParkArms", ctx);
  factory.registerNodeType<NavigateTo>("NavigateTo", ctx);
  factory.registerNodeType<LookAt>("LookAt", ctx);
  factory.registerNodeType<DetectObject>("DetectObject", ctx);
  factory.registerNodeType<RefreshRobotPose>("RefreshRobotPose", ctx);
  factory.registerNodeType<PickObject>("PickObject", ctx);
  factory.registerNodeType<PlaceObject>("PlaceObject", ctx);
  factory.registerNodeType<CheckObjectAt>("CheckObjectAt", ctx);
  factory.registerBehaviorTreeFromFile(tree_file);
  auto tree = factory.createTree(scenario == "set" ? "SetTable" : "CleanTable");

  BT::FileLogger2 file_logger(tree, stem.string() + ".btlog");
  TransitionLogger transitions(tree, ctx->log);
  std::unique_ptr<BT::Groot2Publisher> groot_publisher;
  if (groot) {
    groot_publisher = std::make_unique<BT::Groot2Publisher>(tree, 1667);
    std::cout << "Groot2: connect to localhost:1667" << std::endl;
  }
  ctx->log->write({{"event", "start"}, {"scenario", scenario}, {"tree", tree_file}});

  auto status = tree.tickOnce();
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);
    tree.sleep(std::chrono::milliseconds(20));
    status = tree.tickOnce();
  }

  auto text = explain(scenario, status, *ctx->log, transitions.subtrees());
  ctx->log->write({{"event", "end"}, {"result", BT::toStr(status)}});
  std::ofstream(stem.string() + "_explanation.txt") << text;
  std::cout << "\n" << text << "\nlogs: " << stem.string() << ".{jsonl,btlog} and _explanation.txt" << std::endl;
  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
