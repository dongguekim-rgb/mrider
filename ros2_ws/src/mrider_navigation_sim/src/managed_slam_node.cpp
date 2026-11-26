// Keep Humble SLAM's map/TF workers alive only while its libraries are alive.
// Mapping itself is the installed, unmodified AsynchronousSlamToolbox.
#include <memory>
#include <algorithm>
#include <stdexcept>
#include <sys/resource.h>
#include <rclcpp/rclcpp.hpp>
#include <slam_toolbox/slam_toolbox_async.hpp>

class ManagedSlam : public slam_toolbox::AsynchronousSlamToolbox
{
public:
  ManagedSlam() : AsynchronousSlamToolbox(rclcpp::NodeOptions()) {}

  void join_workers()
  {
    // configure() starts these workers. Helper shared_ptrs can keep the
    // node alive after main's pointer is released, skipping its destructor
    // before process-global Karto/library teardown. Join explicitly first.
    for (auto & worker : threads_) {
      if (worker && worker->joinable()) {
        worker->join();
      }
    }
    threads_.clear();  // A later base destructor must not join them again.
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManagedSlam>();
  const auto requested = node->declare_parameter<int64_t>("stack_size_to_use", 40000000);
  struct rlimit limit;
  if (requested <= 0 || getrlimit(RLIMIT_STACK, &limit) != 0) {
    throw std::runtime_error("Cannot read SLAM stack limit");
  }
  if (limit.rlim_cur < static_cast<rlim_t>(requested)) {
    limit.rlim_cur = std::min(static_cast<rlim_t>(requested), limit.rlim_max);
    if (setrlimit(RLIMIT_STACK, &limit) != 0) {
      throw std::runtime_error("Cannot set SLAM stack limit");
    }
  }
  node->configure();
  node->loadPoseGraphByParams();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  node->join_workers();
  return 0;
}
