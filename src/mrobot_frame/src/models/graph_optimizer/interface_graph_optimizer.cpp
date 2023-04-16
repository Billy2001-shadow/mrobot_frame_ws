#include "mrobot_frame/models/graph_optimizer/interface_graph_optimizer.hpp"

namespace mrobot_frame {
void InterfaceGraphOptimizer::SetMaxIterationsNum(int max_iterations_num) {
    max_iterations_num_ = max_iterations_num;
}
}