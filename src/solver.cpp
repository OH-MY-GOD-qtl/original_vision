#include "solver.hpp"

namespace vision
{

Solver::Solver(const std::string &) {} // TODO: 读内参、畸变、手眼标定

void Solver::set_R_gimbal2world(const Eigen::Quaterniond &q) { R_gimbal2world_ = q; }

void Solver::solve(Armor &)
{
    // TODO: solvePnP(IPPE) → camera → gimbal → world；成功才置 armor.solved = true
}

} // namespace vision
