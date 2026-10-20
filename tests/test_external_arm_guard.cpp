#include "external_arm_guard.hpp"
#include <cassert>
#include <limits>
int main() {
 std::vector<double> limits(46);
 for (size_t i=0;i<23;++i) { limits[2*i]=-1.;limits[2*i+1]=1.; }
 std::vector<double> q(10,0.);
 assert(valid_external_arm_target(q,limits));
 assert(!valid_external_arm_target(std::vector<double>(9,0.),limits));
 assert(!valid_external_arm_target(std::vector<double>(23,0.),limits));
 q[0]=std::numeric_limits<double>::quiet_NaN();assert(!valid_external_arm_target(q,limits));
 q[0]=2.;assert(!valid_external_arm_target(q,limits));
 assert(std::abs(limit_external_arm_step(0.f,1.f,.002f)-.002f)<1e-7);
}
