/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef MOTION_RETARGETING_CONFIGURATION_H_
#define MOTION_RETARGETING_CONFIGURATION_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <motion_adaption/types/adaption_type.h>
#include <tree_kinematics/kinematics_parameters.h>

namespace motion_retargeting
{
/**
 * Motion retargeting parameters
 */
struct MotionRetargetingParameters
{
  /**
   * General parameters
   */
  struct GeneralParameters
  {
    std::string retargeting_type_name;
    double retargeting_freq;
    double wait_for_tf; // who is using that? only motion_adaption, then move it
    std::string robot_model_name;
    bool check_self_collision;
    bool check_joint_limits;
  };
  GeneralParameters general_parameters;
  /**
   * Motion adaption parameters
   */
  std::vector<motion_adaption::MotionAdaptionParameters> motion_adaption_parameters;
  /**
   * IK parameters
   */
  tree_kinematics::KinematicsParameters kinematics_parameters;
};

class MotionRetargetingConfiguration
{
public:
  /**
   * Initialise the configuration
   */
  MotionRetargetingConfiguration(MotionRetargetingParameters& motion_retargeting_parameters)
  {
    parameters_ = motion_retargeting_parameters;
  };
  /**
   * Clean up
   */
  ~MotionRetargetingConfiguration(){};
  /**
   * Get a read-only reference to the configuration parameters
   * @return configuration_ constant reference to the configuration parameters
   */
  const MotionRetargetingParameters& getParameters() const
  {
    return parameters_;
  }

private:
  MotionRetargetingParameters parameters_;
};

typedef boost::shared_ptr<MotionRetargetingConfiguration> MotionRetargetingConfigurationPtr;

} // namespace motion_retargeting

#endif /* MOTION_RETARGETING_CONFIGURATION_H_ */
