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
 * General motion retargeting parameters
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

class MotionRetargetingConfiguration
{
public:
  /**
   * Initialise the configuration
   */
  MotionRetargetingConfiguration(GeneralParameters& general_params,
                                     std::vector<motion_adaption::AdaptionParameters>& motion_adaption_params,
                                     tree_kinematics::KinematicsParameters kinematics_params) :
                                     general_params_(general_params),
                                     motion_adaption_params_(motion_adaption_params),
                                     kinematics_params_(kinematics_params)
//  MotionRetargetingConfiguration(MotionRetargetingParameters& motion_retargeting_params) :
//                                     general_params_(motion_retargeting_params.motion_adaption_params),
//                                     motion_adaption_params_(motion_adaption_params),
//                                     kinematics_params_(kinematics_params)
  {};
  /**
   * Clean up
   */
  ~MotionRetargetingConfiguration(){};
  /**
   * Get a constant reference to the general motion retargeting parameters
   * @return constant reference to the general motion retargeting parameters
   */
  const GeneralParameters& getGeneralParameters() const
  {
    return general_params_;
  };
  /**
   * Get a constant reference to the motion adaption parameters
   * @return constant reference to the motion adaption parameters
   */
  const std::vector<motion_adaption::AdaptionParameters>& getMotionAdaptionParameters() const
  {
    return motion_adaption_params_;
  };
  /**
   * Get a constant reference to the IK parameters
   * @return constant reference to the IK parameters
   */
  const tree_kinematics::KinematicsParameters& getIKParameters() const
  {
    return kinematics_params_;
  };

private:
  GeneralParameters general_params_;
  /**
   * Motion adaption parameters
   */
  std::vector<motion_adaption::AdaptionParameters> motion_adaption_params_;
  /**
   * IK parameters
   */
  tree_kinematics::KinematicsParameters kinematics_params_;
};

typedef boost::shared_ptr<MotionRetargetingConfiguration> MotionRetargetingConfigurationPtr;

} // namespace motion_retargeting

#endif /* MOTION_RETARGETING_CONFIGURATION_H_ */
