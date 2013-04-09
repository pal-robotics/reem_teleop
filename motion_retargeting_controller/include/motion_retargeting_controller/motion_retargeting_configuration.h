#include <string>
#include <vector>

#include <motion_adaption/types/adaption_type.h>

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
    double wait_for_tf;
    double epsilon;
    double max_iterations;
    double q_dot_max_factor;
    double q_dot_min_factor;
    double low_pass_factor;
    double x_dot_trans_min;
    double x_dot_trans_max;
    double x_dot_rot_min;
    double x_dot_rot_max;
    double lambda;
    std::string robot_model_name;
    bool check_self_collision;
    bool check_joint_limits;
  };
  /**
   * Motion adaption parameters
   */
  std::vector<motion_adaption::MotionAdaptionParameters> motion_adaption_parameters;
  /**
   * IK parameters
   */
  struct IkParameters
  {
    std::vector< std::vector<double> > task_space_weights;
    std::vector<double> joint_space_weights;
  };
  std::vector<IkParameters> ik_parameters;
};

class MotionRetargetingConfiguration
{
public:
  /**
   * Initialise the configuration
   */
  MotionRetargetingConfiguration(MotionRetargetingParameters motion_retargeting_parameters);
  /**
   * Clean up
   */
  ~MotionRetargetingConfiguration();
  /**
   * Get a read-only reference to the configuration parameters
   * @return configuration_ constant reference to the configuration parameters
   */
  MotionRetargetingParameters getParameters() // const
  {
    return parameters_;
  }

private:
  MotionRetargetingParameters parameters_;
};

} // namespace motion_retargeting
