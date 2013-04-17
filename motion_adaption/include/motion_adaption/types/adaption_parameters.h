/**
 * @file /motion_adaption/include/motion_adaption/types/adaption_parameters.h
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 17, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ADAPTION_PARAMETERS_H_
#define ADAPTION_PARAMETERS_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include<string>
#include<vector>

namespace motion_adaption
{

struct OrientationCorrection
{
  double roll;
  double pitch;
  double yaw;
};

struct GeneralParameters
{
  std::string adaption_name;
  enum AdaptionTypes
  {
    NoAdaption,
    TransRotAdaption,
    HandsAdaption,
    UpperBodyAdaption
  };
  AdaptionTypes adaption_type;
  double wait_for_tf;
  std::string input_ref_frame;
  std::string input_ref_dep_parent;
  std::string input_ref_dep_child;
  OrientationCorrection input_ref_correction;
  std::string target_ref_frame;
};

struct TransRotParameters
{
  std::string input_endpt;
  std::string target_endpt;
  std::string goal_endpt;
};

struct HandsParameters
{
  std::string user_torso_name;
  std::string user_r_shoulder_name;
  std::string user_r_elbow_name;
  std::string user_r_hand_name;
  std::string user_l_shoulder_name;
  std::string user_l_elbow_name;
  std::string user_l_hand_name;
  std::string robot_r_shoulder_name;
  std::string robot_r_elbow_name;
  std::string robot_r_hand_name;
  std::string robot_l_shoulder_name;
  std::string robot_l_elbow_name;
  std::string robot_l_hand_name;
  std::string goal_torso_name;
  std::string goal_r_shoulder_name;
  std::string goal_r_elbow_name;
  std::string goal_r_hand_name;
  std::string goal_l_shoulder_name;
  std::string goal_l_elbow_name;
  std::string goal_l_hand_name;
};

class AdaptionParameters
{
public:
  AdaptionParameters(const GeneralParameters& general_params) : general_params_(general_params){};
  ~AdaptionParameters(){};
  const GeneralParameters& getGeneralParameters() const
  {
    return general_params_;
  }
private:
  GeneralParameters general_params_;
};

class TransRotAdaptionParameters : public AdaptionParameters
{
public:
  TransRotAdaptionParameters(const GeneralParameters& general_params,
                                const TransRotParameters& trans_rot_params) :
                                AdaptionParameters(general_params),
                                trans_rot_params_(trans_rot_params)
  {};
  ~TransRotAdaptionParameters(){};
  const TransRotParameters& getTransRotParameters() const
  {
    return trans_rot_params_;
  }
private:
  TransRotParameters trans_rot_params_;
};

class HandsAdaptionParameters : public AdaptionParameters
{
public:
  HandsAdaptionParameters(const GeneralParameters& general_params,
                             const HandsParameters& hands_params) :
                             AdaptionParameters(general_params),
                             hands_params_(hands_params)
  {};
  ~HandsAdaptionParameters(){};
private:
  HandsParameters hands_params_;
};

} // namespace motion_adaption

#endif /* ADAPTION_PARAMETERS_H_ */
