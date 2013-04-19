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

struct OrientationAdjustments
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
  std::string input_ref_name;
  std::string input_pos_ref_name;
  OrientationAdjustments input_ref_orient_adjust;
  std::string target_ref_name;
  double wait_for_tf;
};

struct TransRotParameters
{
  std::string input_endpt_name;
  std::string target_endpt_name;
  std::string goal_endpt_name;
  OrientationAdjustments goal_orient_adjust;
};

struct HandsParameters
{
  std::string input_torso_name;
  std::string input_r_shoulder_name;
  std::string input_r_elbow_name;
  std::string input_r_hand_name;
  std::string input_l_shoulder_name;
  std::string input_l_elbow_name;
  std::string input_l_hand_name;
  std::string target_torso_name;
  std::string target_r_shoulder_name;
  std::string target_r_elbow_name;
  std::string target_r_hand_name;
  std::string target_l_shoulder_name;
  std::string target_l_elbow_name;
  std::string target_l_hand_name;
  std::string goal_r_hand_name;
  OrientationAdjustments goal_r_hand_orient_adjust;
  std::string goal_l_hand_name;
  OrientationAdjustments goal_l_hand_orient_adjust;
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
  const TransRotParameters& getAdaptionSpecificParameters() const
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
  const HandsParameters& getAdaptionSpecificParameters() const
  {
    return hands_params_;
  }
  ~HandsAdaptionParameters(){};
private:
  HandsParameters hands_params_;
};

} // namespace motion_adaption

#endif /* ADAPTION_PARAMETERS_H_ */
