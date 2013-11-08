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
#include<iostream>

namespace motion_adaption
{

struct OrientationAdjustments
{
  double roll;
  double pitch;
  double yaw;
};

struct TransRotAdaptionParameters;
struct HandAdaptionParameters;
struct HandsAdaptionParameters;

/**
 * TODO: I'm not proud nor happy of the current style of parameter handling. Let's replace it with something
 *       smarter in the future.
 *       In the meanwhile try to replace normal pointers with boost::shared_ptr - maybe not needed, check the current
 *       use of references and pointers
 */
struct AdaptionParameters
{
  AdaptionParameters(){};
  AdaptionParameters(const AdaptionParameters& adapt_params) :
    adaption_name(adapt_params.adaption_name),
    adaption_type(adapt_params.adaption_type),
    input_ref_name(adapt_params.input_ref_name),
    input_pos_ref_name(adapt_params.input_pos_ref_name),
    input_ref_orient_adjust(adapt_params.input_ref_orient_adjust),
    target_ref_name(adapt_params.target_ref_name),
    wait_for_tf(adapt_params.wait_for_tf)
  {};

  virtual ~AdaptionParameters(){};

  virtual TransRotAdaptionParameters* getAsTransRotAdaptionParams()
  {
    return 0;
  }
  virtual HandAdaptionParameters* getAsHandAdaptionParams()
  {
    return 0;
  }
  virtual HandsAdaptionParameters* getAsHandsAdaptionParams()
  {
    return 0;
  }

  std::string adaption_name;
  enum AdaptionTypes
  {
    NoAdaption,
    TransRotAdaption,
    HandAdaption,
    HandsAdaption,
  };
  AdaptionTypes adaption_type;
  std::string input_ref_name;
  std::string input_pos_ref_name;
  OrientationAdjustments input_ref_orient_adjust;
  std::string target_ref_name;
  double wait_for_tf;
};

struct TransRotAdaptionParameters : public AdaptionParameters
{
  TransRotAdaptionParameters(){};
  TransRotAdaptionParameters(AdaptionParameters adapt_params) : AdaptionParameters(adapt_params){};

  ~TransRotAdaptionParameters(){};

  virtual TransRotAdaptionParameters* getAsTransRotAdaptionParams()
  {
    return static_cast<TransRotAdaptionParameters*>(this);
  }

  std::string input_endpt_name;
  std::string target_endpt_name;
  std::string goal_endpt_name;
  OrientationAdjustments goal_orient_adjust;
};

struct HandAdaptionParameters : public AdaptionParameters
{
  HandAdaptionParameters(){};
  HandAdaptionParameters(AdaptionParameters adapt_params) : AdaptionParameters(adapt_params){};

  ~HandAdaptionParameters(){};

  virtual HandAdaptionParameters* getAsHandAdaptionParams()
  {
    return static_cast<HandAdaptionParameters*>(this);
  }

  std::string input_torso_name;
  std::string input_neck_name;
  std::string input_shoulder_name;
  std::string input_elbow_name;
  std::string input_hand_name;
  std::string target_torso_name;
  std::string target_neck_name;
  std::string target_shoulder_name;
  std::string target_elbow_name;
  std::string target_hand_name;
  std::string goal_hand_name;
  OrientationAdjustments goal_hand_orient_adjust;
};

struct HandsAdaptionParameters : public AdaptionParameters
{
  HandsAdaptionParameters(){};
  HandsAdaptionParameters(AdaptionParameters adapt_params) : AdaptionParameters(adapt_params){};

  ~HandsAdaptionParameters(){};

  virtual HandsAdaptionParameters* getAsHandsAdaptionParams()
  {
    return static_cast<HandsAdaptionParameters*>(this);
  }

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

} // namespace motion_adaption

#endif /* ADAPTION_PARAMETERS_H_ */
