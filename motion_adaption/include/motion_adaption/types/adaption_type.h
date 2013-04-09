/**
 * @file /motion_adaption/include/motion_adaption/types/adaption_type.h
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 9, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ADAPTION_TYPE_H_
#define ADAPTION_TYPE_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <boost/shared_ptr.hpp>

namespace motion_adaption
{

struct MotionAdaptionParameters
{
  enum MotionAdaptionType
  {
    NoAdaption,
    TransRotAdaption
  };
  MotionAdaptionType adaption_type;
  std::string input_ref_frame;
  std::vector<std::string> input_endpt;
  std::string output_ref_frame;
  std::vector<std::string> output_endpt;
  struct InputCorrection
  {
    double pitch;
    double roll;
    double yaw;
  };
  std::vector<InputCorrection> input_correction;
};

class AdaptionType
{
public:
//  struct AvailableTypes
//  {

//  };
  AdaptionType(){};
  virtual ~AdaptionType(){};
  virtual void adapt() = 0;
private:
};

typedef boost::shared_ptr<AdaptionType> AdaptionTypePtr;

} // namespace motion_adaption

#endif /* ADAPTION_TYPE_H_ */
