/**
 * @file /motion_adaption/include/motion_adaption/types/trans_rot_adaption.h
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

#ifndef TRANS_ROT_ADAPTION_H_
#define TRANS_ROT_ADAPTION_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "adaption_type.h"

namespace motion_adaption
{

class TransRotAdaption : public AdaptionType
{
public:
  TransRotAdaption():AdaptionType(){};
  ~TransRotAdaption(){};
  void adapt()
  {
    std::cout << "Performing TransRotAdaption ..." << std::endl;
  };
private:
};

} // namespace motion_adaption

#endif /* TRANS_ROT_ADAPTION_H_ */
