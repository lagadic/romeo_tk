#ifndef __vpCartesianDisplacement_h__
#define __vpCartesianDisplacement_h__

#include <visp/vpExponentialMap.h>
#include <visp_naoqi/vpNaoqiRobot.h>

class vpCartesianDisplacement
{
protected:
  bool m_init_done;
  vpColVector m_v_o;
  double m_t_initial;
  double m_delta_t;
  std::string m_chain_name;
  std::vector<std::string> m_joint_names;
  vpColVector m_q_dot;

public:
  vpCartesianDisplacement();
  ~vpCartesianDisplacement();
  bool computeVelocity(const vpNaoqiRobot &robot, const vpColVector &cart_delta_pos,
                       double delta_t, const std::string &chain_name, const vpMatrix &oVe);

  vpColVector getJointVelocity() const {return m_q_dot;}
  std::vector<std::string> getJointNames() const {return m_joint_names;}
};

#endif
