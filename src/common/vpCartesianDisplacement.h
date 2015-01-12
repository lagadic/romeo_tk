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
  vpCartesianDisplacement() : m_init_done(false) {}
  ~vpCartesianDisplacement() {}
  bool computeVelocity(const vpNaoqiRobot &robot, const vpColVector &cart_delta_pos,
                       double delta_t, const std::string &chain_name, const vpMatrix &oVe)
  {
    if (! m_init_done) {
      // compute the time
      vpHomogeneousMatrix M;
      vpRxyzVector rxyz;

      for (unsigned int i=0; i< 3; i++) {
        M[i][3] = cart_delta_pos[i];
        rxyz[i] = cart_delta_pos[i+3];
      }
      vpRotationMatrix R(rxyz);
      M.insert(R);

      m_v_o = vpExponentialMap::inverse(M, delta_t);

      m_t_initial = vpTime::measureTimeSecond();
      m_init_done = true;
      m_delta_t = delta_t;
      m_chain_name = chain_name;
      m_joint_names = robot.getBodyNames(chain_name);
      if (chain_name == "LArm" || chain_name == "RArm")
        m_joint_names.pop_back(); // Delete last joints LHand or RHand, that we don't consider in the servo
      m_q_dot.resize(m_joint_names.size());
    }

    if (vpTime::measureTimeSecond() < m_t_initial + m_delta_t) {
      vpMatrix oJo = oVe * robot.get_eJe(m_chain_name);

      m_q_dot = oJo.pseudoInverse() * m_v_o;

      return true;
    }
    else {
      m_q_dot = 0;
      return false;
    }
  }

  vpColVector getJointVelocity() const {return m_q_dot;}
  std::vector<std::string> getJointNames() const {return m_joint_names;}
};

#endif
