#include <alvision/alvisiondefinitions.h>
#include <visp/vpImageConvert.h>

#include <vpPepperFollowPeople.h>

vpPepperFollowPeople::vpPepperFollowPeople(std::string ip, int port, vpNaoqiRobot &robot) :m_proxy(ip, port), m_mem_proxy(ip, port),
  m_people_proxy(ip, port), m_led_proxy(ip, port),m_face_tracker(ip, port),m_tts_proxy(ip, port), m_robot(NULL),
  m_image_height(240), m_image_width(320)

{
  m_robot = &robot;
  m_asr_proxy = new  AL::ALSpeechRecognitionProxy(ip, port);
  m_vocabulary.push_back("follow me");
  m_vocabulary.push_back("stop");

  initialization();

}


vpPepperFollowPeople::vpPepperFollowPeople(std::string ip, int port, vpNaoqiRobot &robot, AL::ALSpeechRecognitionProxy *asr_proxy, std::vector<std::string> vocabulary ) :m_proxy(ip, port), m_mem_proxy(ip, port),
  m_people_proxy(ip, port), m_led_proxy(ip, port),m_face_tracker(ip, port),m_tts_proxy(ip, port), m_robot(NULL),
  m_image_height(240), m_image_width(320)

{
  m_robot = &robot;
  m_asr_proxy = asr_proxy;

  m_vocabulary.push_back("follow me");
  m_vocabulary.push_back("stop");
  m_vocabulary.insert( m_vocabulary.end(), vocabulary.begin(), vocabulary.end() );

  std::cout << "Vocabulary"<< std::endl;
  for(unsigned int i = 0;i < m_vocabulary.size();i++)
    std::cout << m_vocabulary[i] << std::endl;

  initialization();

}


void vpPepperFollowPeople::initialization()
{
  // Get the camera parameters
  std::string camera_name = "CameraTopPepper";
  m_cam = vpNaoqiGrabber::getIntrinsicCameraParameters(AL::kQVGA, camera_name, vpCameraParameters::perspectiveProjWithDistortion);
  m_eMc = vpNaoqiGrabber::getExtrinsicCameraParameters(camera_name, vpCameraParameters::perspectiveProjWithDistortion);

  std::cout << "eMc:" << std::endl << m_eMc << std::endl;
  std::cout << "cam:" << std::endl << m_cam << std::endl;

  m_jointNames_head = m_robot->getBodyNames("Head");

  // Open Proxy for the speech
  m_tts_proxy.setLanguage("English");
  m_phrase = " Hi";

  // Inizialize PeoplePerception
  m_people_proxy.subscribe("People", 30, 0.0);
  std::cout << "PeoplePerception started" << std::endl;

  // Open Proxy for the recognition speech
  m_asr_proxy->pause(true);
  m_asr_proxy->setVisualExpression(false);
  m_asr_proxy->setLanguage("English");

  // Set the vocabulary
  m_asr_proxy->setVocabulary(m_vocabulary,false);

  // Start the speech recognition engine with user Test_m_asr_proxy
  m_asr_proxy->pause(false);
  m_asr_proxy->subscribe("Test_ASR");
  std::cout << "Speech recognition engine started" << std::endl;


  //Set bool
  m_stop_vxy = false;
  m_move_base = true;
  m_move_base_prev = true;
  m_servo_time_init = false;
  m_reinit_servo = true;
  m_person_found = false;
  m_reverse = true;

  // Set Visual Servoing:
  m_task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
  m_task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
  m_lambda_base.initStandard(1.2, 1.0, 10); // 2.3, 0.7, 15
  m_lambda_nobase.initStandard(5, 2.9, 15); // 4, 0.5, 15
  m_task.setLambda(m_lambda_base) ;


  // Create the desired visual feature
  // m_head_cog_cur.set_uv(m_image_height/2, m_image_width/2);
  m_ip.set_uv(m_image_height/2, m_image_width/2);
  // Create the current x visual feature
  m_ip.set_uv(m_image_height/2,m_image_width/2);

  vpFeatureBuilder::create(m_s, m_cam, m_ip);
  vpFeatureBuilder::create(m_sd, m_cam, m_ip);
  // Add the feature
  m_task.addFeature(m_s, m_sd);


  // Create the depth feature
  m_Zd = 1.2;
  m_Z = m_Zd;
  m_s_Z.buildFrom(m_s.get_x(), m_s.get_y(), m_Z , 0.); // log(Z/Z*) = 0 that's why the last parameter is 0
  m_s_Zd.buildFrom(m_sd.get_x(), m_sd.get_y(), m_Zd , 0.); // log(Z/Z*) = 0 that's why the last parameter is 0
  // Add the feature
  m_task.addFeature(m_s_Z, m_s_Zd);

  // Initialization Jacobian
  tJe.resize(6,5,true);
  tJe[0][0]= 1;
  tJe[1][1]= 1;
  tJe[5][2]= 1;
  eJe.resize(6,5,true);

  m_limit_yaw = m_robot->getProxy()->getLimits("HeadYaw");

  m_t_prev = vpTime::measureTimeSecond();

  m_robot->getProxy()->setExternalCollisionProtectionEnabled("Move", false);

}


vpPepperFollowPeople::~vpPepperFollowPeople()
{
  m_tts_proxy.setLanguage("French");
  m_people_proxy.unsubscribe("People");

  m_asr_proxy->unsubscribe("Test_ASR");
  m_asr_proxy->setVisualExpression(true);
  m_asr_proxy->setLanguage("French");

  m_robot->getProxy()->setExternalCollisionProtectionEnabled("Move", true);
  // m_robot = NULL;
}


bool vpPepperFollowPeople::computeAndApplyServo()
{
  double t = vpTime::measureTimeMs();

  if (m_reinit_servo) {
    m_servo_time_init = vpTime::measureTimeSecond();
    m_t_prev = vpTime::measureTimeSecond();
    m_reinit_servo = false;
    m_led_proxy.post.fadeRGB("FaceLeds","white",0.1);
    std::cout << "REINIT SERVO ms" << std::endl;

  }

  //std::cout << "Loop time check_speech 0: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

  m_stop_vxy = false;

  AL::ALValue result_speech = m_mem_proxy.getData("WordRecognized");
  // std::cout << "Loop time check_speech 1: " << vpTime::measureTimeMs() - t << " ms" << std::endl;


  if ( ((result_speech[0]) == m_vocabulary[0]) && (double (result_speech[1]) > 0.4 )) //move
  {
    std::cout << "Recognized: " << result_speech[0] << "with confidence of " << result_speech[1] << std::endl;
    m_task.setLambda(m_lambda_base) ;

    m_move_base = true;
  }
  else if ( (result_speech[0] == m_vocabulary[1]) && (double(result_speech[1]) > 0.4 )) //stop
  {
    std::cout << "Recognized: " << result_speech[0] << "with confidence of " << result_speech[1] << std::endl;
    m_task.setLambda(m_lambda_nobase) ;
    m_move_base = false;
  }
  //std::cout << "Loop time check_speech 2: " << vpTime::measureTimeMs() - t << " ms" << std::endl;


  if (m_move_base != m_move_base_prev)
  {
    if (m_move_base)
      m_tts_proxy.post.say("Ok, I will follow you.");
    else
      m_tts_proxy.post.say("Ok, I will stop.");
  }

  m_move_base_prev = m_move_base;

  // std::cout << "Loop time check_speech: " << vpTime::measureTimeMs() - t << " ms" << std::endl;


  // Detect Face from Okao
  bool face_found = m_face_tracker.detect();

  // std::cout << "Loop time face_tracker: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
  if (face_found) {
    m_led_proxy.post.fadeRGB("FaceLeds","blue",0.1);

    double u = m_face_tracker.getCog(0).get_u();
    double v = m_face_tracker.getCog(0).get_v();
    if (u<= m_image_width && v <= m_image_height)
      m_head_cog_cur.set_uv(u,v);
  }


  // Detect Person from Depth Camera
  AL::ALValue result = m_mem_proxy.getData("PeoplePerception/VisiblePeopleList");

  // std::cout << "Loop time get Data PeoplePerception " << vpTime::measureTimeMs() - t << " ms" << std::endl;

  m_person_found = false;
  if (result.getSize() > 0)
  {
    AL::ALValue info = m_mem_proxy.getData("PeoplePerception/PeopleDetected");
    int num_people = info[1].getSize();
    std::ostringstream text;
    text << "Found " << num_people << " person(s)";

    m_person_found = true;

    if (face_found) // Try to find the match between two detection
    {
      vpImagePoint cog_face;
      double dist_min = 1000;
      unsigned int index_person = 0;

      for (unsigned int i = 0; i < num_people; i++)
      {
        float alpha =  info[1][i][2];
        float beta =  info[1][i][3];
        //Z = Zd;
        // Centre of face into the image
        float x =  m_image_width/2 -  m_image_width * beta;
        float y =  m_image_height/2  + m_image_height * alpha;
        cog_face.set_uv(x,y);
        double dist = vpImagePoint::distance(cog_face, m_head_cog_cur);

        if (dist < dist_min)
        {
          dist_min = dist;
          //best_cog_face_peoplep = cog_face;
          index_person  = i;
        }
      }

      if (dist_min < 55.)
        m_Z = info[1][index_person][1]; // Current distance

    }
    else // Take the first one on the list and use cog face from PeoplePerception
    {
      float alpha =  info[1][0][2];
      float beta =  info[1][0][3];
      //Z = Zd;
      // Centre of face into the image
      float x =  m_image_width/2 -  m_image_width * beta;
      float y =  m_image_height/2  + m_image_height * alpha;
      m_head_cog_cur.set_uv(x,y);
      m_Z = info[1][0][1]; // Current distance
    }
  }
  else
  {
    std::cout << "No distance computed " << std::endl;
    m_stop_vxy = true;
    m_robot->getProxy()->setAngles("HipRoll",0.0,1.0);
  }  //  m_robot->stop(m_jointNames_head);
  //  m_robot->stopBase();
  //  std::cout << "Stop request!" << std::endl;
  //  m_reinit_servo = true;

  //std::cout << "Loop time before VS: " << vpTime::measureTimeMs() - t << " ms" << std::endl;


  if (face_found || m_person_found )
  {
    // Get Head Jacobian (6x2)
    vpMatrix torso_eJe_head;
    m_robot->get_eJe("Head",torso_eJe_head);

    // Add column relative to the base rotation (Wz)
    for (unsigned int i = 0; i < 6; i++)
      for (unsigned int j = 0; j < torso_eJe_head.getCols(); j++)
        tJe[i][j+3] = torso_eJe_head[i][j];

    // std::cout << "tJe" << std::endl << tJe << std::endl;

    vpHomogeneousMatrix torsoMHeadPith( m_robot->getProxy()->getTransform("HeadPitch", 0, true));// get transformation  matrix between torso and HeadRoll

    vpVelocityTwistMatrix HeadPitchVLtorso(torsoMHeadPith.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        HeadPitchVLtorso[i][j+3] = 0;

    //std::cout << "HeadPitchVLtorso: " << std::endl << HeadPitchVLtorso << std::endl;
    // Transform the matrix
    eJe = HeadPitchVLtorso *tJe;

    // std::cout << "eJe" << std::endl << eJe << std::endl;

    m_task.set_eJe( eJe );
    m_task.set_cVe( vpVelocityTwistMatrix(m_eMc.inverse()) );

    //    vpDisplay::displayCross(I, head_cog_des, 10, vpColor::blue);
    //    vpDisplay::displayCross(I, head_cog_cur, 10, vpColor::green);
    //  std::cout << "head_cog_des:" << std::endl << head_cog_des << std::endl;
    //  std::cout << "head_cog_cur:" << std::endl << head_cog_cur << std::endl;

    if (m_reverse == false && (m_Z - m_Zd) < 0.0 )
      m_Z = m_Zd;

    double x,y;
    vpPixelMeterConversion::convertPoint(m_cam, m_head_cog_cur, x, y);
    m_s.buildFrom(x, y, m_Z);
    //s.set_xyZ(head_cog_cur.get_u(), head_cog_cur.get_v(), Z);

    // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
    m_s_Z.buildFrom(m_s.get_x(), m_s.get_y(), m_Z, log(m_Z/m_Zd)) ;

    m_q_dot = m_task.computeControlLaw(vpTime::measureTimeSecond() - m_servo_time_init);

    //std::cout << "Loop time compute VS: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

    vpMatrix P = m_task.getI_WpW();
    double alpha = -3.3;
    double min = m_limit_yaw[0][0];
    double max = m_limit_yaw[0][1];

    vpColVector z_q2 (m_q_dot.size());
    vpColVector q_yaw = m_robot->getPosition(m_jointNames_head[0]);

    z_q2[3] = 2 * alpha * q_yaw[0]/ pow((max - min),2);

    vpColVector q3 = P * z_q2;
    m_q_dot =  m_q_dot + q3;

    std::vector<float> vel(m_jointNames_head.size());
    vel[0] = m_q_dot[3];
    vel[1] = m_q_dot[4];

    m_robot->setVelocity(m_jointNames_head,vel);

    std::cout << "q:" << m_q_dot << std::endl;

    if (std::fabs(m_Z - m_Zd) < 0.05 || m_stop_vxy || !m_move_base)
    {
      std::cout << "#################################################!" << std::endl;
      std::cout << std::fabs(m_Z - m_Zd) << std::endl;
      std::cout << m_stop_vxy << std::endl;
      std::cout << !m_move_base << std::endl;
      std::cout << "#################################################!" << std::endl;
      m_robot->setBaseVelocity(0.0, 0.0, m_q_dot[2]);
    }
    else
      m_robot->setBaseVelocity(m_q_dot[0], m_q_dot[1], m_q_dot[2]);

  }
  else {
    m_robot->stop(m_jointNames_head);
    m_robot->stopBase();
    std::cout << "Stop******************************!" << std::endl;
    m_reinit_servo = true;
  }

  std::cout << "Loop time internal: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

  return true;
}

void  vpPepperFollowPeople::setDesiredDistance(double dist)
{
  m_Zd = dist;
  m_s_Zd.buildFrom(m_sd.get_x(), m_sd.get_y(), m_Zd , 0.); // log(Z/Z*) = 0 that's why the last parameter is 0
}

void vpPepperFollowPeople::setReverse(bool flag)
{
  m_reverse = flag;
}

double vpPepperFollowPeople::getActualDistance() const
{
  return m_Z;
}

void vpPepperFollowPeople::stop()
{
  m_robot->stop(m_jointNames_head);
  m_robot->stopBase();
  std::cout << "Stop request!" << std::endl;
  m_reinit_servo = true;
}

void vpPepperFollowPeople::stopTranslationBase()
{
  m_move_base = false;
}

void vpPepperFollowPeople::activateTranslationBase()
{
  m_move_base = true;
}


void vpPepperFollowPeople::exit()
{
  m_move_base = true;
}




