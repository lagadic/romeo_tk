#ifndef __vpJointLimitAvoidance_h__
#define __vpJointLimitAvoidance_h__


double sigmoidFunction(const vpColVector & e)
{
  double e0 = 0.1;
  double e1 = 0.7;
  double sig = 0.0;

  double norm_e = e.euclideanNorm() ;

  // std::cout << "norm_e: " << std::endl << norm_e << std::endl;

  if (norm_e > e1)
    sig = 1.0;
  else if (e0 <= norm_e && norm_e <= e1 )
    sig = 1.0 / (1.0 + exp(-12.0 * ( (norm_e-e0)/((e1-e0))) + 6.0 ) );
  else
    sig = 0.0;

  return sig;
}

vpMatrix computeP(const vpColVector & e, const vpMatrix & J, const vpMatrix & J_pinv,const int & n)
{

  vpMatrix P(n,n);

  vpMatrix I(n,n);
  I.eye();


  vpMatrix P_e(n,n);
  P_e =  I - J_pinv * J; // vpMatrix 	I_WpW in Visp vpServo

  double pp = (e.t() * J * J.transpose() * e);

  vpMatrix  ee_t(n,n);
  ee_t =  e * e.t();


  vpMatrix P_norm_e(n,n);
  P_norm_e = I - (1.0 / pp ) * J.transpose() * ee_t * J;

  P = sigmoidFunction(e) * P_norm_e + (1 - sigmoidFunction(e)) * P_e;

  //
  // std::cout << "SigmoidFunction: " << std::endl << sigmoidFunction(e) << std::endl;

  //std::cout << "P: " << std::endl << P << std::endl;
  //std::cout << "Pe: " << std::endl << P_e << std::endl;
  //std::cout << "P_norm_e: " << std::endl << P_norm_e << std::endl;
  //std::cout << "J: " << std::endl << J << std::endl;
  //std::cout << "J_pinv: " << std::endl << J_pinv << std::endl;


  return P;

}



vpColVector computeQsec(const vpMatrix &P, const vpColVector &jointMin, const vpColVector &jointMax,  const vpColVector & q , const vpColVector & q1,  const double & ro,const double & ro1, vpColVector & q_l0_min, vpColVector & q_l0_max, vpColVector &q_l1_min, vpColVector &q_l1_max, bool use_custom_lim )
{


  double lambda = 0.7;
  double lambda_l = 0.0;

  int n = q.size();
  vpColVector q2 (n);


  // Computation of gi ([nx1] vector) and lambda_l ([nx1] vector)
  vpMatrix g(n,n);
  vpColVector q2_i(n);


  for(unsigned int i = 0; i < n; i++)
  {

    if (!use_custom_lim)
    {
      double qmin = jointMin[i];
      double qmax = jointMax[i];

      q_l0_min[i] = qmin + ro *(qmax - qmin);
      q_l0_max[i] = qmax - ro *(qmax - qmin);

      q_l1_min[i] =  q_l0_min[i] - ro * ro1 * (qmax - qmin);
      q_l1_max[i] =  q_l0_max[i] + ro * ro1 * (qmax - qmin);

    }

    if (q[i] < q_l0_min[i] )
      g[i][i] = -1;
    else if (q[i] > q_l0_max[i] )
      g[i][i] = 1;
    else
      g[i][i]= 0;

  }

  // std::cout << "------------" << std::endl;
  // std::cout << "g: " << std::endl << g << std::endl;
  // std::cout << "------------" << std::endl;

  //  std::cout << "***************************" << std::endl;
  for(unsigned int i = 0; i < n; i++)
  {
    // std::cout << "ITERATION : " << std::endl << i << std::endl;

    if (q[i] > q_l0_min[i] && q[i] < q_l0_max[i])
    {
      //   std::cout << "---- caso 4 : zero "  << std::endl;
      q2_i = 0 * q2_i;
    }

    else
    {
      vpColVector Pg_i(n);
      Pg_i = (P * g.getCol(i));
      double b = ( vpMath::abs(q1[i]) )/( vpMath::abs( Pg_i[i] ) );

      if (b < 1)
      {

        if (q[i] < q_l1_min[i] || q[i] > q_l1_max[i] )
        {
          q2_i = - (1 + lambda) * b * Pg_i;
          //    std::cout << "---- caso 1 "  << std::endl;
          //   std::cout << "b: " << std::endl << b << std::endl;
          //    std::cout << "Pg_i: " << std::endl << Pg_i << std::endl;
        }

        else
        {
          if (q[i] >= q_l0_max[i] && q[i] <= q_l1_max[i] )
          {
            //     std::cout << "---- caso 2"  << std::endl;
            lambda_l = 1 / (1 + exp(-12 *( (q[i] - q_l0_max[i]) / (q_l1_max[i] - q_l0_max[i])  ) + 6 ) );
          }
          else if (q[i] >= q_l1_min[i] && q[i] < q_l0_min[i])
          {
            lambda_l = 1 / (1 + exp(-12 *( (q[i] - q_l0_min[i]) / (q_l1_min[i] - q_l0_min[i])  ) + 6 ) );
            //     std::cout << "---- caso 3 "  << std::endl;
          }

          q2_i = - lambda_l * (1 + lambda)* b * Pg_i;
          //  std::cout << "b: " << std::endl << b << std::endl;
          //    std::cout << "Pg_i: " << std::endl << Pg_i << std::endl;
        }

      }
      q2 = q2 + q2_i;

      // std::cout << "q2_i: " << std::endl << q2_i << std::endl;

    }
    //  std::cout << "Projector operator very small: not possible activate joint avoidance. " << std::endl << q2_i << std::endl;


  }

  // std::cout << "***************************" << std::endl;

  return q2;

}




vpColVector computeQdotLimitAvoidance(const vpColVector & e, const vpMatrix & J, const vpMatrix & J_pinv,const vpColVector & jointMin,const vpColVector & jointMax, const vpColVector & q , const vpColVector & q1, const double & ro,const double & ro1, vpColVector & q_l0_min, vpColVector & q_l0_max, vpColVector &q_l1_min, vpColVector &q_l1_max, bool use_custom_lim = false  )
{
  int n = q.size();
  // Computation Projector operator P
  vpMatrix P(n,n);
  P = computeP(e, J, J_pinv, n);

  // Computation secondary task (q2)
  vpColVector q_sec(n);

  q_sec = computeQsec(P, jointMin, jointMax, q, q1, ro, ro1, q_l0_min, q_l0_max, q_l1_min, q_l1_max, use_custom_lim  );


  return q_sec;
}







#endif
