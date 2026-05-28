#include <geometry_msgs/msg/transform.hpp>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpMatrix.h>
#include <visp/vpGaussRand.h>
#include <visp/vpQuadProg.h>

std::vector<double> computeVisualServo(const geometry_msgs::msg::Transform &cMo,
                                       const std::array<double, 42> &eJe)
{
  // a lot of static init here, just to allocate once
  // build Jacobian matrix
  static vpMatrix J(6,7);
  // init J from eJe
  auto elem{eJe.begin()};
  for (size_t row=0; row<6; row++)
    for (size_t col=0; col<7; col++)
      J[row][col] = *elem++;

  // interaction matrix and VS error
  static auto L{[](){vpMatrix L(5,6); L[0][0] = L[1][1] = L[2][2] = -1; return L;}()};
  vpColVector e(5);

  // 3D feature
  e[0] = cMo.translation.x;
  e[1] = cMo.translation.y;
  e[2] = cMo.translation.z - 0.6; // desired depth
  L.insert(vpColVector::skew({cMo.translation.x,
                              cMo.translation.y,
                              cMo.translation.z}), 0, 3);

  // rotation wrt x, keep horizon horizontal
  const auto oRc{vpRotationMatrix(vpQuaternionVector(cMo.rotation.x,
                                                     cMo.rotation.y,
                                                     cMo.rotation.z,
                                                     cMo.rotation.w)).t()};
  e[3] = oRc[2][0];
  L[3][4] = -oRc[2][2];
  L[3][5] = oRc[2][1];

  // rotation wrt z, keep optical axis horizontal
  e[4] = oRc[2][2];
  L[4][3] = -oRc[2][1];
  L[4][4] = oRc[2][0];

  // fixed transform ee to camera
  const static vpVelocityTwistMatrix cWe(vpTranslationVector(.012, -0.038, 0.142),
                                         vpThetaUVector(0, 0, M_PI/2));

  // add some useless random noise because we can
  static const auto I7{[](){vpMatrix I; I.eye(7); return I;}()};
  static vpGaussRand noise(.08, 0);
  static vpColVector z(7);
  for(int i = 0; i < 7; ++i)
    z[i] = (i+1)*noise();

  static vpColVector qdot(7);
  vpQuadProg::solveQPe(I7, z, L*cWe*J, -2.5*e, qdot);
  return qdot.toStdVector();
}

