//////////////////////////////////////////////////////////////////////////////
//	  Author: Jake Fountain 2015
//    
//
//////////////////////////////////////////////////////////////////////////////

#ifndef AUTOCAL_CALIBRATION_TOOLS_H
#define AUTOCAL_CALIBRATION_TOOLS_H

#include <armadillo>
#include <math.h>
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/geometry/UnitQuaternion.h"

namespace autocal{

	class CalibrationTools{
	public:
		static int kroneckerDelta(int i, int j);

		static arma::mat33 crossMatrix(const arma::vec3& v);

		static bool solveWithSVD(const arma::mat& A, const arma::vec& b, arma::mat& x);

		static std::pair<utility::math::matrix::Transform3D, utility::math::matrix::Transform3D> solveHomogeneousDualSylvester(const std::vector<utility::math::matrix::Transform3D>& samplesA,const std::vector<utility::math::matrix::Transform3D>& samplesB, bool& success);

	};
}
#endif
