//////////////////////////////////////////////////////////////////////////////
//	  Author: Jake Fountain 2015
//    
/// \file CalibrationTools.cpp
/// \brief CPP file for CalibrationTools.h
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <math.h>
#include "CalibrationTools.h"

namespace autocal{

	using utility::math::geometry::UnitQuaternion;
	using utility::math::matrix::Rotation3D;
	using utility::math::matrix::Transform3D;

	int CalibrationTools::kroneckerDelta(int i, int j){
		return (i == j) ? 1 : 0;
	}

	/*
	Returns matrix M(v) such that for any vector x, cross(v,x) = dot(M(v),x)
	*/
	arma::mat33 CalibrationTools::crossMatrix(const arma::vec3& v){
		arma::mat33 omega;
		omega <<  0 <<    -v[2] <<  v[1] << arma::endr
			  << v[2] <<     0  << -v[0] << arma::endr
			  << -v[1]<<   v[0] <<    0  << arma::endr;
		return omega;
	}

	//
	// void CalibrationTools::checkOrthonormal(M){

	// 	if M.shape[0] == 4:
	// 		if numpy.linalg.norm(M[3,:3]) != 0:
	// 			print "\n\n\n\n\n\n\nBottom row of matrix non-zero ", numpy.linalg.norm(M[3,:3]), "\n\n\n\n\n\n\n"
	// 			return False
	// 	for i in range(3):
	// 		for j in range(3):
	// 			if not (numpy.allclose(dot(M[:3,i],M[:3,j]), kroneckerDelta(i,j))):
	// 				print "\n\n\n\n\n\n\nColumn ", i, " and Column ", j, " are not orthonormal: dotprod = ", dot(M[:3,i],M[:3,j]), "\n\n\n\n\n\n\n"
	// 				return False
	// 	return True
	// }

	/*
		Computes least squares solution x to Ax = b using the psuedoinverse
	*/
	bool CalibrationTools::solveWithSVD(const arma::mat& A, const arma::vec& b, arma::mat& x){
		if (A.n_rows != b.n_rows){
			throw("Problem badly formulated!");
		}
		//Compute pseudo inverse
		arma::mat pinvA;
		bool success = arma::pinv(pinvA,A);
		//Compute x in Ax=b
		if(success) x = pinvA * b;
		//Return whether or not the SVD was performed correctly
		return success;
	}

		/*
		solves AX=YB for X,Y and A in sampleA, B in sampleB

		Source:
		@ARTICLE{Zhuang1994, 
		author={Hanqi Zhuang and Roth, Zvi S. and Sudhakar, R.}, 
		journal={Robotics and Automation, IEEE Transactions on}, 
		title={Simultaneous robot/world and tool/flange calibration by solving homogeneous transformation equations of the form AX=YB}, 
		year={1994}, 
		month={Aug}, 
		volume={10}, 
		number={4}, 
		pages={549-554}
		}	
		*/
	std::pair<Transform3D, Transform3D> CalibrationTools::solveHomogeneousDualSylvester(const std::vector<Transform3D>& samplesA,const std::vector<Transform3D>& samplesB, bool& success){
		if(samplesA.size() < 3 || samplesB.size() < 3){
			std::cout << "CalibrationTools::solveHomogeneousDualSylvester - NEED MORE THAN 2 SAMPLES" << std::endl;
			throw std::domain_error("CalibrationTools::solveHomogeneousDualSylvester - NEED MORE THAN 2 SAMPLES");
		}
		Transform3D X,Y;

		arma::mat combinedG;
		arma::vec combinedC;

		float a0 = 0;
		float b0 = 0;

		arma::vec3 a;
		arma::vec3 b;

		for (int i = 0; i < samplesA.size(); i++){
			const Transform3D& A = samplesA[i]; 
			const Transform3D& B = samplesB[i]; 

			//Get Quaternions for rotations
			UnitQuaternion quat_a(A.rotation()); 
			a0 = quat_a[0]; 
			a = quat_a.rows(1,3); 
			
			UnitQuaternion quat_b(B.rotation()); 
			b0 = quat_b[0]; 
			b = quat_b.rows(1,3); 

			//Compute G in Gw = C
			if(std::fabs(a0) < 1e-10){
				std::cout << "\n\n\n\n\nBAD SAMPLE\n\n\n\n" << std::endl; 
				return std::pair<Transform3D, Transform3D>(); 
			}

			arma::mat G1 = a0 * arma::eye(3,3) + crossMatrix(a) + a*a.t() / a0; 
			arma::mat G2 = -b0 * arma::eye(3,3) + crossMatrix(b) - a*b.t() / a0; 
			
			arma::mat G = arma::join_rows(G1,G2); 

			//Compute C in Gw = C
			arma::vec C = b - (b0/a0) * a; 

			if (i == 0){	
				combinedG = G; 
				combinedC = C; 
			}else{	
				combinedG = arma::join_cols(combinedG,G); 
				combinedC = arma::join_cols(combinedC,C); 
			}
		}

		arma::vec w;
		bool wSuccess = solveWithSVD(combinedG,combinedC, w); 
		if(!wSuccess){
			//If SVD fails, return identity
			std::cout << __FILE__ << " : " << __LINE__ << " - WARNING: SVD FAILED" << std::endl;
			success = false;
			return std::pair<Transform3D, Transform3D>(X,Y);
		}
		
		//Compute x and y Quaternions
		UnitQuaternion x, y; 
		y[0] = 1 / std::sqrt(1 + w[3]*w[3] + w[4]*w[4] + w[5]*w[5]); 
		if (std::fabs(y[0])< 1e-3){
			std::cout << "\n\n\n\n\n\n\ny[0] == 0 so you need to rotate the ref base with respect to the base\n\n\n\n\n\n\n" << std::endl; 
			return std::pair<Transform3D, Transform3D>(); 
		}
		y.rows(1,3) = y[0] * w.rows(3,5); 
		x.rows(1,3) = y[0] * w.rows(0,2); 
		
		float x0 = arma::dot((a/a0), x.rows(1,3)) + (b0/a0) * y[0] - arma::dot((b/a0) , y.rows(1,3)); 
		int x_sign = x0 > 0 ? 1 : -1; 
		//TODO: figure out how to handle when x is nan
		x[0] = x_sign * std::sqrt(1 - std::fmin(1, x[1]*x[1] + x[2]*x[2] + x[3]*x[3]) ); 
		// check:
		// check = tr.quaternion_multiply(tr.quaternion_inverse(tr.quaternion_multiply(quat_a,x)), tr.quaternion_multiply(y,quat_b))

		Rotation3D Rx(x); 
		Rotation3D Ry(y); 

		arma::mat combinedF; 
		arma::vec combinedD; 

		for (int i = 0; i < samplesA.size(); i++){
			Rotation3D RA = samplesA[i].submat(0,0,2,2); 
			arma::vec3 pA = samplesA[i].submat(0,3,2,3); 
			arma::vec3 pB = samplesB[i].submat(0,3,2,3); 

			arma::mat F = arma::join_rows(RA,-arma::eye(3,3)); 

			arma::vec D = Ry * pB - pA; 

			if (i == 0){	
				combinedF = F; 
				combinedD = D; 
			}else{	
				combinedF = arma::join_cols(combinedF,F); 
				combinedD = arma::join_cols(combinedD,D); 
			}
		}
		arma::vec pxpy;
		bool pxpySuccess = solveWithSVD(combinedF,combinedD,pxpy); 
		if(!pxpySuccess){
			//If SVD fails, return identity
			std::cout << __FILE__ << " : " << __LINE__ << " - WARNING: SVD FAILED" << std::endl;
			success = false;
			return std::pair<Transform3D, Transform3D>(X,Y);
		}
		
		X.submat(0,0,2,2) = Rx; 
		Y.submat(0,0,2,2) = Ry; 

		X.submat(0,3,2,3) = pxpy.rows(0,2); 
		Y.submat(0,3,2,3) = pxpy.rows(3,5); 

		return std::pair<Transform3D, Transform3D>(X,Y);
	}
}
