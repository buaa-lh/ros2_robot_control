#ifndef ROBOT_MATH__ROBOT_MATH_HPP_
#define ROBOT_MATH__ROBOT_MATH_HPP_

#include "robot_math/visibility_control.h"
#include "robot_math/coder_array.h"
#include <eigen3/Eigen/Dense>

namespace Eigen
{
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Vector<double, 6> Vector6d;
	typedef Eigen::Matrix<double, 6, 7> Matrix6x7d;
	typedef Eigen::Matrix<double, 7, 6> Matrix7x6d;
	typedef Eigen::Matrix<double, 7, 7> Matrix7d;
	typedef Eigen::Vector<double, 7> Vector7d;
}

namespace robot_math
{

	struct Robot
	{
		double dof;
		coder::array<double, 1U> mass;
		coder::array<double, 3U> inertia;
		coder::array<double, 2U> A;
		coder::array<double, 3U> M;
		double ME[16];
		coder::array<double, 2U> com;
		double gravity[3];
		double TCP[16];
	};
	 Robot urdf2Robot(const std::string &description);
	 void print_robot(const Robot &robot);
    // pose: first three are position
	Eigen::Matrix4d pose2T(const std::vector<double> &pose);
	std::vector<double> T2pose(const Eigen::Matrix4d &T);
	Eigen::Matrix3d so_w(const Eigen::Vector3d &w);
	Eigen::Matrix4d se_twist(const Eigen::Vector6d &V);
	Eigen::Vector7d normalize_twist(const Eigen::Vector6d &V);
	Eigen::Matrix3d exp_r(const Eigen::Vector3d &r);
	Eigen::Vector3d logR(const Eigen::Matrix3d &R);
	Eigen::Matrix4d exp_twist(const Eigen::Vector6d &twist);
	Eigen::Vector6d logT(const Eigen::Matrix4d &T);
	Eigen::Matrix4d invertT(const Eigen::Matrix4d &T);
	Eigen::Matrix6d adjoint_T(const Eigen::Matrix4d &T);
	Eigen::Matrix6d adjoint_V(const Eigen::Vector6d &V);
	Eigen::Matrix4d make_transform(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);

	Eigen::MatrixXd pInv(const Eigen::MatrixXd &matrix, double tol = 1e-4);
	Eigen::Vector3d cond_Matrix(const Eigen::MatrixXd &A);

	Eigen::MatrixXd J_sharp(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M); // X x 6
	Eigen::MatrixXd d_J_sharp(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &dM);
	Eigen::MatrixXd d_J_sharp_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &X);
	Eigen::MatrixXd J_sharp_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &X); // X x
	Eigen::MatrixXd J_sharp_T_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &X);
	Eigen::MatrixXd A_x(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M);
	Eigen::MatrixXd A_x_inv(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M);
	Eigen::MatrixXd A_x_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &X);
	Eigen::VectorXd null_proj(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::VectorXd &v);
	Eigen::MatrixXd null_z(const Eigen::MatrixXd &J); // n x r
	Eigen::MatrixXd d_null_z(const Eigen::MatrixXd &J, const Eigen::MatrixXd &dJ);
	Eigen::MatrixXd z_sharp(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M); // r x n
	Eigen::MatrixXd d_z_sharp(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM);
	Eigen::MatrixXd Mu_x_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &C, const Eigen::MatrixXd &X);
	Eigen::MatrixXd Mu_x(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &C);
	Eigen::MatrixXd A_v(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M);
	Eigen::MatrixXd Mu_v(const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &C);
	Eigen::MatrixXd Mu_xv(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dJ, const Eigen::MatrixXd &Z, const Eigen::MatrixXd &C);
	Eigen::MatrixXd Mu_vx(const Eigen::MatrixXd &J, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &Z, const Eigen::MatrixXd &C);
	Eigen::MatrixXd Mu_vx_X(const Eigen::MatrixXd &J, const Eigen::MatrixXd &Z, const Eigen::MatrixXd &M, const Eigen::MatrixXd &dZ, const Eigen::MatrixXd &dM, const Eigen::MatrixXd &C, const Eigen::MatrixXd &X);

	void cal_motion_error(const Eigen::Matrix4d &T, const Eigen::Matrix4d &T_d,
						  const Eigen::Vector3d &v, const Eigen::Vector3d &w,
						  const Eigen::Vector3d &v_d, const Eigen::Vector3d &w_d,
						  const Eigen::Vector3d &a_d, const Eigen::Vector3d &alpha_d,
						  Eigen::Vector6d &xe, Eigen::Vector6d &dxe, Eigen::Vector6d &ddx_d);

	void bodyTwist2SpatialTwist(const double T[16], const double Vb[6], const double dVb[16], double Vs[6], double dVs[6]);
	void spatialTwist2BodyTwist(const double T[16], const double Vs[6], double const dVs[16], double Vb[6], double dVb[6]);
	void swapOrder(double *buf, int n);

	void getExternalForce(float force[6], float mass, const float offset[6], const float cog[3], const std::vector<double> &pose);

	Eigen::Vector6d twist_estimate(const Eigen::Matrix4d &Td, const Eigen::Matrix4d &Td_pre, double dt);

	void inverse_kin_general(const Robot *robot, Eigen::Matrix4d Td, const std::vector<double> &qref, const double tol[2], std::vector<double> &q, double *flag);

	void forward_kin_general(const Robot *robot, const std::vector<double> &q, Eigen::Matrix4d &T);

	// load robot structure in Json file
	Robot loadRobot(const char *filename);

	// jacobian of ME in the robot, not TCP
	void jacobian_matrix(const Robot *robot, const std::vector<double> &q, Eigen::MatrixXd &J, Eigen::Matrix4d &T);

	// TCP
	void m_c_g_matrix(const Robot *robot, const std::vector<double> &q,
					  const std::vector<double> &dq, Eigen::MatrixXd &M,
					  Eigen::MatrixXd &C, Eigen::VectorXd &G, Eigen::MatrixXd &J, Eigen::MatrixXd &dJ,
					  Eigen::MatrixXd &dM, Eigen::Matrix4d &dT, Eigen::Matrix4d &T);

	// jacobian derivative of ME in the robot, not TCP
	void derivative_jacobian_matrix(const Robot *robot, const std::vector<double> &q, const std::vector<double> &dq,
									Eigen::MatrixXd &dJ, Eigen::MatrixXd &J, Eigen::Matrix4d &dT, Eigen::Matrix4d &T);

	void admittance_control(const Robot *robot, const Eigen::Matrix4d &Tcp, const Eigen::Matrix4d &Td, const Eigen::Vector6d &Vd,
							const Eigen::Matrix3d &Mp, const Eigen::Matrix3d &Bp, const Eigen::Matrix3d &Kp,
							const Eigen::Matrix3d &Mr, const Eigen::Matrix3d &Br, const Eigen::Matrix3d &Kr,
							const std::vector<double> &q, const std::vector<double> &qd, const Eigen::Vector6d &F, double dt,
							Eigen::Vector3d &re, Eigen::Vector3d &pe, Eigen::Vector3d &red, Eigen::Vector3d &ped, bool flag = false, double *Tcmd = nullptr);

	void admittance_error_cal(const Robot *robot, const Eigen::Matrix4d &Tcp, const Eigen::Matrix4d &Td, const Eigen::Vector6d &Vd,
							  const std::vector<double> &q, const std::vector<double> &qd, Eigen::Vector3d &re, Eigen::Vector3d &pe, Eigen::Vector3d &red, Eigen::Vector3d &ped, bool flag);

	void getTaskSpaceMotion(const Robot &robot, const std::vector<double> &q, const std::vector<double> &qd, const std::vector<double> &qdd, double T[16], double V[6], double dV[6]);
	
	void gravityAndInertComp(const Robot &robot, const std::vector<double> &q, const std::vector<double> &qd, const std::vector<double> &qdd, double _T[16], double _Tcb[16], double _Tcs[16], float force[6], float mass, const float offset[6], const float cog[3], const std::vector<double> &pose, const double _G[36]);
	
	Eigen::Vector6d gravityAndInertiaCompensation(const Robot &robot, const Eigen::Matrix4d &Tcp, const Eigen::Matrix4d &Tsensor, const std::vector<double> &q, const std::vector<double> &qd,
												  const std::vector<double> &qdd, const float *rawForce, float mass, const float offset[6], const float cog[3], const Eigen::Matrix3d &mI, double scale = 1.0);

	template <class T, int m, int n>
	coder::array<T, 1> &coder_array_1d_wrapper(Eigen::Matrix<T, m, n> &M)
	{
		static coder::array<T, 1> array;
		array.set(M.data(), m * n);
		return array;
	}

	template <class T, int m, int n>
	coder::array<T, 2> &coder_array_wrapper(Eigen::Matrix<T, m, n> &M)
	{
		static coder::array<T, 2> array;
		array.set(M.data(), m, n);
		return array;
	}

	template <class T, int m, int n>
	coder::array<T, 2> &coder_array_wrapper1(Eigen::Matrix<T, m, n> &M)
	{
		static coder::array<T, 2> array;
		array.set(M.data(), m, n);
		return array;
	}

	template <class T, int m, int n>
	coder::array<T, 2> &coder_array_wrapper2(Eigen::Matrix<T, m, n> &M)
	{
		static coder::array<T, 2> array;
		array.set(M.data(), m, n);
		return array;
	}
	template <class T, int m, int n>
	coder::array<T, 2> &coder_array_wrapper3(Eigen::Matrix<T, m, n> &M)
	{
		static coder::array<T, 2> array;
		array.set(M.data(), m, n);
		return array;
	}
	template <class T, int m, int n>
	coder::array<T, 2> &coder_array_wrapper4(Eigen::Matrix<T, m, n> &M)
	{
		static coder::array<T, 2> array;
		array.set(M.data(), m, n);
		return array;
	}

	template <class T, int m, int n>
	coder::array<T, 2> &coder_array_wrapper5(Eigen::Matrix<T, m, n> &M)
	{
		static coder::array<T, 2> array;
		array.set(M.data(), m, n);
		return array;
	}

	template <class T, int m, int n>
	coder::array<T, 2> &coder_array_wrapper6(Eigen::Matrix<T, m, n> &M)
	{
		static coder::array<T, 2> array;
		array.set(M.data(), m, n);
		return array;
	}

	template <class T, int m, int n>
	coder::array<T, 2> &coder_array_wrapper7(Eigen::Matrix<T, m, n> &M)
	{
		static coder::array<T, 2> array;
		array.set(M.data(), m, n);
		return array;
	}
} // namespace robot_math

#endif // ROBOT_MATH__ROBOT_MATH_HPP_
