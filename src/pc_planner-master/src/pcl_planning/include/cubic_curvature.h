#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>
#include <vector>
#include <algorithm>
#include <cmath>
#include <gsl/gsl_integration.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_poly.h>

using namespace std;
using namespace Eigen;

namespace cubic_curvature {

	// forward Integrate
	Vector4d forward(Vector4d init, Vector4d x);

	// get cubic curvature
	Vector4d get_cubic_curvature(Vector4d init, Vector4d ends);
}
