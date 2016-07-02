#pragma once

namespace Geo 
{
// Solution of a 3x3 linear system
bool invert_3x3(const double A[3][3], double iA[3][3]);

bool solve_3x3(const double A[3][3], double x[3], const double b[3]);

// Solution of a 3x3 linear system
bool invert_2x2(const double A[2][2], double iA[2][2]);

bool solve_2x2(const double A[2][2], double x[2], const double b[2]);

}// namespace Geo