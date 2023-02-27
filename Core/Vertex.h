#pragma once

#include <cmath>
#include <Eigen/Eigen>

namespace core
{
	struct SVertex
	{
		float x, y, z;
		float u, v;
		float nx, ny, nz;

		SVertex()
			: x(0), y(0), z(0), u(0), v(0), nx(0), ny(0), nz(0) {}

		SVertex(float vX, float vY, float vZ)
			: x(vX), y(vY), z(vZ), u(0), v(0), nx(0), ny(0), nz(0) {}

		SVertex(float vX, float vY, float vZ, float vU, float vV)
			: x(vX), y(vY), z(vZ), u(vU), v(vV), nx(0), ny(0), nz(0) {}

		SVertex(float vX, float vY, float vZ, float vU, float vV, float vNx, float vNy, float vNz)
			: x(vX), y(vY), z(vZ), u(vU), v(vV), nx(vNx), ny(vNy), nz(vNz) {}

		const bool isValid() const
		{
			if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(u) || std::isnan(v) || std::isnan(nx) || std::isnan(ny) || std::isnan(nz))	return false;
			else																																			return true;
		}

		const Eigen::Vector3f getXYZ() const
		{
			return Eigen::Vector3f(x, y, z);
		}

		const Eigen::Vector2f getUV() const
		{
			return Eigen::Vector2f(u, v);
		}

		const Eigen::Vector3f getUV3f() const
		{
			return Eigen::Vector3f(u, v, 0);
		}

		const Eigen::Vector3f getNormal3f() const 
		{
			return Eigen::Vector3f(nx, ny, nz);
		}

		void setNormals(float vNx, float vNy, float vNz)
		{
			nx = vNx;
			ny = vNy;
			nz = vNz;
		}
	};
}