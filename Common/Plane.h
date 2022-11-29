#pragma once
#include <cmath>

namespace common
{
	struct SPlane
	{
		SPlane() = default;
		SPlane(float vA, float vB, float vC, float vD)
			: _A(vA)
			, _B(vB)
			, _C(vC)
			, _D(vD)
		{}

		float _A;
		float _B;
		float _C;
		float _D;

		float& operator[](int i)
		{
			if (i == 0) return _A;
			if (i == 1) return _B;
			if (i == 2) return _C;
			if (i == 3) return _D;
		}

		const float& operator[](int i) const
		{
			if (i == 0) return _A;
			if (i == 1) return _B;
			if (i == 2) return _C;
			if (i == 3) return _D;
		}

		const int Size() const
		{
			return 4;
		}

		[[nodiscard]] bool isValid()
		{
			if (std::isnan(_A) || std::isnan(_B) || std::isnan(_C) || std::isnan(_D))
				return false;
			if (_A == 0 && _B == 0 && _C == 0)
				return false;
			return true;
		}

		void normalize(float vEpsilon = 0.00001f)
		{
			_ASSERTE(this->isValid());
			for (int i = 0; i < this->Size(); i++)
				if (std::fabsf(this->operator[](i)) < vEpsilon)
					this->operator[](i) = 0.0f;
			float Divisor = 1.0f;
			for (int i = 0; i < this->Size(); i++)
				if (this->operator[](i) != 0)
				{
					Divisor = this->operator[](i);
					break;
				}
			_ASSERTE(Divisor != 0 && !std::isnan(Divisor));
			for (int i = 0; i < this->Size(); i++)
				this->operator[](i) /= Divisor;
		}
	};
}