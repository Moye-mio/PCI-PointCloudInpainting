#pragma once

namespace core
{
	class CStandardHeightMap
	{
	public:
		CStandardHeightMap(float vMaxHeight, float vMinHeight, const Eigen::Matrix<float, -1, -1> vHeightMap);
		~CStandardHeightMap() = default;

		void setMaxHeight(float vHeight);
		void setMinHeight(float vHeight);
		void setHeightMap(const Eigen::Matrix<float, -1, -1> vHeightMap);
		void setHeightAt(float vHeight, int vRow, int vCol);

	private:
		bool __isHeightMapValid(const Eigen::Matrix<float, -1, -1> vHeightMap);

	private:
		float m_MaxHeight;
		float m_MinHeight;
		Eigen::Matrix<float, -1, -1> m_HeightMap;	// each element ranges from 0 to 1
	};
}