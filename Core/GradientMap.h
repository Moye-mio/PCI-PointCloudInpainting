#pragma once
#include "HeightMap.h"

namespace core
{
	class CGradientMap
	{
	public:
		CGradientMap() {}
		CGradientMap(const Eigen::Matrix<Eigen::Vector2f, -1, -1>& vGradientMap);
		~CGradientMap() = default;

		[[nodiscard]] bool isEmptyValue(int vRow, int vCol);
		[[nodiscard]] bool isNoEmptyValue() const;
		[[nodiscard]] bool isValid() const;
		[[nodiscard]] bool setSize(int vWidth, int vHeight);
		[[nodiscard]] bool setGradientMap(const Eigen::Matrix<Eigen::Vector2f, -1, -1>& vGradientMap);
		[[nodiscard]] bool setValueAt(Eigen::Vector2f vGradient, int vRow, int vCol);
		[[nodiscard]] bool setValueAt(float vGradient, int vRow, int vCol, int vAxis);
		[[nodiscard]] bool setEmptyAt(int vRow, int vCol);

		Eigen::Vector2f getValueAt(int vRow, int vCol) const;
		Eigen::MatrixXf getDataInX() const;
		Eigen::MatrixXf getDataInY() const;
		int getWidth() const { return m_Map.rows(); }
		int getHeight() const { return m_Map.cols(); }
		void generateMask(CHeightMap& voMap);

	private:
		bool __isEmptyValue(int vRow, int vCol) const;

	private:
		Eigen::Matrix<Eigen::Vector2f, -1, -1> m_Map;
		Eigen::Vector2f m_Empty = { -FLT_MAX, -FLT_MAX };
	};
}