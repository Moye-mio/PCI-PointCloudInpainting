#pragma once

namespace core
{
	class CGradientMap
	{
	public:
		CGradientMap() {}
		CGradientMap(const Eigen::Matrix<Eigen::Vector2f, -1, -1>& vGradientMap);
		~CGradientMap() = default;

		[[nodiscard]] bool isEmptyValue(int vRow, int vCol);
		[[nodiscard]] bool setSize(int vWidth, int vHeight);
		[[nodiscard]] bool setGradientMap(const Eigen::Matrix<Eigen::Vector2f, -1, -1>& vGradientMap);
		[[nodiscard]] bool setValueAt(Eigen::Vector2f vGradient, int vRow, int vCol);
		[[nodiscard]] bool setEmptyAt(int vRow, int vCol);

		Eigen::Vector2f getValueAt(int vRow, int vCol) const;
		int getWidth() const { return m_Map.rows(); }
		int getHeight() const { return m_Map.cols(); }

	private:
		bool __isEmptyValue(int vRow, int vCol);

	private:
		Eigen::Matrix<Eigen::Vector2f, -1, -1> m_Map;
		Eigen::Vector2f m_Empty = { -FLT_MAX, -FLT_MAX };
	};
}