#pragma once

namespace core
{
	class CHeightMap
	{
	public:
		CHeightMap() {}
		CHeightMap(const Eigen::Matrix<float, -1, -1>& vHeightMap);
		~CHeightMap() = default;

		[[nodiscard]] bool isEmptyValue(int vRow, int vCol);
		[[nodiscard]] bool isNeighborhoodEmpty(int vRow, int vCol);
		[[nodiscard]] bool setSize(int vWidth, int vHeight);
		[[nodiscard]] bool setHeightMap(const Eigen::Matrix<float, -1, -1>& vHeightMap);
		[[nodiscard]] bool setValueAt(float vHeight, int vRow, int vCol);
		[[nodiscard]] bool setEmptyAt(int vRow, int vCol);
		[[nodiscard]] bool isValid() const;

		float getValueAt(int vRow, int vCol) const;
		int getWidth() const { return m_Map.rows(); }
		int getHeight() const { return m_Map.cols(); }

	private:
		bool __isIndexValid(int vRow, int vCol) const;
		bool __isEmptyValue(int vRow, int vCol) const;

	private:
		Eigen::Matrix<float, -1, -1> m_Map;
		float m_Empty = -FLT_MAX;
	};
}