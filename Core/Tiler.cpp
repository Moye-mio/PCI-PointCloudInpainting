#include "pch.h"
#include "Tiler.h"
#include "AABBEstimation.h"

using namespace core;

bool CTiler::run(const PC_t::Ptr& vCloud, int vSizeX, int vSizeY, float vRate)
{
	_HIVE_EARLY_RETURN(vCloud == nullptr, "Cloud is nullptr", false);
	_HIVE_EARLY_RETURN(vCloud->size() == 0, "Cloud is empty", false);
	_HIVE_EARLY_RETURN(vSizeX <= 0 || vSizeY <= 0, "Size is Invalid", false);
	_HIVE_EARLY_RETURN(vRate < 0 || vRate > 1, "Rate is Invalid", false);

	CAABBEstimation Estimation(vCloud);
	SAABB Box = Estimation.compute();
	_HIVE_EARLY_RETURN(Box.isValid() == false, "AABB is Invalid", false);

	float ScaleX = (Box._Max[0] - Box._Min[0]) / vSizeX;
	float ScaleY = (Box._Max[1] - Box._Min[1]) / vSizeY;
	float OffsetX = ScaleX * vRate;
	float OffsetY = ScaleY * vRate;

	Eigen::Matrix<Eigen::Vector4f, -1, -1> Ranges;
	Ranges.resize(vSizeX, vSizeY);
	for (int i = 0; i < vSizeX; i++)
		for (int k = 0; k < vSizeY; k++)
		{
			Eigen::Vector4f Range(i * ScaleX, k * ScaleY, (i + 1) * ScaleX, (k + 1) * ScaleY);
			if (i != 0)
				Range[0] -= OffsetX;
			if (k != 0)
				Range[1] -= OffsetY;
			if (i != vSizeX - 1)
				Range[2] += OffsetX;
			if (k != vSizeY - 1)
				Range[3] += OffsetY;
			Ranges.coeffRef(i, k) = Range;
		}
	
	Eigen::Matrix<std::vector<int>, -1, -1> Tiles;
	Tiles.resize(vSizeX, vSizeY);
	for (int i = 0; i < vCloud->size(); i++)
	{
		const auto& p = vCloud->at(i);
		for (int Row = 0; Row < vSizeX; Row++)
			for (int Col = 0; Col < vSizeY; Col++)
			{
				if (__isInRange(Ranges.coeff(Row, Col), Eigen::Vector2f(p.x, p.y)))
					Tiles.coeffRef(Row, Col).emplace_back(i);
			}
	}

	for (int i = 0; i < vSizeX; i++)
		for (int k = 0; k < vSizeY; k++)
		{
			m_Tiles.emplace_back(std::move(Tiles.coeffRef(i, k)));
			m_Coors.emplace_back(std::make_pair(i, k));
		}
}

bool CTiler::__isInRange(const Eigen::Vector4f& vRange, const Eigen::Vector2f& vPos)
{
	_HIVE_EARLY_RETURN(std::isnan(vRange[0]) || std::isnan(vRange[1]) || std::isnan(vRange[2]) || std::isnan(vRange[3]), "Range is invalid", false);
	_HIVE_EARLY_RETURN(std::isnan(vPos[0]) || std::isnan(vPos[1]), "Pos is invalid", false);

	if (vPos[0] >= vRange[0] && vPos[1] >= vRange[1] && vPos[0] <= vRange[2] && vPos[1] <= vRange[3])
		return true;
	else
		return false;
}
