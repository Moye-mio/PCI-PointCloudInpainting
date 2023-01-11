#include "pch.h"
#include "Trimmer.h"

using namespace core;

CTrimmer::CTrimmer()
	: m_BaseWidth(5)
	, m_BaseHeight(5)
	, m_Scale(1.0f)
{}

bool CTrimmer::setData(const std::vector<std::pair<Eigen::Vector3i, Point_t>>& vData, float vScale)
{
	_ASSERTE(vData.size() && vScale > 0);
	m_Data = vData;
	m_Scale = vScale;
	return true;
}

void CTrimmer::fillAndTrim(enum class EProj vProj /*= ProjZ*/)
{
	if (vProj != EProj::ProjZ) /* TODO */ return;

	__calcBaseSize();
	Eigen::Matrix<std::vector<unsigned int>, -1, -1> Map;
	__proj2Base(Map);
	__fillEmpty(Map);
	__trim(Map);
}

void CTrimmer::__calcBaseSize()
{
	_ASSERTE(m_Data.size());

	Eigen::Vector2i X{ INT_MAX, -INT_MAX }, Y{ INT_MAX, -INT_MAX }; /* min, max */
	for (const auto& e : m_Data)
	{
		int PosX = e.first.x();
		int PosY = e.first.y();
		X[0] = (PosX < X[0]) ? PosX : X[0];
		X[1] = (PosX > X[1]) ? PosX : X[1];
		Y[0] = (PosY < Y[0]) ? PosY : Y[0];
		Y[1] = (PosY > Y[1]) ? PosY : Y[1];
	}

	_ASSERTE(X[0] == 0 && Y[0] == 0);
	m_BaseWidth = X[1] + 1;
	m_BaseHeight = Y[1] + 1;
}

void CTrimmer::__proj2Base(Eigen::Matrix<std::vector<unsigned int>, -1, -1>& voMap)
{
	_ASSERTE(m_BaseWidth > 0 && m_BaseHeight > 0);

	voMap.resize(m_BaseWidth, m_BaseHeight);
	int Index = 0;
	for (const auto& e : m_Data)
	{
		int Row = e.first.x();
		int Col = e.first.y();
		voMap.coeffRef(Row, Col).push_back(Index);
		Index++;
	}
}

void CTrimmer::__fillEmpty(Eigen::Matrix<std::vector<unsigned int>, -1, -1>& voMap)
{
	/* order dependent */
	for (int i = 0; i < voMap.rows(); i++)
		for (int k = 0; k < voMap.cols(); k++)
		{
			if (voMap.coeff(i, k).size() != 0) continue;
			std::vector<unsigned int> NeighInfo;
			__ExtractNeighInfo(voMap, i, k, NeighInfo);
			unsigned int Index = __fillEmptyElement(NeighInfo, i, k);
			voMap.coeffRef(i, k).emplace_back(Index);
		}
}

void CTrimmer::__ExtractNeighInfo(const Eigen::Matrix<std::vector<unsigned int>, -1, -1>& vMap, int vRow, int vCol, std::vector<unsigned int>& voNeighInfo)
{
	_ASSERTE(vRow < vMap.rows() && vCol < vMap.cols() && vRow >= 0 && vCol >= 0);

	for (int i = vRow - 1; i <= vRow + 1; i++)
		for (int k = vCol - 1; k <= vCol + 1; k++)
		{
			if (i == vRow && k == vCol) continue;
			if (vMap(i, k).size() == 0) continue;
			if (std::abs(i - vRow) + std::abs(k - vCol) > 1) continue;

			for (const auto& e : vMap(i, k))
				voNeighInfo.push_back(e);
		}
}

unsigned int CTrimmer::__fillEmptyElement(const std::vector<unsigned int>& vNeighInfo, int vRow, int vCol)
{
	_ASSERTE(vNeighInfo.size() > 0);

	std::vector<int> Zs;
	std::vector<int> ZNum;

	for (const auto& e : vNeighInfo)
	{
		int Z = m_Data[e].first.z();
		auto Iter = std::find(Zs.begin(), Zs.end(), Z);
		if (Iter == Zs.end())
		{
			Zs.push_back(Z);
			ZNum.push_back(1);
			_ASSERTE(Zs.size() == ZNum.size());
		}
		else
			ZNum[std::distance(Zs.begin(), Iter)]++;
	}

	std::vector<Point_t> Points;
	int Offset = std::distance(ZNum.begin(), std::max_element(ZNum.begin(), ZNum.end()));
	int Depth = Zs[Offset];
	for (const auto& e : vNeighInfo)
	{
		const auto& Point = m_Data[e].second;
		if (m_Data[e].first.z() == Depth)
		{
			Eigen::Vector2f Pos(Point.x, Point.y);
			Eigen::Vector2i Offset(vRow - m_Data[e].first.x(), vCol - m_Data[e].first.y());
			Pos += Offset.cast<float>() * m_Scale;

			{
				std::cout << "Offset: (" << Offset[0] << ", " << Offset[1] << ")\tPos: (" << Point.x << ", " << Point.y << ") -> (" << Pos.x() << ", " << Pos.y() << ")" << std::endl;
			}

			Points.emplace_back(Point_t(Pos[0], Pos[1], Point.z, Point.r, Point.g, Point.b, Point.a));
		}
	}
	m_Data.emplace_back(std::make_pair(Eigen::Vector3i(vRow, vCol, Depth), __calcAveragePoint(Points)));
	return m_Data.size() - 1;
}

Point_t CTrimmer::__calcAveragePoint(const std::vector<Point_t>& vPoints)
{
	_ASSERTE(vPoints.size());

	Eigen::Vector3f Pos(0, 0, 0);
	Eigen::Vector4i Color(0, 0, 0, 0);
	for (const auto& e : vPoints)
	{
		Pos += e.getVector3fMap();
		Color += Eigen::Vector4i((int)e.r, (int)e.g, (int)e.b, (int)e.a);
	}

	Pos /= vPoints.size();
	Color /= vPoints.size();

	return Point_t(Pos[0], Pos[1], Pos[2], (std::uint8_t)Color[0], (std::uint8_t)Color[1], (std::uint8_t)Color[2], (std::uint8_t)Color[3]);
}

Point_t CTrimmer::__calcAveragePoint(const std::vector<unsigned int>& vIndices)
{
	_ASSERTE(vIndices.size());

	std::vector<Point_t> Points;
	for (const auto e : vIndices)
	{
		_ASSERTE(e < m_Data.size());
		Points.emplace_back(m_Data[e].second);
	}
	return __calcAveragePoint(Points);
}

void CTrimmer::__trim(const Eigen::Matrix<std::vector<unsigned int>, -1, -1>& vMap)
{
	_ASSERTE(vMap.size());

	m_Sorted.resize(m_BaseWidth, m_BaseHeight);
	for (int i = 0; i < vMap.rows(); i++)
		for (int k = 0; k < vMap.cols(); k++)
		{
			unsigned int PointNum = vMap(i, k).size();
			_ASSERTE(PointNum != 0);
			if (PointNum == 1)
				m_Sorted.coeffRef(i, k) = m_Data[vMap(i, k)[0]].second;
			else
				m_Sorted.coeffRef(i, k) = __calcAveragePoint(vMap(i, k));
		}
}
