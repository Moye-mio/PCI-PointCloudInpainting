#include "pch.h"

#include "BiasedVoxelization.h"
#include "AABBEstimation.h"

using namespace core;

CBiasedVoxelization::CBiasedVoxelization()
	: m_pCloud(new PC_t)
	, m_Thres(0)
{}

bool CBiasedVoxelization::setCloud(const PC_t::Ptr& vCloud)
{
	if (vCloud == nullptr) return false;
	if (vCloud->size() == 0) return false;

	m_pCloud = vCloud;
	return true;
}

bool CBiasedVoxelization::setAABB(const SAABB& vBox)
{
	if (!vBox.isValid()) return false;

	m_Box = vBox;
	return true;
}

bool CBiasedVoxelization::setDenoiseThres(std::uint32_t vThres)
{
	m_Thres = vThres;
	return true;
}

bool CBiasedVoxelization::generate(float vDist)
{
	if (!__isDistValid(vDist)) return false;
	if (m_pCloud->size() == 0) return false;

	std::vector<std::pair<Eigen::Vector3i, std::vector<int>>> VoxelList;
	__init(VoxelList, vDist);

	for (int i = 0; i < m_pCloud->size(); i++)
	{
		Eigen::Vector3i Offset = __computeOffset(m_pCloud->at(i), vDist);
		bool IsFind = false;
		for (auto& e : VoxelList)
			if (e.first == Offset)
			{
				e.second.push_back(i);
				IsFind = true;
				break;
			}

		if (IsFind == false)
		{
#ifdef _LOG
			std::cout << "Can not find matching Voxel..." << std::endl;
			std::cout << "Point: " << m_pCloud->at(i) << "\t, Index: " << i << std::endl;
#endif // _LOG
			return false;
		}
	}

	std::uint32_t ValidCount = 0;
	std::uint32_t DiscardCount = 0;

	for (const auto& e : VoxelList)
	{
		const auto& Indices = e.second;
		if (Indices.size() <= m_Thres)
		{
			DiscardCount += Indices.size();
			continue;
		}
		else
			ValidCount += Indices.size();

		Eigen::Vector3f Pos(0.0f, 0.0f, 0.0f);
		Eigen::Vector4i Color(0, 0, 0, 0);
		for (int i = 0; i < Indices.size(); i++)
		{
			const auto& Point = m_pCloud->at(Indices[i]);
			Pos += Eigen::Vector3f(Point.x, Point.y, Point.z);
			Color += Eigen::Vector4i((int)Point.r, (int)Point.g, (int)Point.b, (int)Point.a);
		}

		Pos = Pos / Indices.size();
		Color = Color / Indices.size();
		m_Voxel.emplace_back(std::make_pair(e.first, Point_t(Pos[0], Pos[1], Pos[2], (std::uint8_t)Color[0], (std::uint8_t)Color[1], (std::uint8_t)Color[2], (std::uint8_t)Color[3])));
	}

	if (DiscardCount + ValidCount != m_pCloud->size())
	{
#ifdef _LOG
		std::cout << "ERROR: Discard + Valid Can not match Total Point Size..." << m_Voxel.size() << std::endl;
#endif // _LOG
		return false;
	}

#ifdef _LOG
	std::cout << "Valid Voxel Size: " << m_Voxel.size() << std::endl;
	std::cout << "Discard: " << DiscardCount << std::endl;
	std::cout << "Valid: " << ValidCount << std::endl;
#endif // _LOG

	return true;
}

bool CBiasedVoxelization::__init(std::vector<std::pair<Eigen::Vector3i, std::vector<int>>>& voVoxelList, float vDist)
{
	if (!__isDistValid(vDist)) return false;

	voVoxelList.clear();
	voVoxelList.shrink_to_fit();

	if (!m_Box.isValid())
	{
		CAABBEstimation Estimation(m_pCloud);
		m_Box = Estimation.compute();
		if (!m_Box.isValid())
		{
#ifdef _LOG
			std::cout << "AABB Estimation Failed..." << std::endl;
#endif // _LOG
			return false;
		}
	}

	__biasAABB(vDist);

	Eigen::Vector3f Scale = (m_Box._Max - m_Box._Min) / vDist;
	m_Scale = Eigen::Vector3i(std::ceil(Scale[0]), std::ceil(Scale[1]), std::ceil(Scale[2]));
	m_Scale[0] = (m_Scale[0] == 0) ? 1 : m_Scale[0];
	m_Scale[1] = (m_Scale[1] == 0) ? 1 : m_Scale[1];
	m_Scale[2] = (m_Scale[2] == 0) ? 1 : m_Scale[2];

	for (int i = 0; i < m_Scale[0]; i++)
		for (int k = 0; k < m_Scale[1]; k++)
			for (int m = 0; m < m_Scale[2]; m++)
				voVoxelList.emplace_back(std::make_pair(Eigen::Vector3i(i, k, m), std::vector<int>()));

	if (voVoxelList.size() == 0)
	{
#ifdef _LOG
		std::cout << "VoxelList Init Failed..." << std::endl;
#endif // _LOG

		return false;
	}

#ifdef _LOG
	std::cout << "VoxelList Init Succeed..." << std::endl;
	std::cout << "VoxelList Size: (" << m_Scale[0] << ", " << m_Scale[1] << ", " << m_Scale[2] << ")" << std::endl;
#endif // _LOG

	return true;
}

Eigen::Vector3i CBiasedVoxelization::__computeOffset(const Point_t& vPoint, float vDist)
{
	int OffsetX = (vPoint.getVector3fMap()[0] - m_Box._Min[0]) / vDist;
	int OffsetY = (vPoint.getVector3fMap()[1] - m_Box._Min[1]) / vDist;
	int OffsetZ = (vPoint.getVector3fMap()[2] - m_Box._Min[2]) / vDist;

	if (m_Box._Max[0] == m_Box._Min[0]) OffsetX = 0;
	if (m_Box._Max[1] == m_Box._Min[1]) OffsetY = 0;
	if (m_Box._Max[2] == m_Box._Min[2]) OffsetZ = 0;

	if (OffsetX == m_Scale[0]) OffsetX--;
	if (OffsetY == m_Scale[1]) OffsetY--;
	if (OffsetZ == m_Scale[2]) OffsetZ--;

	return Eigen::Vector3i(OffsetX, OffsetY, OffsetZ);
}

bool CBiasedVoxelization::__isDistValid(float vDist)
{
	if (vDist <= 0)		return false;
	else				return true;
}

bool CBiasedVoxelization::__biasAABB(float vDist)
{
	if (!m_Box.isValid()) return false;
	if (!__isDistValid(vDist)) return false;

	m_Box._Min -= Eigen::Vector3f(vDist, vDist, vDist) * 0.9f;
	m_Box._Max += Eigen::Vector3f(vDist, vDist, vDist) * 0.9f;

	if (!m_Box.isValid())
	{
#ifdef _LOG
		std::cout << "AABB Bias Failed..." << std::endl;
#endif // _LOG
		return false;
	}

#ifdef _LOG
	std::cout << "AABB Bias Succeed..." << std::endl;
#endif // _LOG
	return true;
}
