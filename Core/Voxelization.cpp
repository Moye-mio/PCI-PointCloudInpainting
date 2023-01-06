#include "pch.h"

#include "Voxelization.h"
#include "AABBEstimation.h"

using namespace core;


CVoxelization::CVoxelization()
	: m_pCloud(new PC_t)
{}

bool CVoxelization::setCloud(const PC_t::Ptr& vCloud)
{
	_ASSERTE(vCloud != nullptr);
	_ASSERTE(vCloud->size());
	m_pCloud = vCloud;
	return true;
}

bool CVoxelization::setAABB(const SAABB& vBox)
{
	_ASSERTE(vBox.isValid());
	m_Box = vBox;
	return true;
}

bool CVoxelization::generate(float vDist)
{
	_ASSERTE(vDist > 0);
	_ASSERTE(m_pCloud->size());
	
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
		
		_ASSERTE(IsFind == true);
	}

	for (const auto& e : VoxelList)
	{
		const auto& Indices = e.second;
		if (Indices.size() == 0) continue;
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
	return true;
}

void CVoxelization::__init(std::vector<std::pair<Eigen::Vector3i, std::vector<int>>>& voVoxelList, float vDist)
{
	if (!m_Box.isValid())
	{
		CAABBEstimation AABBEstimation(m_pCloud);
		m_Box = AABBEstimation.compute();
		_ASSERTE(m_Box.isValid());
	}

	Eigen::Vector3f Scale = (m_Box._Max - m_Box._Min) / vDist;
	m_Scale = Eigen::Vector3i(std::ceil(Scale[0]), std::ceil(Scale[1]), std::ceil(Scale[2]));
	m_Scale[0] = (m_Scale[0] == 0) ? 1 : m_Scale[0];
	m_Scale[1] = (m_Scale[1] == 0) ? 1 : m_Scale[1];
	m_Scale[2] = (m_Scale[2] == 0) ? 1 : m_Scale[2];

	for (int i = 0; i < m_Scale[0]; i++)
		for (int k = 0; k < m_Scale[1]; k++)
			for (int m = 0; m < m_Scale[2]; m++)
				voVoxelList.emplace_back(std::make_pair(Eigen::Vector3i(i, k, m), std::vector<int>()));
	_ASSERTE(voVoxelList.size() > 0);
}

Eigen::Vector3i CVoxelization::__computeOffset(const Point_t& vPoint, float vDist)
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
