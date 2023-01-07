#include "pch.h"

class TestDataTrimmer : public testing::Test
{
protected:
	void SetUp() override
	{

	}

	void TearDown() override
	{
	}

	void generatePlane(std::vector<Eigen::Vector3i>& voData, Eigen::Matrix<unsigned int, -1, -1>& voSorted)
	{
		int Size = 4;
		voSorted.resize(Size, Size);
		std::vector<Eigen::Vector3i> Data;
		for (int i = 0; i < Size; i++)
			for (int k = 0; k < Size; k++)
			{
				voSorted.coeffRef(i, k) = i * Size + k;
				Data.emplace_back(Eigen::Vector3i(i, k, 0));
			}
		voData = Data;
	}

	void generateConcave(std::vector<Eigen::Vector3i>& voData, Eigen::Matrix<unsigned int, -1, -1>& voSorted)
	{
		int Size = 4;
		for (int i = 0; i < 4; i++)
			for (int k = 0; k < 4; k++)
				for (int m = 0; m < 4; m++)
				{
					if ((i == 2 || i == 3) && m > 0) continue;
					voData.emplace_back(Eigen::Vector3i(i, k, m));
				}


		voSorted.resize(10, 4);
		Eigen::Matrix<Eigen::Vector3i, -1, -1> Sorted;
		Sorted.resize(10, 4);
		for (int i = 0; i < Size; i++)
		{
			for (int k = 0; k < Size; k++)
				Sorted.coeffRef(k, i) = Eigen::Vector3i(0, i, Size - k - 1);
			for (int k = 1; k < Size; k++)
				Sorted.coeffRef(Size + k - 1, i) = Eigen::Vector3i(k, i, 0);
			for (int k = 1; k < Size; k++)
				Sorted.coeffRef(Size * 2 + k - 1, i) = Eigen::Vector3i(Size - 1, i, k);
		}

		for (int i = 0; i < 10; i++)
			for (int k = 0; k < 4; k++)
			{
				const auto& e = Sorted.coeff(i, k);
				auto Iter = std::find(voData.begin(), voData.end(), e);
				_ASSERTE(Iter != voData.end());
				voSorted.coeffRef(i, k) = std::distance(voData.begin(), Iter);
			}
	}

	bool isMatrixSame(const Eigen::Matrix<unsigned int, -1, -1>& vLhs, Eigen::Matrix<unsigned int, -1, -1>& vRhs) const
	{
		if (vLhs.rows() != vRhs.rows() || vLhs.cols() != vRhs.cols()) return false;
		for (int i = 0; i < vLhs.rows(); i++)
			for (int k = 0; k < vLhs.cols(); k++)
				if (vLhs.coeff(i, k) != vRhs.coeff(i, k))
					return false;
		return true;
	}
};

TEST_F(TestDataTrimmer, DT_InValidInput)
{
	core::CDataTrimmer Trimmer;
	ASSERT_DEATH(Trimmer.sort(std::vector<Eigen::Vector3i>()), "");
}

TEST_F(TestDataTrimmer, NT_Plane)
{
	std::vector<Eigen::Vector3i> Data;
	Eigen::Matrix<unsigned int, -1, -1> GTSorted, Sorted;
	generatePlane(Data, GTSorted);
	core::CDataTrimmer Trimmer;
	Trimmer.sort(Data);
	Trimmer.dumpSortedIndices(Sorted);
	ASSERT_TRUE(isMatrixSame(Sorted, GTSorted));
}