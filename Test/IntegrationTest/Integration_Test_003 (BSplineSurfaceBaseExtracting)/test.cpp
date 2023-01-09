#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestBaseSurface : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	PC_t::Ptr loadPC(const std::string& vPath)
	{
		std::string FileName = hiveUtility::hiveLocateFile(vPath);
		_ASSERTE(!FileName.empty());

		PC_t::Ptr pCloud(new PC_t);
		int r = pcl::io::loadPLYFile<Point_t>(FileName, *pCloud);
		_ASSERTE(r != -1);
		_ASSERTE(pCloud->size());
		std::cout << "Model Point Size: " << pCloud->size() << std::endl;
		return pCloud;
	}

	float m_Epsilon = 0.00001;
};

TEST_F(TestBaseSurface, NT_CrossPlane)
{
	PC_t::Ptr pCloud = loadPC(ModelPath);


}