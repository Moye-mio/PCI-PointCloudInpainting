#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/UniformRaw.ply");

class TestGC : public testing::Test
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
		_ASSERTE(pcl::io::loadPLYFile<Point_t>(ModelPath, *pCloud) != -1);
		_ASSERTE(pCloud->size());
		return pCloud;
	}
};

TEST_F(TestGC, DT_InValidInput)
{
	PC_t::Ptr pCloud = loadPC(ModelPath);
	std::vector<int> Result;

	GA::CGeneticClustering GC(2, 10, 3, 100);
	GC.setCloud(pCloud);

	hiveCommon::CCPUTimer Timer;
	double t = Timer.getElapsedTime();
	Timer.start();
	/* Timer */
	GC.run();
	/* Timer */
	Timer.stop();
	t = Timer.getElapsedTimeInMS();
	
	GC.dumpBestResult(Result);
	_ASSERTE(Result.size() == pCloud->size());

	PC_t::Ptr pOutput(new PC_t);
	for (int i = 0; i < 3; i++)
	{
		PC_t::Ptr pPart(new PC_t);
		for (int k = 0; k < pCloud->size(); k++)
			if (Result[k] == i)
				pPart->emplace_back(pCloud->at(k));

		for (auto& e : *pPart)
		{
			e.r = 0;
			e.g = 0;
			e.b = 0;

			if (i == 0)
				e.r = 255;
			else if (i == 1)
				e.g = 255;
			else
				e.b = 255;

			pOutput->emplace_back(e);
		}
	}

	pcl::io::savePLYFileBinary("Output.ply", *pOutput);

	std::cout << "Use Time: " << t << "ms" << std::endl;
}