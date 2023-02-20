#include "pch.h"

const std::string Path1 = TESTMODEL_DIR + std::string("/Trimmed/CrossPlane/GT_CrossPlane.ply");
const std::string Path2 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_GT.ply");
const std::string Path3 = TESTMODEL_DIR + std::string("/Trimmed/Scene/MergeBasedOnGT2.ply");

using namespace pcl::geometric_quality;

void printusage()
{
	std::cout << "pc_psnr cloud_a cloud_b [radiusTimes]" << endl;
	std::cout << "  default radiusTimes is 10" << endl;
}

void printCommand(commandPar& cPar)
{
	cout << "infile1: " << cPar.file1 << endl;
	cout << "infile2: " << cPar.file2 << endl;

	if (cPar.knn > 0)
		cout << "knn = " << cPar.knn << endl;
	else
		cout << "rtimes = " << cPar.rtimes << endl;

	if (cPar.normFile != "")
		cout << "save normals of original point cloud to: " << cPar.normFile << endl;
	cout << "force normal estimation: " << cPar.force << endl;
	if (cPar.singlePass)
		cout << "force running a single pass" << endl;

	cout << endl;
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

TEST(TestPNSR, 1)
{
	PC_t::Ptr pCloud1 = loadPC(Path2);
	PC_t::Ptr pCloud2 = loadPC(Path3);

	core::CSimilarityEstimator Estimator;
	auto r1 = Estimator.compute(pCloud1, pCloud2, core::ESimilarityMode::GPSNR);
	auto r2 = Estimator.compute(pCloud1, pCloud2, core::ESimilarityMode::NSHD);
	EXPECT_TRUE(r1.has_value());
	EXPECT_TRUE(r2.has_value());
	std::cout << "GPSNR: " << r1.value() << std::endl;
	std::cout << "NSHD: " << r2.value() << std::endl;
}