#include "pch.h"

#include "MultilayerSurface.h"

const std::string Path = TESTMODEL_DIR + std::string("/Trimmed/CrossPlane/GT_CrossPlane.ply");

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


TEST(NurbsFitting, DT) 
{
	PC_t::Ptr pCloud;
	core::CNurbsFitting Fitting;
	EXPECT_FALSE(Fitting.run(pCloud, 3, 4, 10));
}

TEST(NurbsFitting, NT_CrossPlane)
{
	PC_t::Ptr pCloud = loadPC(Path);
	core::CNurbsFitting Fitting;
	EXPECT_TRUE(Fitting.run(pCloud, 3, 4, 10));

	ON_NurbsSurface Nurbs;
	Fitting.dumpFittingSurface(Nurbs);
	EXPECT_TRUE(Nurbs.m_cv_count[0] == 19);
	EXPECT_TRUE(Nurbs.m_cv_count[1] == 19);

	Eigen::Matrix<core::SPoint, -1, -1> Ctrlpts;
	Ctrlpts.resize(Nurbs.m_cv_count[0], Nurbs.m_cv_count[1]);
	for (int i = 0; i < Ctrlpts.rows(); i++)
		for (int k = 0; k < Ctrlpts.cols(); k++)
		{
			ON_3dPoint p;
			Nurbs.GetCV(i, k, p);
			Ctrlpts.coeffRef(i, k) = core::SPoint(p.x, p.y, p.z);
		}
	core::CMultilayerSurface Surface(3);
	Surface.setControlPoints(Ctrlpts);
	Surface.setIsSaveMesh(true);
	Surface.setSubNumber(2);
	Surface.setSubLayer(3);
	EXPECT_TRUE(Surface.preCompute());
}