#include "pch.h"

const std::string BunnyPath = TESTMODEL_DIR + std::string("/Bunny.ply");

class TestBSS : public testing::Test
{
protected:
	void SetUp() override
	{

	}

	void TearDown() override
	{
	}

	void PointCloud2Vector3d(PC_t::Ptr cloud, pcl::on_nurbs::vector_vec3d& data) {
		for (const auto& p : *cloud) {
			if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
				data.emplace_back(p.x, p.y, p.z);
		}
	}

	void visualizeCurve(ON_NurbsCurve& curve, ON_NurbsSurface& surface, pcl::visualization::PCLVisualizer& viewer) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, surface, curve_cloud, 4);
		for (std::size_t i = 0; i < curve_cloud->size() - 1; i++) {
			pcl::PointXYZRGB& p1 = curve_cloud->at(i);
			pcl::PointXYZRGB& p2 = curve_cloud->at(i + 1);
			std::ostringstream os;
			os << "line" << i;
			viewer.removeShape(os.str());
			viewer.addLine<pcl::PointXYZRGB>(p1, p2, 1.0, 0.0, 0.0, os.str());
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int i = 0; i < curve.CVCount(); i++) {
			ON_3dPoint p1;
			curve.GetCV(i, p1);

			double pnt[3];
			surface.Evaluate(p1.x, p1.y, 0, 3, pnt);
			pcl::PointXYZRGB p2;
			p2.x = float(pnt[0]);
			p2.y = float(pnt[1]);
			p2.z = float(pnt[2]);

			p2.r = 255;
			p2.g = 0;
			p2.b = 0;

			curve_cps->push_back(p2);
		}
		viewer.removePointCloud("cloud_cps");
		viewer.addPointCloud(curve_cps, "cloud_cps");
	}
};

TEST_F(TestBSS, init)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）

	if (pcl::io::loadPLYFile<pcl::PointXYZ>(BunnyPath, *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_file.pcd with the following fields: "
		<< std::endl;
	
	pcl::visualization::PCLVisualizer viewer("pcd viewer");
	viewer.addPointCloud(cloud);
	viewer.updateCamera();
}


TEST_F(TestBSS, 1)
{
	//pcl::visualization::PCLVisualizer viewer("B-spline surface fitting");
	//viewer.setSize(800, 600);

	//// ############################################################################
	//// load point cloud

	//printf("  loading %s\n", BunnyPath.c_str());
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//pcl::PCLPointCloud2 cloud2;
	//pcl::on_nurbs::NurbsDataSurface data;

	//if (pcl::io::loadPLYFile(BunnyPath, cloud2) == -1)
	//	throw std::runtime_error("  PCD file not found.");

	//fromPCLPointCloud2(cloud2, *cloud);
	//PointCloud2Vector3d(cloud, data.interior);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> handler(cloud, 0, 255, 0);
	//viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, handler, "cloud_cylinder");
	//printf("  %lu points in data set\n", cloud->size());
	//viewer.initCameraParameters();
	//viewer.resetCamera();

	// ############################################################################
	// fit B-spline surface

	// parameters
	//unsigned order(3);
	//unsigned refinement(5);
	//unsigned iterations(10);
	//unsigned mesh_resolution(256);

	//pcl::on_nurbs::FittingSurface::Parameter params;
	//params.interior_smoothness = 0.2;
	//params.interior_weight = 1.0;
	//params.boundary_smoothness = 0.2;
	//params.boundary_weight = 0.0;

	//// initialize
	//printf("  surface fitting ...\n");
	//ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
	//pcl::on_nurbs::FittingSurface fit(&data, nurbs);
	////  fit.setQuiet (false); // enable/disable debug output

	//// mesh for visualization
	//pcl::PolygonMesh mesh;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//std::vector<pcl::Vertices> mesh_vertices;
	//std::string mesh_id = "mesh_nurbs";
	//pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);
	//viewer.addPolygonMesh(mesh, mesh_id);

	//// surface refinement
	//for (unsigned i = 0; i < refinement; i++) {
	//	fit.refine(0);
	//	fit.refine(1);
	//	fit.assemble(params);
	//	fit.solve();
	//	pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
	//	viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
	//	viewer.spinOnce();
	//}

	//// surface fitting with final refinement level
	//for (unsigned i = 0; i < iterations; i++) {
	//	fit.assemble(params);
	//	fit.solve();
	//	pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
	//	viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
	//	viewer.spinOnce();
	//}

	//// ############################################################################
	//// fit B-spline curve

	//// parameters
	//pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
	//curve_params.addCPsAccuracy = 5e-2;
	//curve_params.addCPsIteration = 3;
	//curve_params.maxCPs = 200;
	//curve_params.accuracy = 1e-3;
	//curve_params.iterations = 100;

	//curve_params.param.closest_point_resolution = 0;
	//curve_params.param.closest_point_weight = 1.0;
	//curve_params.param.closest_point_sigma2 = 0.1;
	//curve_params.param.interior_sigma2 = 0.00001;
	//curve_params.param.smooth_concavity = 1.0;
	//curve_params.param.smoothness = 1.0;

	//// initialisation (circular)
	//printf("  curve fitting ...\n");
	//pcl::on_nurbs::NurbsDataCurve2d curve_data;
	//curve_data.interior = data.interior_param;
	//curve_data.interior_weight_function.push_back(true);
	//ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);

	//// curve fitting
	//pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
	//// curve_fit.setQuiet (false); // enable/disable debug output
	//curve_fit.fitting(curve_params);
	//visualizeCurve(curve_fit.m_nurbs, fit.m_nurbs, viewer);

	//// ############################################################################
	//// triangulation of trimmed surface

	//printf("  triangulate trimmed surface ...\n");
	//viewer.removePolygonMesh(mesh_id);
	//pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh,
	//	mesh_resolution);
	//viewer.addPolygonMesh(mesh, mesh_id);


	//// save trimmed B-spline surface
	//if (fit.m_nurbs.IsValid()) {
	//	ONX_Model model;
	//	ONX_Model_Object& surf = model.m_object_table.AppendNew();
	//	surf.m_object = new ON_NurbsSurface(fit.m_nurbs);
	//	surf.m_bDeleteObject = true;
	//	surf.m_attributes.m_layer_index = 1;
	//	surf.m_attributes.m_name = "surface";

	//	ONX_Model_Object& curv = model.m_object_table.AppendNew();
	//	curv.m_object = new ON_NurbsCurve(curve_fit.m_nurbs);
	//	curv.m_bDeleteObject = true;
	//	curv.m_attributes.m_layer_index = 2;
	//	curv.m_attributes.m_name = "trimming curve";
	//	std::string file_3dm = "Output.ply";
	//	model.Write(file_3dm.c_str());
	//	printf("  model saved: %s\n", file_3dm.c_str());
	//}

	//printf("  ... done.\n");

	//viewer.spin();
}