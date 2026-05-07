#include <itkeigen/Eigen/Core>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkImageData.h>
#include <vtkBox.h>
#include <vtkActor.h>
#include <vtkImageData.h>
#include<string>
#include<time.h>

#include <vtkBoxWidget2.h>
#include <vtkConeSource.h>
#include <vtkOBBTree.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkDistancePolyDataFilter.h>
#include <vtkScalarBarActor.h>
#include <vtkCleanPolyData.h>
#include <vtkIntersectionPolyDataFilter.h>
#include <vtkBooleanOperationPolyDataFilter.h>
#include <vtkKdTree.h>
#include <vtkPoints.h>

#include "ImageIO.h"
#include "vtk_tools.h"
#include "HipSurgeryPlanner.h"
#include "DRRImageProcess.h"
#include "HipDrrPlanner.h"
#include "polydata_dist_calculator.h"
#include "Coverage_Calculator.h"
#include <thread>
#include "future"
#include "itkBinaryThresholdImageFilter.h"
#include <iostream>
void init_plane(vector<Eigen::Vector3d>& landmark_points, vector<string>& data_path, int case_num = 0, string surgery_side = "left")
{
	if (case_num == 0)
	{
		if (surgery_side=="left")
		{
			//髁间窝，大转子，小转子，股骨特征点3的顺序
			Eigen::Vector3d INTERCONDYLAR_FOSSA(59.6594, 20.1816, -587.8555);  //髁间窝
			Eigen::Vector3d BIG_TROCHANTER_FEATURES(130.6474, 20.5513, -172.4124);  //大转子
			Eigen::Vector3d LESSER_TROCHANTER_FEATURES(99.5763, 41.2269, -219.3709);  //小转子
			Eigen::Vector3d FEMORAL_FEATURE_3(63.7718, 17.6701, -173.754);  //股骨特征点3

			landmark_points.push_back(INTERCONDYLAR_FOSSA);
			landmark_points.push_back(BIG_TROCHANTER_FEATURES);
			landmark_points.push_back(LESSER_TROCHANTER_FEATURES);
			landmark_points.push_back(FEMORAL_FEATURE_3);
		}
		else
		{
			Eigen::Vector3d INTERCONDYLAR_FOSSA(-77.324, -35.1558, -595.024);  //髁间窝
			Eigen::Vector3d BIG_TROCHANTER_FEATURES(-144.772, 33.3008, -188.127);  //大转子
			Eigen::Vector3d LESSER_TROCHANTER_FEATURES(-114.724, 34.2589, -231.078);
			Eigen::Vector3d FEMORAL_FEATURE_3(-84.735, 22.0343, -183.677); //股骨特征点3

			landmark_points.push_back(INTERCONDYLAR_FOSSA);
			landmark_points.push_back(BIG_TROCHANTER_FEATURES);
			landmark_points.push_back(LESSER_TROCHANTER_FEATURES);
			landmark_points.push_back(FEMORAL_FEATURE_3);
		}

		//左右polydata，dicom，左右股骨mask的nrrd
		data_path.push_back("D:\\data\\hip\\hospital\\doctor\\40701339_R\\projectfiles\\bone1.stl");  //40701339_R对应HD0007
		data_path.push_back("D:\\data\\hip\\hospital\\doctor\\40701339_R\\projectfiles\\bone2.stl");
		data_path.push_back("C:/fake_usb/HD0007/DICOM");
		data_path.push_back("D:/DeepHip_data/HD0007/label/Femur_L.nrrd");
		data_path.push_back("D:/DeepHip_data/HD0007/label/Femur_R.nrrd");
	}
	else if (case_num == 1)
	{
		if (surgery_side == "left")
		{
			Eigen::Vector3d INTERCONDYLAR_FOSSA(55.1239, -102.547, -663.689);  //髁间窝
			Eigen::Vector3d BIG_TROCHANTER_FEATURES(138.19, -95.0798, -211.366);  //大转子
			Eigen::Vector3d LESSER_TROCHANTER_FEATURES(98.8736, -97.4452, -258.022);  //小转子
			Eigen::Vector3d FEMORAL_FEATURE_3(76.5275, -121.581, -202.906);  //股骨特征点3

			//Eigen::Vector3d INTERCONDYLAR_FOSSA(0, 0, 0);  //髁间窝
			//Eigen::Vector3d BIG_TROCHANTER_FEATURES(0, 0, 0);  //大转子
			//Eigen::Vector3d LESSER_TROCHANTER_FEATURES(0, 0, 0);  //小转子
			//Eigen::Vector3d FEMORAL_FEATURE_3(0,0, 0);  //股骨特征点3
		
			landmark_points.push_back(INTERCONDYLAR_FOSSA);
			landmark_points.push_back(BIG_TROCHANTER_FEATURES);
			landmark_points.push_back(LESSER_TROCHANTER_FEATURES);
			landmark_points.push_back(FEMORAL_FEATURE_3);
		}
		else
		{
			Eigen::Vector3d INTERCONDYLAR_FOSSA(-64.1802, -96.3522, -663.759);  //髁间窝
			Eigen::Vector3d BIG_TROCHANTER_FEATURES(-136.327, -94.4546, -211.322);  //大转子
			Eigen::Vector3d LESSER_TROCHANTER_FEATURES(-99.1393, -92.7726, -260.944);  //小转子
			Eigen::Vector3d FEMORAL_FEATURE_3(-73.9199, -121.895, -194.191);  //股骨特征点3

			landmark_points.push_back(INTERCONDYLAR_FOSSA);
			landmark_points.push_back(BIG_TROCHANTER_FEATURES);
			landmark_points.push_back(LESSER_TROCHANTER_FEATURES);
			landmark_points.push_back(FEMORAL_FEATURE_3);
		}
		data_path.push_back("D:\\data\\hip\\hospital\\doctor\\46470156_L\\projectfiles\\bone1.stl");  //46470156_L对应HD0008
		data_path.push_back("D:\\data\\hip\\hospital\\doctor\\46470156_L\\projectfiles\\bone2.stl");
		data_path.push_back("D:\\data\\hip\\hospital\\doctor\\46470156_L\\DICOM");
		data_path.push_back("C:\\fake_usb\\HD0008\\label\\Femur_L.nrrd");
		data_path.push_back("C:\\fake_usb\\HD0008\\label\\Femur_R.nrrd");
	}
}

void test_surface_cutter()
{	
	Coverage_Calculator myCoverageCalculator;

	string hip_polydata_path = "D:\\data\\hip\\hospital\\doctor\\473010_R\\projectfiles\\bone4.stl";
	string  femur_cup_polydata_path  = "D:\\data\\hip\\hospital\\doctor\\473010_R\\projectfiles\\transformStl\\solid_cup.stl";
	string intersection_polydata_path = "D:\\data\\hip\\hospital\\doctor\\473010_R\\projectfiles\\transformStl\\intersection.stl";

	MyImageIO MyIO;
	vtkSmartPointer<vtkPolyData> intersection_polydata = MyIO.load_STL(intersection_polydata_path);
	vtkSmartPointer<vtkPolyData> femur_cup_polydata = MyIO.load_STL(femur_cup_polydata_path);
	vtkSmartPointer<vtkPolyData> hip_polydata = MyIO.load_STL(hip_polydata_path);

	myCoverageCalculator.set_cup_polydata(femur_cup_polydata);
	myCoverageCalculator.set_intersection_polydata(intersection_polydata);
	myCoverageCalculator.set_hip_polydata(hip_polydata);
	myCoverageCalculator.set_cup_sphere_radius(38);
	myCoverageCalculator.set_cup_sphere_center(-96, -17, 363);
	myCoverageCalculator.set_cup_z_axis(0.097982, 0.05588, -0.993617); //-0.097982, -0.05588, 0.993617

	time_t start_t = clock();
	double coverage_rate = myCoverageCalculator.cal_coverage();
	time_t finish_t = clock();
	cout << "cal coverage rate takes : " << (double)(finish_t - start_t) / CLOCKS_PER_SEC << " s" << endl;
	std::cout << coverage_rate << std::endl;
}

void test_2d_render()
{
	Coverage_Calculator myCoverageCalculator;

	string hip_polydata_path = "D:\\data\\hip\\hospital\\doctor\\473010_R\\projectfiles\\bone4.stl";
	string  femur_cup_polydata_path = "D:\\data\\hip\\hospital\\doctor\\473010_R\\projectfiles\\transformStl\\solid_cup.stl";
	string intersection_polydata_path = "D:\\data\\hip\\hospital\\doctor\\473010_R\\projectfiles\\transformStl\\intersection.stl";

	MyImageIO MyIO;
	vtkSmartPointer<vtkPolyData> intersection_polydata = MyIO.load_STL(intersection_polydata_path);
	vtkSmartPointer<vtkPolyData> femur_cup_polydata = MyIO.load_STL(femur_cup_polydata_path);
	vtkSmartPointer<vtkPolyData> hip_polydata = MyIO.load_STL(hip_polydata_path);

	myCoverageCalculator.set_cup_polydata(femur_cup_polydata);
	myCoverageCalculator.set_intersection_polydata(intersection_polydata);
	myCoverageCalculator.set_hip_polydata(hip_polydata);

	myCoverageCalculator.set_cup_sphere_radius(26.25);
	myCoverageCalculator.set_cup_sphere_center(-85.0832, -12.6899, 381.737);
	myCoverageCalculator.set_cup_z_axis(0.682546, 0.388327, 0.619139);

	time_t start_t = clock();
	myCoverageCalculator.render_projection();
	//myCoverageCalculator.cal_coverage();
	time_t finish_t = clock();
	cout << "cal coverage rate takes : " << (double)(finish_t - start_t) / CLOCKS_PER_SEC << " s" << endl;
}

void run_simple_planner_example()
{
}

void run_drr_planner() 
{
	vtkNew<vtkNamedColors> colors;
	bool reset_mode = true;
	vtkNew<vtkTransform> transform;
	vtkNew<vtkTransform> inv_left_transform;
	vtkNew<vtkTransform> inv_right_transform;
	vtkNew<vtkTransform> left_transform;
	HipDrrPlanner MyHipDrrPlanner;
	if (reset_mode)
	{
		Eigen::Matrix4d left_femur_reset_matrix;
		left_femur_reset_matrix << 0.766175, -0.636221, -0.0905493, 11.8764,
			0.594827, 0.755433, -0.274774, -95.9743,
			0.243221, 0.156664, 0.957236, -27.59,
			0, 0, 0, 1;
		//left_femur_reset_matrix << 0.987256, -0.0469383, 0.15206, 110.376,
		//	0.0786949, 0.974504, -0.210117, -143.634,
		//	-0.138321, 0.219406, 0.965779, -39.6323,
		//	0, 0, 0, 1;
		Eigen::Matrix4d inv_left_femur_reset_matrix = left_femur_reset_matrix.inverse();
		Eigen::Matrix4d right_femur_reset_matrix;
		right_femur_reset_matrix << 0.991798, 0.111076, 0.0632342, 6.4651,
			-0.0750849, 0.906706, -0.415026, -89.7208,
			-0.103434, 0.406874, 0.90761, -34.39411,
			0, 0, 0, 1;
		//right_femur_reset_matrix << 0.996239, 0.0791004, 0.03537, 29.8868,
		//	-0.0714926, 0.98102, -0.180246, -131.505,
		//	-0.0489563, 0.17704,   0.982985, -19.2564,
		//	0, 0,        0, 1;
		MyHipDrrPlanner.set_left_femur_reset_matrix(left_femur_reset_matrix);
		MyHipDrrPlanner.set_right_femur_reset_matrix(right_femur_reset_matrix);
		Eigen::Matrix4d inv_right_femur_reset_matrix = right_femur_reset_matrix.inverse();
		vtkNew<vtkMatrix4x4> m_right_femur_reset_matrix_vtk;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				m_right_femur_reset_matrix_vtk->SetElement(i, j, right_femur_reset_matrix(i, j));
			}
		}

		vtkNew<vtkMatrix4x4> m_left_femur_reset_matrix_vtk;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				m_left_femur_reset_matrix_vtk->SetElement(i, j, left_femur_reset_matrix(i, j));
			}
		}

		transform->SetMatrix(m_right_femur_reset_matrix_vtk);
		left_transform->SetMatrix(m_left_femur_reset_matrix_vtk);
	}

	string dicom_path = "D:\\DeepHip_data\\HD0007\\DICOM";
	//string left_femur_mask_path = "D:\\DeepHip_data\\HD0007\\label\\Femur_L.nrrd";
	//string right_femur_mask_path = "D:\\DeepHip_data\\HD0007\\label\\Femur_R.nrrd";
	//string left_hip_mask_path = "D:\\DeepHip_data\\HD0007\\label\\Hip_L.nrrd";
	//string right_hip_mask_path = "D:\\DeepHip_data\\HD0007\\label\\Hip_R.nrrd";
	//string scarum_mask_path = "D:\\DeepHip_data\\HD0007\\label\\Sacrum.nrrd";
	string left_femur_mask_path = "D:\\DeepHip_data\\HD0007\\label\\LF_bad.nrrd";
	string right_femur_mask_path = "D:\\DeepHip_data\\HD0007\\label\\RF_bad.nrrd";
	string left_hip_mask_path = "D:\\DeepHip_data\\HD0007\\label\\LH_bad.nrrd";
	string right_hip_mask_path = "D:\\DeepHip_data\\HD0007\\label\\RH_bad.nrrd";
	string scarum_mask_path = "D:\\DeepHip_data\\HD0007\\label\\Sacrum.nrrd";
	string hip_cup_polydata_path = "D:\\data\\hip\\hospital\\doctor\\40701339_R\\projectfiles\\transformStl\\cup.stl";
	string femur_stem_polydata_path = "D:\\data\\hip\\hospital\\doctor\\40701339_R\\projectfiles\\transformStl\\handle.stl";
	string femur_head_polydata_path = "D:\\data\\hip\\hospital\\doctor\\40701339_R\\projectfiles\\transformStl\\head.stl";
	string left_femur_polydata_path = "D:\\data\\hip\\hospital\\doctor\\40701339_R\\projectfiles\\bone1.stl";
	string right_femur_polydata_path = "D:\\data\\hip\\hospital\\doctor\\40701339_R\\projectfiles\\bone2.stl";

	//string dicom_path = "D:\\DeepHip_data\\0004226089_L\\DICOM";
	//string left_femur_mask_path = "D:\\DeepHip_data\\0004226089_L\\label\\lf.nrrd";
	//string right_femur_mask_path = "D:\\DeepHip_data\\0004226089_L\\label\\rf.nrrd";
	//string left_hip_mask_path = "D:\\DeepHip_data\\0004226089_L\\label\\lh.nrrd";
	//string right_hip_mask_path = "D:\\DeepHip_data\\0004226089_L\\label\\rh.nrrd";
	//string hip_cup_polydata_path = "D:\\data\\hip\\hospital\\doctor\\0004226089_L\\projectfiles\\transformStl\\cup.stl";
	//string femur_stem_polydata_path = "D:\\data\\hip\\hospital\\doctor\\0004226089_L\\projectfiles\\transformStl\\handle.stl";
	//string femur_head_polydata_path = "D:\\data\\hip\\hospital\\doctor\\0004226089_L\\projectfiles\\transformStl\\head.stl";
	//string left_femur_polydata_path = "D:\\data\\hip\\hospital\\doctor\\0004226089_L\\projectfiles\\bone1.stl";
	//string right_femur_polydata_path = "D:\\data\\hip\\hospital\\doctor\\0004226089_L\\projectfiles\\bone3.stl";

	//string dicom_path = "D:\\DeepHip_data\\HD0040\\DICOM";
	//string left_femur_mask_path = "D:\\DeepHip_data\\HD0040\\predict_LRHip\\LFemur.nrrd";
	//string right_femur_mask_path = "D:\\DeepHip_data\\HD0040\\predict_LRHip\\RFemur.nrrd";
	//string left_hip_mask_path = "D:\\DeepHip_data\\HD0040\\predict_LRHip\\LHip.nrrd";
	//string right_hip_mask_path = "D:\\DeepHip_data\\HD0040\\predict_LRHip\\RHip.nrrd";

	MyImageIO MyIO;
	vtkSmartPointer<vtkPolyData> hip_cup_polydata = MyIO.load_STL(hip_cup_polydata_path);
	vtkNew<vtkActor> hip_cup_actor;
	vtkNew<vtkPolyDataMapper> hip_cup_mapper;
	hip_cup_mapper->SetInputData(hip_cup_polydata);
	hip_cup_actor->SetMapper(hip_cup_mapper);
	hip_cup_actor->GetProperty()->SetColor(0,255,0);

	vtkSmartPointer<vtkPolyData> femur_head_polydata = MyIO.load_STL(femur_head_polydata_path);
	vtkNew<vtkActor> femur_head_actor;
	vtkNew<vtkPolyDataMapper> femur_head_mapper;
	femur_head_mapper->SetInputData(femur_head_polydata);
	femur_head_actor->SetMapper(femur_head_mapper);
	//femur_head_actor->GetProperty()->SetOpacity(0.5);
	femur_head_actor->GetProperty()->SetColor(0, 255, 0);

	vtkSmartPointer<vtkPolyData> femur_stem_polydata = MyIO.load_STL(femur_stem_polydata_path);
	vtkNew<vtkActor> femur_stem_actor;
	vtkNew<vtkPolyDataMapper> femur_stem_mapper;
	femur_stem_mapper->SetInputData(femur_stem_polydata);
	femur_stem_actor->SetMapper(femur_stem_mapper);
	//femur_stem_actor->GetProperty()->SetOpacity(0.5);
	femur_stem_actor->GetProperty()->SetColor(0, 255, 0);

	vtkSmartPointer<vtkPolyData> left_femur_polydata = MyIO.load_STL(left_femur_polydata_path);
	vtkNew<vtkActor> left_femur_actor;
	vtkNew<vtkPolyDataMapper> left_femur_mapper;
	left_femur_mapper->SetInputData(left_femur_polydata);
	left_femur_actor->SetMapper(left_femur_mapper);

	vtkSmartPointer<vtkPolyData> right_femur_polydata = MyIO.load_STL(right_femur_polydata_path);
	vtkNew<vtkActor> right_femur_actor;
	vtkNew<vtkPolyDataMapper> right_femur_mapper;
	right_femur_mapper->SetInputData(right_femur_polydata);
	right_femur_actor->SetMapper(right_femur_mapper);


	MyHipDrrPlanner.set_ct_image(MyIO.readDicomFiles(dicom_path));
//	MyHipDrrPlanner.set_left_femur_mask_image(MyIO.readNrrdMaskFile(left_femur_mask_path));
	MyHipDrrPlanner.set_left_femur_mask_image(nullptr);
	MyHipDrrPlanner.set_right_femur_mask_image(MyIO.readNrrdMaskFile(right_femur_mask_path));
	MyHipDrrPlanner.set_left_hip_mask_image(MyIO.readNrrdMaskFile(left_hip_mask_path));
	MyHipDrrPlanner.set_right_hip_mask_image(MyIO.readNrrdMaskFile(right_hip_mask_path));
	//MyHipDrrPlanner.set_scarum_mask_image(MyIO.readNrrdMaskFile(scarum_mask_path));

	//XRayImageType::Pointer out_img0;
	//XRayImageType::Pointer out_img1;
	//std::future<XRayImageType::Pointer> result = std::async(std::launch::async, binarized_masked_image_thread, MyHipDrrPlanner.m_left_femur_mask);
	//XRayImageType::Pointer res0 = result.get();
	////std::thread thread1(binarized_masked_image_thread, MyHipDrrPlanner.m_left_femur_mask);
	////std::thread thread2(binarized_masked_image_thread, MyHipDrrPlanner.m_right_femur_mask);
	////thread1.join();
	////thread2.join();
	//MyHipDrrPlanner.write_nrrd_test_result(res0, "./out0.nrrd");
//	MyHipDrrPlanner.write_nrrd_test_result(out_img1, "./out1.nrrd");
	//MyHipDrrPlanner.set_left_femur_mask_image(nullptr);
	////MyHipDrrPlanner.set_right_femur_mask_image(nullptr);
	//MyHipDrrPlanner.set_left_hip_mask_image(nullptr);
	//MyHipDrrPlanner.set_right_hip_mask_image(nullptr);
	//MyHipDrrPlanner.set_scarum_mask_image(nullptr);
	//hip_cup_actor->GetProperty()->SetOpacity(0.5);

	//init_drr_planner一定要在init_drr_reset_planner前执行

	//for (int i = 0; i < 10; i++)
	//{
	//	MyHipDrrPlanner.init_drr_planner();
	//	//MyHipDrrPlanner.init_drr_planner_threads();
	//}
	MyHipDrrPlanner.init_drr_planner();
	//MyHipDrrPlanner.init_drr_planner_threads();
	if (reset_mode)
	{
		//部署时不需要，直接从三维界面取摆正后的actor
		femur_stem_actor->SetUserTransform(transform);
		femur_head_actor->SetUserTransform(transform);

		left_femur_actor->SetUserTransform(left_transform);
		right_femur_actor->SetUserTransform(transform);

		//直接取vtk的bbox效率更高，算法内重新计算为itk的bbox
		MyHipDrrPlanner.set_transformed_left_femur_bbox(left_femur_actor->GetBounds());
		MyHipDrrPlanner.set_left_femur_bbox(left_femur_actor->GetMapper()->GetInput()->GetBounds());
		MyHipDrrPlanner.set_transformed_right_femur_bbox(right_femur_actor->GetBounds());
		MyHipDrrPlanner.set_right_femur_bbox(right_femur_actor->GetMapper()->GetInput()->GetBounds());
		MyHipDrrPlanner.init_drr_reset_planner();
	}

	MyHipDrrPlanner.set_hip_cup_actor(hip_cup_actor);
	MyHipDrrPlanner.set_femur_head_actor(femur_head_actor);
	MyHipDrrPlanner.set_femur_stem_actor(femur_stem_actor);

	//MyHipDrrPlanner.set_hip_cup_actor(nullptr);
	//MyHipDrrPlanner.set_femur_head_actor(nullptr);
	//MyHipDrrPlanner.set_femur_stem_actor(nullptr);
	//需要reset时
	/*MyHipDrrPlanner.write_nrrd_test_result(MyHipDrrPlanner.get_whole_masked_image(), "./whole_masked_image.nrrd");
	MyHipDrrPlanner.write_nrrd_test_result(MyHipDrrPlanner.get_whole_reset_masked_image(), "./whole_reset_masked_image.nrrd");*/

	float rx = -90; 
	float ry = 0;
	float rz = 0;
	//MyHipDrrPlanner.run_drr_planner2();
	MyHipDrrPlanner.run_drr_planner(reset_mode);

	//time_t start_t = clock();
	//for (int i = 0; i < 20; i++)
	//{
	//	rx = -90;
	//	ry = 0;
	//	rz = 0;
	//	rx = rx + 2*i - 20;
	//	MyHipDrrPlanner.set_angle(rx, ry, rz);
	//	//MyHipDrrPlanner.run_drr_planner();
	//	MyHipDrrPlanner.run_drr_planner(reset_mode);
	//	std::cout << std::endl;
	//}
	//time_t finish_t = clock();
	//cout << "20 times drr cost " << (double)(finish_t - start_t) / CLOCKS_PER_SEC << " s" << endl;

	////调整假体位置后传入新的polydata，使用run_implant_refresh刷新假体位置
	//MyHipDrrPlanner.run_implant_refresh();
	//render_res = MyHipDrrPlanner.get_render_result();

	//auto writer = vtkSmartPointer<vtkPNGWriter>::New();
	//writer->SetFileName("screem_shot.png");
	//writer->SetInputData(render_res);
	//writer->Write();
}


void run_hip_surgery_palnner()
{
	string surgery_side = "left";  //or "right"

		vector<Eigen::Vector3d> landmark_points;
		vector<string> data_path;
		init_plane(landmark_points, data_path, 1, surgery_side);
		
		std::string left_femur_stl_path = data_path.at(0);
		std::string right_femur_stl_path = data_path.at(1);
		HipPlanner myHipPlanner;

		vtkSmartPointer<vtkPolyData> left_femur_polydata = myHipPlanner.load_STL(left_femur_stl_path);
		vtkSmartPointer<vtkPolyData> right_femur_polydata = myHipPlanner.load_STL(right_femur_stl_path);

		MyImageIO MyIO;
		CTImageType::Pointer left_femur_image = MyIO.readNrrdFile(data_path.at(3));
		myHipPlanner.set_left_femur_mask_image(left_femur_image);
		CTImageType::Pointer right_femur_image = MyIO.readNrrdFile(data_path.at(4));
		myHipPlanner.set_right_femur_mask_image(right_femur_image);
		CTImageType::Pointer ct_image = MyIO.readDicomFiles(data_path.at(2));
		myHipPlanner.set_ct_image(ct_image);

		clock_t start, finish;  // clock_t为时钟计时单元数
		start = clock();
		std::cout << "start" << std::endl;

		myHipPlanner.set_femur_neck_landmark_points(landmark_points.at(1), landmark_points.at(2), landmark_points.at(3));
		myHipPlanner.set_femur_shaft_landmark_points(landmark_points.at(0), landmark_points.at(1));

		myHipPlanner.set_target_left_femur_polydata(left_femur_polydata);
		myHipPlanner.set_target_right_femur_polydata(right_femur_polydata);
		
		myHipPlanner.run_femur_neck_planning(surgery_side);
		myHipPlanner.run_femur_shaft_planning(surgery_side);

		finish = clock();
		std::cout << "代码运行花费时间为：" << double(finish - start) / CLOCKS_PER_SEC << "s" << std::endl;  //时间计算过程

		std::cout << myHipPlanner.m_femur_neck_p0 << std::endl;
		std::cout << myHipPlanner.m_femur_neck_p1 << std::endl;
		std::cout << myHipPlanner.m_femur_shaft_p0 << std::endl;
		std::cout << myHipPlanner.m_femur_shaft_p1 << std::endl;

		//可视化结果																						   
		vtkNew<vtkNamedColors> colors;
		std::vector<vtkActor*> all_actors;

		vtkNew<vtkActor> spine_actor;
		vtkNew<vtkPolyDataMapper> spine_mapper;
		spine_mapper->SetInputData(myHipPlanner.m_target_femur_polydata);
		spine_actor->SetMapper(spine_mapper);
		spine_actor->GetProperty()->SetOpacity(0.5);
		all_actors.push_back(spine_actor);

		vtkNew<vtkActor> line_actor;
		std::vector<Eigen::Vector3d> points;

		points.push_back(myHipPlanner.m_femur_neck_p0);
		points.push_back(myHipPlanner.m_femur_neck_p1);
		//m_res_femur_neck_normal;
		//m_res_femur_neck_center ;
		createLineActor(points, line_actor);
		all_actors.push_back(line_actor);

		vector<Eigen::Vector3d> femur_shaft_points;
		femur_shaft_points.push_back(myHipPlanner.m_femur_shaft_p1);
		femur_shaft_points.push_back(myHipPlanner.m_femur_shaft_p0);
		vtkNew<vtkActor> femur_shaft_line_actor;
		createLineActor(femur_shaft_points, femur_shaft_line_actor);
		all_actors.push_back(femur_shaft_line_actor);

		show_actors(all_actors);
}

void test()
{
	//	test_surface_cutter();
//	test_2d_render();
	run_simple_planner_example();
	//run_drr_planner();
}

//void binarized_masked_image_thread(XRayImageType::Pointer in_ct_image, XRayImageType::Pointer out_ct_image)
//{
//	//将图像中所有大于threshold的像素替换为constantValue，这里将mask的255转换为1
//	CTPixelType threshold = 1;
//	CTPixelType constantValue = 1;
//	using BinaryThresholdFilterType = itk::BinaryThresholdImageFilter<XRayImageType, XRayImageType>;
//	BinaryThresholdFilterType::Pointer thresholdFilter = BinaryThresholdFilterType::New();
//	thresholdFilter->SetInput(in_ct_image);
//	thresholdFilter->SetLowerThreshold(threshold);
//	thresholdFilter->SetInsideValue(constantValue);
//	thresholdFilter->SetOutsideValue(0);
//	thresholdFilter->Update();
//
//	// 获取输出图像
//	out_ct_image = thresholdFilter->GetOutput();
//}
//
//XRayImageType::Pointer binarized_masked_image_thread(XRayImageType::Pointer in_ct_image)
//{
//	//将图像中所有大于threshold的像素替换为constantValue，这里将mask的255转换为1
//	CTPixelType threshold = 1;
//	CTPixelType constantValue = 1;
//	using BinaryThresholdFilterType = itk::BinaryThresholdImageFilter<XRayImageType, XRayImageType>;
//	BinaryThresholdFilterType::Pointer thresholdFilter = BinaryThresholdFilterType::New();
//	thresholdFilter->SetInput(in_ct_image);
//	thresholdFilter->SetLowerThreshold(threshold);
//	thresholdFilter->SetInsideValue(constantValue);
//	thresholdFilter->SetOutsideValue(0);
//	thresholdFilter->Update();
//
//	// 获取输出图像
//	XRayImageType::Pointer out_ct_image = thresholdFilter->GetOutput();
//	return out_ct_image;
//}