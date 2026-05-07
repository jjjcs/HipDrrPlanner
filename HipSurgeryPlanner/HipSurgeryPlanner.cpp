#pragma once
#include "ImageIO.h";
#include "itkImage.h"
#include "itkImageDuplicator.h"
#include "itkRandomImageSource.h"
#include "itkMultiplyImageFilter.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkOtsuThresholdImageFilter.h"
#include "itkOtsuThresholdCalculator.h"
#include "itkExtractImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include <vtkPolyData.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkPointLocator.h>
#include <vtkLandmarkTransform.h>
#include <vtkPolyDataMapper.h>
#include <vtkMatrix4x4.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkActor.h>
#include <vtkSTLReader.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkNamedColors.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkGlyph3DMapper.h>
#include <vtkSphereSource.h>
#include <vtkCellData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkStripper.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataNormals.h>
#include <vtkMassProperties.h>
#include <vtkClipPolyData.h>
#include <vtkCenterOfMass.h>

#include <itkeigen/Eigen/Dense>

#include "vtk_tools.h"
#include "HipSurgeryPlanner.h"
//using namespace ImgIO;

CTImageType::Pointer HipPlanner::binarized_masked_image(CTImageType::Pointer in_ct_image, CTPixelType threshold, CTPixelType constantValue)
{
	//将图像中所有大于threshold的像素替换为constantValue，这里将mask的255转换为1
	using BinaryThresholdFilterType = itk::BinaryThresholdImageFilter<CTImageType, CTImageType>;
	BinaryThresholdFilterType::Pointer thresholdFilter = BinaryThresholdFilterType::New();
	thresholdFilter->SetInput(in_ct_image);
	thresholdFilter->SetLowerThreshold(threshold);
	thresholdFilter->SetInsideValue(constantValue);
	thresholdFilter->SetOutsideValue(0);

	thresholdFilter->Update();

	// 获取输出图像
	CTImageType::Pointer outputImage = thresholdFilter->GetOutput();
	//write_test_result(outputImage);
	
	return outputImage;
}

void HipPlanner::write_test_result(CTImageType::Pointer ct_image)
{
	using WriterType = itk::ImageFileWriter<CTImageType>;
	using ImageIOType = itk::NiftiImageIO;

	WriterType::Pointer writer = WriterType::New();
	ImageIOType::Pointer niiIO = ImageIOType::New();

	writer->SetInput(ct_image);
	writer->SetFileName("./test_out_data.nii.gz");
	writer->UseCompressionOn();
	writer->SetImageIO(niiIO);
	writer->Update();
}

CTImageType::Pointer HipPlanner::cal_masked_image(CTImageType::Pointer image_mask, CTImageType::Pointer ctimage)
{
	//spacing会有精度引起的误差，MultiplyImageFilter要求两个输入spacing严格相同
	CTImageType::SpacingType target_spacing = image_mask->GetSpacing();
	ctimage->SetSpacing(target_spacing);

	using FilterType = itk::MultiplyImageFilter<CTImageType, CTImageType, CTImageType>;
	auto filter = FilterType::New();
	filter->SetInput1(image_mask);
	filter->SetInput2(ctimage);
	filter->Update();
	//write_test_result(filter->GetOutput());
	return filter->GetOutput();
}


CTImageType::Pointer copy_itk_image(CTImageType::Pointer ct_image)
{
	CTImageType::Pointer image = ct_image;
	
	using DuplicatorType = itk::ImageDuplicator<CTImageType>;
	auto duplicator = DuplicatorType::New();
	duplicator->SetInputImage(image);
	duplicator->Update();

	CTImageType::Pointer clonedImage = duplicator->GetOutput();

	return clonedImage;
}

void HipPlanner::pixel_iter(CTImageType::Pointer ct_image, CTImageType::IndexType& top_point_index, CTImageType::IndexType& bottom_point_index)
{
	// 使用 ImageRegionIterator 遍历图像
	using IteratorType = itk::ImageRegionIterator<CTImageType>;
	IteratorType it(ct_image, ct_image->GetLargestPossibleRegion());

	//CTImageType::IndexType top_point_index;
	//CTImageType::IndexType bottom_point_index;

	int max_z_coordinate = 0;
	int min_z_coordinate = 10000;
	for (it.GoToBegin(); !it.IsAtEnd(); ++it) {
		if (it.Get() != 0) {
			if (it.GetIndex()[2] > max_z_coordinate)
			{
				max_z_coordinate = it.GetIndex()[2];
				top_point_index = it.GetIndex();
			}
			if (it.GetIndex()[2] < min_z_coordinate)
			{
				min_z_coordinate = it.GetIndex()[2];
				bottom_point_index = it.GetIndex();
			}
		}
	}
}

CTImageType::Pointer HipPlanner::ct_slice(CTImageType::Pointer ct_image, int slice_idx)
{
	CTImageType::IndexType index;
	index[2] = slice_idx; // 选择第10层

	using ExtractFilterType = itk::ExtractImageFilter<CTImageType, CTImageType>;
	ExtractFilterType::Pointer extractFilter = ExtractFilterType::New();
	extractFilter->SetInput(ct_image);

	// 设置要提取的切片的区域
	CTImageType::RegionType region = ct_image->GetLargestPossibleRegion();
	region.SetIndex(2, index[2]); // 设置索引
	region.SetSize(2, 1); // 设置大小，这里设为0表示只取一个切片
	extractFilter->SetExtractionRegion(region);
	
	// 执行提取滤波器
	extractFilter->Update();

	// 获取提取的图像
	CTImageType::Pointer outputImage = extractFilter->GetOutput();
	//write_test_result(outputImage);
	return outputImage;
}

CTImageType::Pointer HipPlanner::extract_femur_cavity(CTImageType::Pointer in_ct_slice)
{
	using FilterType = itk::OtsuThresholdImageFilter<CTImageType, CTImageType>;
	FilterType::Pointer Otsufilter = FilterType::New();
	Otsufilter->SetInput(in_ct_slice);
	Otsufilter->SetInsideValue(0);
	Otsufilter->SetOutsideValue(1);

	Otsufilter->Update();
	CTImageType::Pointer outputImage = Otsufilter->GetOutput();
	//write_test_result(outputImage);
	return outputImage;
}


Eigen::Vector3d HipPlanner::extract_femur_cavity_center(CTImageType::Pointer in_ct_slice)
{
	unsigned int bone_pixel_num = 0;
	CTImageType::IndexType centerIndex;
	float x_coordinate = 0;
	float y_coordinate = 0;

	using IteratorType = itk::ImageRegionIterator<CTImageType>;
	IteratorType it(in_ct_slice, in_ct_slice->GetLargestPossibleRegion());
	int z_coordinate = it.GetIndex()[2];
	for (it.GoToBegin(); !it.IsAtEnd(); ++it) 
	{
		if (it.Get() != 0) 
		{
			CTImageType::IndexType currentIndex = it.GetIndex();
			x_coordinate += currentIndex[0];
			y_coordinate += currentIndex[1];
			bone_pixel_num++;
		}
	}
	if (bone_pixel_num > 0)
	{
		x_coordinate /= bone_pixel_num;
		y_coordinate /= bone_pixel_num;
	}

	using ContinuousIndexType = itk::ContinuousIndex<float, 3>;
	ContinuousIndexType continuousIndex;
	continuousIndex[0] = x_coordinate;
	continuousIndex[1] = y_coordinate;
	continuousIndex[2] = z_coordinate;
	
	PointType physicalPoint;
	in_ct_slice->TransformContinuousIndexToPhysicalPoint(continuousIndex, physicalPoint);
	Eigen::Vector3d extracted_point;
	extracted_point << physicalPoint[0], physicalPoint[1], physicalPoint[2];
	return extracted_point;
}

void HipPlanner::set_femur_shaft_landmark_points(Eigen::Vector3d OPERATIVE_INTERCONDYLAR_FOSSA, Eigen::Vector3d OPERATIVE_BIG_TROCHANTER_FEATURES)
{
	//CONTRALATERAL_INTERCONDYLAR_FOSSA,			//髁间窝
	//CONTRALATERAL_BIG_TROCHANTER_FEATURES,		 //大转子
	m_OPERATIVE_INTERCONDYLAR_FOSSA = OPERATIVE_INTERCONDYLAR_FOSSA;
	m_OPERATIVE_BIG_TROCHANTER_FEATURES = OPERATIVE_BIG_TROCHANTER_FEATURES;
	//m_OPERATIVE_FEMORAL_FEATURE_3 = OPERATIVE_FEMORAL_FEATURE_3;

	m_OPERATIVE_INTERCONDYLAR_FOSSA_pyhsicalpoint[0] = OPERATIVE_INTERCONDYLAR_FOSSA[0];
	m_OPERATIVE_INTERCONDYLAR_FOSSA_pyhsicalpoint[1] = OPERATIVE_INTERCONDYLAR_FOSSA[1];
	m_OPERATIVE_INTERCONDYLAR_FOSSA_pyhsicalpoint[2] = OPERATIVE_INTERCONDYLAR_FOSSA[2];

	m_OPERATIVE_BIG_TROCHANTER_FEATURES_pyhsicalpoint[0] = OPERATIVE_BIG_TROCHANTER_FEATURES[0];
	m_OPERATIVE_BIG_TROCHANTER_FEATURES_pyhsicalpoint[1] = OPERATIVE_BIG_TROCHANTER_FEATURES[1];
	m_OPERATIVE_BIG_TROCHANTER_FEATURES_pyhsicalpoint[2] = OPERATIVE_BIG_TROCHANTER_FEATURES[2];

	ct_image->TransformPhysicalPointToIndex(m_OPERATIVE_INTERCONDYLAR_FOSSA_pyhsicalpoint, m_OPERATIVE_INTERCONDYLAR_FOSSA_idx);
	ct_image->TransformPhysicalPointToIndex(m_OPERATIVE_BIG_TROCHANTER_FEATURES_pyhsicalpoint, m_OPERATIVE_BIG_TROCHANTER_FEATURES_idx);

	m_target_z_slice0 = m_OPERATIVE_INTERCONDYLAR_FOSSA_idx[2] + (m_OPERATIVE_BIG_TROCHANTER_FEATURES_idx[2] - m_OPERATIVE_INTERCONDYLAR_FOSSA_idx[2])*0.6;  //股骨z方向长度取0.6
	m_target_z_slice1 = m_OPERATIVE_INTERCONDYLAR_FOSSA_idx[2] + (m_OPERATIVE_BIG_TROCHANTER_FEATURES_idx[2] - m_OPERATIVE_INTERCONDYLAR_FOSSA_idx[2])*0.75;
}

void  HipPlanner::run_femur_shaft_planning(string surgery_side)
{
	//clock_t start, finish;  // clock_t为时钟计时单元数
	//start = clock();
	//std::cout << "start" << std::endl;

	if (surgery_side == "left")
	{
		m_target_femur_mask = m_left_femur_mask;
	}
	else
	{
		m_target_femur_mask = m_right_femur_mask;
	}

	set_one_pixel_value_target_mask(binarized_masked_image(m_target_femur_mask));  //将255的mask变成1的像素值
	CTImageType::Pointer one_pixel_value_mask = cal_masked_image(ct_image, one_pixel_value_target_mask);  //两个图像矩阵相乘
	set_target_feumr_masked_image(one_pixel_value_mask);

	CTImageType::Pointer slice_top = ct_slice(target_feumr_masked_image, m_target_z_slice0);
	CTImageType::Pointer slice_cavity = extract_femur_cavity(slice_top);
	m_femur_shaft_p0 = extract_femur_cavity_center(slice_cavity);

	CTImageType::Pointer slice_top1 = ct_slice(target_feumr_masked_image, m_target_z_slice1);
	CTImageType::Pointer slice_cavity1 = extract_femur_cavity(slice_top1);
	m_femur_shaft_p1 = extract_femur_cavity_center(slice_cavity1);

	//Eigen::Vector3d normal = m_femur_shaft_p1 - m_femur_shaft_p0;
	//normal = normal.normalized();
	//std::cout << normal << std::endl;
	//m_femur_shaft_p1 = m_femur_shaft_p0 + normal * 200;


//	finish = clock();
//	std::cout << "代码运行花费时间为：" << double(finish - start) / CLOCKS_PER_SEC << "s" << std::endl;  //时间计算过程
}

vtkSmartPointer<vtkPolyData> HipPlanner::load_STL(std::string STL_FILE_PATH)
{
	vtkSmartPointer<vtkPolyData> polyData;

	vtkNew<vtkSTLReader> reader;
	reader->SetFileName(STL_FILE_PATH.c_str());
	reader->Update();
	polyData = reader->GetOutput();

	return polyData;
}

void HipPlanner::set_femur_neck_landmark_points(Eigen::Vector3d OPERATIVE_BIG_TROCHANTER_FEATURES, Eigen::Vector3d OPERATIVE_LESSER_TROCHANTER_FEATURES, Eigen::Vector3d OPERATIVE_FEMORAL_FEATURE_3)
{
	m_OPERATIVE_BIG_TROCHANTER_FEATURES = OPERATIVE_BIG_TROCHANTER_FEATURES;
	m_OPERATIVE_LESSER_TROCHANTER_FEATURES = OPERATIVE_LESSER_TROCHANTER_FEATURES;
	m_OPERATIVE_FEMORAL_FEATURE_3 = OPERATIVE_FEMORAL_FEATURE_3;
}


void HipPlanner::template_registration()
{
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkPoints> target_points;
	
	double p0[3] = {m_OPERATIVE_BIG_TROCHANTER_FEATURES[0], m_OPERATIVE_BIG_TROCHANTER_FEATURES[1], m_OPERATIVE_BIG_TROCHANTER_FEATURES[2]};
	double p1[3] = {m_OPERATIVE_LESSER_TROCHANTER_FEATURES[0], m_OPERATIVE_LESSER_TROCHANTER_FEATURES[1], m_OPERATIVE_LESSER_TROCHANTER_FEATURES[2] };
	double p2[3] = {m_OPERATIVE_FEMORAL_FEATURE_3[0], m_OPERATIVE_FEMORAL_FEATURE_3[1], m_OPERATIVE_FEMORAL_FEATURE_3[2]};
	target_points->InsertNextPoint(p0);
	target_points->InsertNextPoint(p1);
	target_points->InsertNextPoint(p2);

	vtkNew<vtkLandmarkTransform> landmarkTransform;
	landmarkTransform->SetSourceLandmarks(m_template_points);
	landmarkTransform->SetTargetLandmarks(target_points);
	landmarkTransform->SetModeToRigidBody();
	landmarkTransform->Update(); // should this be here?

	vtkNew<vtkPolyData> source;
	source->SetPoints(m_template_points);

	vtkNew<vtkPolyData> target;
	target->SetPoints(target_points);

	vtkNew<vtkTransformPolyDataFilter> transformFilter;
	transformFilter->SetInputData(source);
	transformFilter->SetTransform(landmarkTransform);
	transformFilter->Update();

	//vtkPolyData* transformed_source = transformFilter->GetOutput();
	//vtkSmartPointer<vtkPoints> x = transformed_source->GetPoints();
	//double pointx[3];
	//x->GetPoint(0, pointx);
	//std::cout << pointx[0] << std::endl;
	//std::cout << pointx[1] << std::endl;
	//std::cout << pointx[2] << std::endl;
	
	Eigen::Matrix4d transform_matrix;
	transform_matrix = static_cast<Eigen::Matrix4d>(landmarkTransform->GetMatrix()->GetData());
	transform_matrix.transposeInPlace();
	m_transform_matrix = transform_matrix;
	transform_matrix(0, 3) = 0;
	transform_matrix(1, 3) = 0;
	transform_matrix(2, 3) = 0;
	m_rotation_matrix = transform_matrix;
	// Display the transformation matrix that was computed
	vtkMatrix4x4* mat = landmarkTransform->GetMatrix();
	std::cout << "template matched matrix: " << transform_matrix << std::endl;

	m_femur_neck_template_2_target_transform = landmarkTransform;
	m_femur_neck_template_2_target_matrix = mat;
}

Eigen::Vector3d HipPlanner::transform_point(Eigen::Vector3d point)
{
	Eigen:;Vector3d transfered_point =  Eigen::Isometry3d(m_transform_matrix) * point;
	return transfered_point;
}

Eigen::Vector3d HipPlanner::transfer_normal(Eigen::Vector3d normal)
{
	Eigen:; Vector3d rotated_normal = Eigen::Isometry3d(m_rotation_matrix) * normal;
	return rotated_normal;
}

void HipPlanner::clip_femur_polydata()
{
	vtkNew<vtkPlane> cut_plane;
	cut_plane->SetOrigin(m_info_transfered.femur_neck_cutplane_center[0], m_info_transfered.femur_neck_cutplane_center[1], m_info_transfered.femur_neck_cutplane_center[2]);
	cut_plane->SetNormal(m_info_transfered.femur_neck_cutplane_normal[0], m_info_transfered.femur_neck_cutplane_normal[1], m_info_transfered.femur_neck_cutplane_normal[2]);

	vtkNew<vtkClipPolyData> clipper;
	clipper->SetClipFunction(cut_plane);
	clipper->SetInputData(m_target_femur_polydata);
	clipper->Update();

	m_clipped_femur_polydata = clipper->GetOutput();

	//vtkNew<vtkActor> boundary_actor;
	//vtkNew<vtkPolyDataMapper> boundary_mapper;
	//boundary_mapper->SetInputData(m_clipped_femur_polydata);
	//boundary_actor->SetMapper(boundary_mapper);
	//std::vector<vtkActor*> all_actors;
	//all_actors.push_back(boundary_actor);
	//show_actors(all_actors);
}

void HipPlanner::transfer_template()
{
	m_info_transfered.femur_neck_center = Eigen::Isometry3d(m_transform_matrix)*m_info_original.femur_neck_center;
	m_info_transfered.femur_neck_cutplane_center = Eigen::Isometry3d(m_transform_matrix)*m_info_original.femur_neck_cutplane_center;
	m_info_transfered.femur_neck_cutplane_normal = Eigen::Isometry3d(m_rotation_matrix)*m_info_original.femur_neck_cutplane_normal;
	m_info_transfered.femur_neck_axisx = Eigen::Isometry3d(m_rotation_matrix)*m_info_original.femur_neck_axisx;
	m_info_transfered.femur_neck_axisy = Eigen::Isometry3d(m_rotation_matrix)*m_info_original.femur_neck_axisy;
	m_info_transfered.femur_neck_axisz = Eigen::Isometry3d(m_rotation_matrix)*m_info_original.femur_neck_axisz;
}

Eigen::Vector3d HipPlanner::rotate_axis(Eigen::Vector3d target_axis, Eigen::Vector3d rotation_axis, float angle)
{
	vtkNew<vtkTransform> transform;
	float axis[3] = { rotation_axis[0],rotation_axis[1],rotation_axis[2] };
	transform->RotateWXYZ(angle, axis);
	float* transfomed_result = transform->TransformFloatNormal(target_axis[0], target_axis[1], target_axis[2]);
	Eigen::Vector3d transformed_axis = { transfomed_result[0], transfomed_result[1], transfomed_result[2]};
	return transformed_axis;
}

vtkSmartPointer<vtkPolyData> HipPlanner::get_cut_plane(vtkSmartPointer<vtkPolyData> target_polydata, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal)
{
	vtkNew<vtkPlane> cut_plane;
	cut_plane->SetOrigin(plane_center[0], plane_center[1], plane_center[2]);
	cut_plane->SetNormal(plane_normal[0], plane_normal[1], plane_normal[2]);
	
	vtkNew<vtkCutter> cutter;
	cutter->SetInputData(target_polydata);
	cutter->SetCutFunction(cut_plane);
	cutter->GenerateTrianglesOn();
	cutter->Update();

	vtkNew<vtkStripper> stripper;
	stripper->SetInputData(cutter->GetOutput());
	stripper->JoinContiguousSegmentsOn();
	stripper->Update();

	vtkNew<vtkPolyData> cutted_boundary;
	//vtkSmartPointer<vtkPoints> points = stripper->GetOutput()->GetPoints();
	//std::cout << points->GetNumberOfPoints() << std::endl;
	cutted_boundary->SetPoints(stripper->GetOutput()->GetPoints());
	cutted_boundary->SetPolys(stripper->GetOutput()->GetLines());

	//vtkNew<vtkActor> boundary_actor;
	//vtkNew<vtkPolyDataMapper> boundary_mapper;
	//boundary_mapper->SetInputData(cutted_boundary);
	//boundary_actor->SetMapper(boundary_mapper);
	//std::vector<vtkActor*> all_actors;
	//all_actors.push_back(boundary_actor);
	//show_actors(all_actors);

	return cutted_boundary;
}

float HipPlanner::cal_area_volume(vtkSmartPointer<vtkPolyData> plane_polydata, double center[], int stage)
{
	int num_cells = plane_polydata->GetNumberOfCells();

	if (num_cells != 0) 
	{
		if (stage == 1)
		{
			vtkNew<vtkCenterOfMass> COM_filter;
			COM_filter->SetInputData(plane_polydata);
			COM_filter->Update();
			double* center_tmp = COM_filter->GetCenter();

			center[0] = center_tmp[0];
			center[1] = center_tmp[1];
			center[2] = center_tmp[2];	
		}

		vtkNew<vtkTriangleFilter> TriangleFilter;
		TriangleFilter->SetInputData(plane_polydata);
		TriangleFilter->Update();

		vtkNew<vtkPolyDataNormals> normal_calculator;
		normal_calculator->SetInputConnection(TriangleFilter->GetOutputPort());
		normal_calculator->ConsistencyOn();
		normal_calculator->SplittingOff();
		normal_calculator->Update();
	
		vtkNew<vtkMassProperties> property_calculator;
		property_calculator->SetInputData(normal_calculator->GetOutput());
		property_calculator->Update();

		return property_calculator->GetSurfaceArea();
	}
	else
	{
		center[0] = m_info_transfered.femur_neck_center[0];
		center[1] = m_info_transfered.femur_neck_center[1];
		center[2] = m_info_transfered.femur_neck_center[2];
		return 10000;
	}
}

void HipPlanner::search_femur_neck()
{
	vtkNew<vtkNamedColors> colors;
	std::vector<vtkActor*> all_actors;

	std::vector<Eigen::Vector3d> axisy_list;

	int best_cutplane_idx_stage0 = 0;
	float min_area_stage0 = 10000;

	for (int current_step=0; current_step< m_num_iterations; current_step++)
	{
		Eigen::Vector3d rotated_zaxis = 
		rotate_axis(m_info_transfered.femur_neck_axisz, m_info_transfered.femur_neck_axisy, m_step_degree*(current_step-m_num_iterations/2));
		axisy_list.push_back(rotated_zaxis);
		vtkSmartPointer<vtkPolyData> cut_plane = get_cut_plane(m_target_femur_polydata, m_info_transfered.femur_neck_center, rotated_zaxis);
		
		double center[3];
		float area = cal_area_volume(cut_plane, center);
		if (area < min_area_stage0)
		{
			min_area_stage0 = area;
			best_cutplane_idx_stage0 = current_step;
		}
	}

	set_stage0_zaxis(axisy_list.at(best_cutplane_idx_stage0));

	std::vector<Eigen::Vector3d> axisx_list;
	std::vector<double*> cutplane_center;

	int best_cutplane_idx_stage1 = 0;
	float min_area_stage1 = 10000;

	for (int current_step=0; current_step<m_num_iterations; current_step++)
	{
		Eigen::Vector3d rotated_zaxis =
			rotate_axis(m_stage0_femur_neck_axisz, m_info_transfered.femur_neck_axisx, m_step_degree*(current_step - m_num_iterations / 2)); //从stage0得出的结果计算
		axisx_list.push_back(rotated_zaxis);
		vtkSmartPointer<vtkPolyData> cut_plane = get_cut_plane(m_clipped_femur_polydata, m_info_transfered.femur_neck_center, rotated_zaxis);

		double center[3];
		float area = cal_area_volume(cut_plane, center, 1);
		cutplane_center.push_back(center);

		if (area < min_area_stage1)
		{
			min_area_stage1 = area;
			best_cutplane_idx_stage1 = current_step;
		}
	}
	
	double* best_center = cutplane_center.at(best_cutplane_idx_stage1);
	m_res_femur_neck_normal = axisx_list.at(best_cutplane_idx_stage1);
	m_res_femur_neck_center <<best_center[0], best_center[1], best_center[2];
}

void HipPlanner::set_femur_neck_point()
{
	m_femur_neck_p0 = m_res_femur_neck_center + m_res_femur_neck_normal * 50;
	m_femur_neck_p1 = m_res_femur_neck_center - m_res_femur_neck_normal * 50;
}

void HipPlanner::run_femur_neck_planning(string surgery_side)
{
	if (surgery_side == "left")
	{
		init_left_template();
	}
	else
	{
		init_right_template();
	}

	template_registration();
	transfer_template();
	clip_femur_polydata();
	search_femur_neck();
	set_femur_neck_point();
	//render_result();
}

