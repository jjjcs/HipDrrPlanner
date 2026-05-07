#pragma once
//#include "ImageIO.h"
#include <itkeigen/Eigen/Core>
#include<vector>
#include <vtkLandmarkTransform.h>

using namespace ImgIO;

struct femur_neck_info 
{
	Eigen::Vector3d femur_neck_center;
	Eigen::Vector3d femur_neck_cutplane_center;
	Eigen::Vector3d femur_neck_cutplane_normal;
	Eigen::Vector3d femur_neck_axisx;
	Eigen::Vector3d femur_neck_axisy;
	Eigen::Vector3d femur_neck_axisz;
};

class HipPlanner
{
private:
	int m_target_z_slice0;
	int m_target_z_slice1;
	int m_num_iterations = 10; //搜索的迭代次数
;
	int m_step_degree = 3;  //搜索时每步改变的角度，搜索的角度为+-num_iterations/2*step_degree
	bool m_ct_ready_flag = false;

//	using CTPixelType = signed short;
//	using CTImageType = itk::Image<CTPixelType, 3>;
//	using ReaderType = itk::ImageSeriesReader<CTImageType>;
//	using ImageIOType = itk::GDCMImageIO;
//	using NamesGeneratorType = itk::GDCMSeriesFileNames;
//
//	using PointType = itk::Point<float, 3>;

public:
	femur_neck_info m_info_transfered;
	femur_neck_info m_info_original;

	CTImageType::Pointer ct_image;  //ct原图
	CTImageType::Pointer m_left_femur_mask;  //双腿的mask
	CTImageType::Pointer m_right_femur_mask;
	CTImageType::Pointer m_target_femur_mask;
	CTImageType::Pointer target_feumr_masked_image;  //按maks取的原ct

	CTImageType::Pointer one_pixel_value_target_mask;


	Eigen::Vector3d m_OPERATIVE_INTERCONDYLAR_FOSSA;  //世界坐标系上的点
	Eigen::Vector3d m_OPERATIVE_BIG_TROCHANTER_FEATURES;
	Eigen::Vector3d m_OPERATIVE_FEMORAL_FEATURE_3;
	Eigen::Vector3d m_OPERATIVE_LESSER_TROCHANTER_FEATURES; //小转子
	PointType m_OPERATIVE_INTERCONDYLAR_FOSSA_pyhsicalpoint;
	PointType m_OPERATIVE_BIG_TROCHANTER_FEATURES_pyhsicalpoint;
	CTImageType::IndexType m_OPERATIVE_INTERCONDYLAR_FOSSA_idx;
	CTImageType::IndexType m_OPERATIVE_BIG_TROCHANTER_FEATURES_idx;

	Eigen::Vector3d m_femur_shaft_p0;
	Eigen::Vector3d m_femur_shaft_p1;

	//股骨颈相关成员变量
	vtkSmartPointer<vtkPolyData> m_left_femur_polydata;
	vtkSmartPointer<vtkPolyData> m_right_femur_polydata;
	vtkSmartPointer<vtkPolyData> m_target_femur_polydata;
	vtkSmartPointer<vtkPolyData> m_clipped_femur_polydata;
	
	vtkSmartPointer<vtkMatrix4x4> m_femur_neck_template_2_target_matrix;
	vtkSmartPointer<vtkLandmarkTransform> m_femur_neck_template_2_target_transform;
	Eigen::Matrix4d m_transform_matrix;
	Eigen::Matrix4d m_rotation_matrix;

	vtkNew<vtkPoints> m_template_points;

	Eigen::Vector3d m_stage0_femur_neck_axisz;

	Eigen::Vector3d m_res_femur_neck_normal;
	Eigen::Vector3d m_res_femur_neck_center;
	Eigen::Vector3d m_femur_neck_p0;
	Eigen::Vector3d m_femur_neck_p1;

	//股骨干相关代码
	CTImageType::Pointer cal_masked_image(CTImageType::Pointer image_mask, CTImageType::Pointer ctimage);
	CTImageType::Pointer binarized_masked_image(CTImageType::Pointer in_ct_image, CTPixelType threshold=1, CTPixelType constantValue=1);
	void write_test_result(CTImageType::Pointer ct_image);
	void pixel_iter(CTImageType::Pointer ct_image, CTImageType::IndexType& top_point_index, CTImageType::IndexType& bottom_point_index);
	CTImageType::Pointer ct_slice(CTImageType::Pointer ct_image, int slice_idx);
	CTImageType::Pointer extract_femur_cavity(CTImageType::Pointer in_ct_slice);
	Eigen::Vector3d extract_femur_cavity_center(CTImageType::Pointer in_ct_slice);
	void set_femur_shaft_landmark_points(Eigen::Vector3d OPERATIVE_INTERCONDYLAR_FOSSA, Eigen::Vector3d OPERATIVE_BIG_TROCHANTER_FEATURES);
	void run_femur_shaft_planning(string surgery_side="left");
	void run_femur_neck_planning(string surgery_side="left");

	void set_ct_image(CTImageType::Pointer target_ct_image)
	{
		ct_image = target_ct_image;
		m_ct_ready_flag = true;
	}
	void set_left_femur_mask_image(CTImageType::Pointer target_left_femur_mask)
	{
		m_left_femur_mask = target_left_femur_mask;
	}
	void set_right_femur_mask_image(CTImageType::Pointer target_right_femur_mask)
	{
		m_right_femur_mask = target_right_femur_mask;
	}
	void set_target_feumr_masked_image(CTImageType::Pointer target_left_feumr_masked)
	{
		target_feumr_masked_image = target_left_feumr_masked;
	}
	void set_one_pixel_value_target_mask(CTImageType::Pointer target_one_pixel_value_left_mask)
	{
		one_pixel_value_target_mask = target_one_pixel_value_left_mask;
	}


	//股骨颈相关
	vtkSmartPointer<vtkPolyData> load_STL(std::string STL_FILE_PATH);
	void set_femur_neck_landmark_points(Eigen::Vector3d OPERATIVE_INTERCONDYLAR_FOSSA, Eigen::Vector3d OPERATIVE_BIG_TROCHANTER_FEATURES, Eigen::Vector3d OPERATIVE_FEMORAL_FEATURE_3);
	void template_registration();
	void transfer_template();
	Eigen::Vector3d transform_point(Eigen::Vector3d point);
	void clip_femur_polydata();
	Eigen::Vector3d transfer_normal(Eigen::Vector3d normal);
	Eigen::Vector3d rotate_axis(Eigen::Vector3d target_axis, Eigen::Vector3d rotation_axis, float angle);
	vtkSmartPointer<vtkPolyData> get_cut_plane(vtkSmartPointer<vtkPolyData> target_polydata, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);
	float cal_area_volume(vtkSmartPointer<vtkPolyData> plane_polydata, double* center, int stage=0);
	void search_femur_neck();
	void set_femur_neck_point();
	//void render_result();

	void set_target_left_femur_polydata(vtkSmartPointer<vtkPolyData> target_polydata)
	{
		m_left_femur_polydata = target_polydata;
	}
	void set_target_right_femur_polydata(vtkSmartPointer<vtkPolyData> target_polydata)
	{
		m_right_femur_polydata = target_polydata;
	}
	void set_stage0_zaxis(Eigen::Vector3d femur_neck_axisz_new)
	{
		m_stage0_femur_neck_axisz = femur_neck_axisz_new;
	}


	void init_left_template()
	{
		double left_BIG_TROCHANTER_FEATURES[3] = { 124.224, 7.51454, -117.852 };
		m_template_points->InsertNextPoint(left_BIG_TROCHANTER_FEATURES);
		double left_LESSER_TROCHANTER_FEATURES[3] = { 100.569, 15.1608, -159.284 };
		m_template_points->InsertNextPoint(left_LESSER_TROCHANTER_FEATURES);
		double left_FEMORAL_FEATURE_3[3] = { 73.5557, -15.7137, -110.081 };
		m_template_points->InsertNextPoint(left_FEMORAL_FEATURE_3);

		m_info_original.femur_neck_center << 102.7795, -12.4660, -134.3659;
		m_info_original.femur_neck_cutplane_center << 122.819, -4.15586667, -169.4523;
		m_info_original.femur_neck_cutplane_normal << 0.20001134, -0.27879166, 0.93929264;
		m_info_original.femur_neck_axisx << -0.02992115, 0.96817822, 0.24846666;
		m_info_original.femur_neck_axisy << -0.60881033, 0.17949635, -0.77274255;
		m_info_original.femur_neck_axisz << -0.79276185, -0.17438732, 0.58405284;

		m_target_femur_polydata = m_left_femur_polydata;
	}

	void init_right_template()
	{
		double right_BIG_TROCHANTER_FEATURES[3] = { -109.63, 45.4139, -126.647 };
		m_template_points->InsertNextPoint(right_BIG_TROCHANTER_FEATURES);
		double right_LESSER_TROCHANTER_FEATURES[3] = { -84.0098, 37.0968, -170.79 };
		m_template_points->InsertNextPoint(right_LESSER_TROCHANTER_FEATURES);
		double right_FEMORAL_FEATURE_3[3] = { -81.5288, 5.01612, -114.098 };
		m_template_points->InsertNextPoint(right_FEMORAL_FEATURE_3);

		m_info_original.femur_neck_center << -105.55, 20.495, -142.59;
		m_info_original.femur_neck_cutplane_center << -109.99, 31.46, -171.596;
		m_info_original.femur_neck_cutplane_normal << -0.09981183, 0.24207728, 0.96510942;
		m_info_original.femur_neck_axisx << -0.58299541, -0.80242902, 0.12737332;
		m_info_original.femur_neck_axisy << 0.43816426, -0.44253927, -0.7824136;
		m_info_original.femur_neck_axisz << 0.68419908, -0.40033309, 0.60959415;

		m_target_femur_polydata = m_right_femur_polydata;
	}
};