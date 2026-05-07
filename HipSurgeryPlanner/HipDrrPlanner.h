#pragma once
#include "ImageIO.h"
#include <itkImage.h>
#include <itkImageRegionIterator.h>
#include <itkImageDuplicator.h>
#include <itkRigid3DTransform.h>
#include <itkAffineTransform.h>
#include <itkResampleImageFilter.h>
#include <itkPoint.h>

#include <vtkMatrix4x4.h>
#include <vtkNew.h>
//#include "HipSurgeryPlanner.h"
#include "DRRImageProcess.h"

using namespace ImgIO;

struct DRR_parameters
{
	//记录生成DRR的参数，传递给窗口以便显示DRR图像
	double drr_origin[3];  //drr 图像的 origin
	XRayImageType::SizeType  drr_size;
	XRayImageType::SpacingType drr_spacing;
	double focal_point[3];
	vtkSmartPointer<vtkMatrix4x4> m_drr_transform_matrix;
};


//在drr正位图像上展示规划结果
class HipDrrPlanner
{

private:
	using AffineTransformType = itk::AffineTransform<double, 3>;
	using TransformType = itk::Rigid3DTransform<double>;
	using FilterType = itk::ResampleImageFilter<CTImageType, CTImageType >;
	float rx;
	float ry;
	float rz;

	CTImageType::Pointer m_left_femur_masked_image;
	CTImageType::Pointer m_right_femur_masked_image;
	CTImageType::Pointer m_left_femur_reset_masked_image;
	CTImageType::Pointer m_right_femur_reset_masked_image;
	CTImageType::Pointer m_left_hip_masked_image;
	CTImageType::Pointer m_right_hip_masked_image;
	CTImageType::Pointer m_cropped_right_femur_masked_image;
	CTImageType::Pointer m_cropped_left_femur_masked_image;

	CTImageType::Pointer m_scarum_masked_image;
	CTImageType::Pointer m_whole_masked_image;
	CTImageType::Pointer m_whole_reset_masked_image;

	XRayImageType::Pointer m_left_femur_drr_image;
	XRayImageType::Pointer m_right_femur_drr_image;
	XRayImageType::Pointer m_left_hip_drr_image;
	XRayImageType::Pointer m_right_hip_drr_image;
	XRayImageType::Pointer m_scarum_drr_image;
	XRayImageType::Pointer m_whole_drr_image;
	XRayImageType::Pointer m_femur_stem_drr_image;

	XRayImageType::Pointer m_left_hip_mask;
	XRayImageType::Pointer m_right_hip_mask;
	XRayImageType::Pointer m_whole_hip_mask;

	vtkSmartPointer<vtkImageData> m_femur_stem_vtkimage;
	XRayImageType::PointType m_focalpoint;
	std::vector<XRayImageType::Pointer> m_drr_images;

	vtkSmartPointer<vtkPolyData> m_femur_stem_polydata;
	vtkSmartPointer<vtkActor> m_femur_stem_actor;
	vtkSmartPointer<vtkPolyData> m_femur_head_polydata;
	vtkSmartPointer<vtkActor> m_femur_head_actor;
	vtkSmartPointer<vtkPolyData> m_hip_cup_polydata;
	vtkSmartPointer<vtkActor> m_hip_cup_actor;

	CTImageType::Pointer m_femur_stem_image;
	bool m_using_scarum = FALSE;

	vtkSmartPointer<vtkImageData> m_render_result;
	vtkSmartPointer<vtkImageData> m_implant_render_result;
	vtkNew<vtkRenderWindow> m_renderWindow;

	Eigen::Matrix4d m_left_femur_reset_matrix;
	Eigen::Matrix4d m_right_femur_reset_matrix;

	double* m_left_femur_bounds;
	double* m_right_femur_bounds;
	double* m_transformed_left_femur_bounds;
	double* m_transformed_right_femur_bounds;
	bool left_reset_matrix_change=true;
	bool right_reset_matrix_change = true;

public:
	double left_f_bbox_itk[6];
	double right_f_bbox_itk[6];
	double transformed_left_f_bbox_itk[6];
	double transformed_right_f_bbox_itk[6];
	int transformed_left_f_ref_size[3];
	int transformed_right_f_ref_size[3];

	std::vector<vtkSmartPointer<vtkActor>> m_target_actors_list;
	CTImageType::Pointer m_ct_image;  //ct原图
	CTImageType::SpacingType m_ori_spacing;
	CTImageType::SizeType m_ori_size;
	CTImageType::PointType m_ori_origin;
	CTImageType::DirectionType m_ori_direction;

	std::vector<XRayImageType::Pointer> m_mask_list;
	std::vector<CTImageType::Pointer> m_masked_list;
	XRayImageType::Pointer m_left_femur_mask;  //双腿的mask
	XRayImageType::Pointer m_right_femur_mask;

	CTImageType::Pointer m_whole_hip_masked_image;
	XRayImageType::Pointer m_scarum_mask;
	XRayImageType::Pointer m_whole_mask_image;
	vtkSmartPointer<vtkMatrix4x4> m_drr_transform_matrix;
	itk::Matrix<double, 3, 3> m_drr_transform_matrix_itk;

	void set_whole_drr_image(XRayImageType::Pointer target_drr_image)
	{
		m_whole_drr_image = target_drr_image;
	}

	void set_left_femur_drr_image(XRayImageType::Pointer target_drr_image)
	{
		m_left_femur_drr_image = target_drr_image;
	}
	void set_right_femur_drr_image(XRayImageType::Pointer target_drr_image)
	{
		m_right_femur_drr_image = target_drr_image;
	}
	void set_left_hip_drr_image(XRayImageType::Pointer target_drr_image)
	{
		m_left_hip_drr_image = target_drr_image;
	}
	void set_right_hip_drr_image(XRayImageType::Pointer target_drr_image)
	{
		m_right_hip_drr_image = target_drr_image;
	}
	void set_scarum_drr_image(XRayImageType::Pointer target_drr_image)
	{
		m_scarum_drr_image = target_drr_image;
	}

	void set_ct_image(CTImageType::Pointer target_ct_image)
	{
		m_ct_image = target_ct_image;
		m_ori_spacing = m_ct_image->GetSpacing();
		m_ori_size = m_ct_image->GetLargestPossibleRegion().GetSize();
		m_ori_origin = m_ct_image->GetOrigin();
		m_ori_direction = m_ct_image->GetDirection();
	}

	void set_left_femur_mask_image(XRayImageType::Pointer target_left_femur_mask)
	{
		if (target_left_femur_mask == nullptr)
			return;
		m_left_femur_mask = target_left_femur_mask;
	}
	void set_right_femur_mask_image(XRayImageType::Pointer target_right_femur_mask)
	{
		if (target_right_femur_mask == nullptr)
			return;
		m_right_femur_mask = target_right_femur_mask;
	}
	void set_left_hip_mask_image(XRayImageType::Pointer target_left_hip_mask)
	{
		if (target_left_hip_mask == nullptr)
			return;
		m_left_hip_mask = target_left_hip_mask;
	}
	void set_right_hip_mask_image(XRayImageType::Pointer target_right_hip_mask)
	{
		if (target_right_hip_mask == nullptr)
			return;
		m_right_hip_mask = target_right_hip_mask;
	}
	void set_scarum_mask_image(XRayImageType::Pointer target_scarum_mask)
	{
		if (target_scarum_mask == nullptr)
			return;
		m_scarum_mask = target_scarum_mask;
		m_using_scarum = TRUE;
	}

	void set_femur_stem_vtkimage(vtkSmartPointer<vtkImageData> target_femur_stem)
	{
		m_femur_stem_vtkimage = target_femur_stem;
	}

	void set_left_femur_masked_image(CTImageType::Pointer target_left_femur_masked)
	{
		m_left_femur_masked_image = target_left_femur_masked;
	}
	CTImageType::Pointer get_left_femur_masked_image()
	{
		CTImageType::Pointer clonedImage = copy_image(m_left_femur_masked_image);
		return clonedImage;
	}
	void set_left_femur_reset_masked_image(CTImageType::Pointer target_left_femur_masked)
	{
		m_left_femur_reset_masked_image = target_left_femur_masked;
	}
	CTImageType::Pointer get_left_femur_reset_masked_image()
	{
		CTImageType::Pointer clonedImage = copy_image(m_left_femur_reset_masked_image);
		return clonedImage;
	}

	void set_right_femur_masked_image(CTImageType::Pointer target_right_femur_masked)
	{
		m_right_femur_masked_image = target_right_femur_masked;
	}
	CTImageType::Pointer get_right_femur_masked_image()
	{
		CTImageType::Pointer clonedImage = copy_image(m_right_femur_masked_image);
		return clonedImage;
	}
	void set_right_femur_reset_masked_image(CTImageType::Pointer target_right_femur_masked)
	{
		m_right_femur_reset_masked_image = target_right_femur_masked;
	}
	CTImageType::Pointer get_right_femur_reset_masked_image()
	{
		CTImageType::Pointer clonedImage = copy_image(m_right_femur_reset_masked_image);
		return clonedImage;
	}

	void set_left_hip_masked_image(CTImageType::Pointer target_left_hip_masked)
	{
		m_left_hip_masked_image = target_left_hip_masked;
	}
	CTImageType::Pointer get_left_hip_masked_image()
	{
		CTImageType::Pointer clonedImage = copy_image(m_left_hip_masked_image);
		return clonedImage;
	}

	void set_right_hip_masked_image(CTImageType::Pointer target_right_hip_masked)
	{
		m_right_hip_masked_image = target_right_hip_masked;
	}
	CTImageType::Pointer get_right_hip_masked_image()
	{
		CTImageType::Pointer clonedImage = copy_image(m_right_hip_masked_image);
		return clonedImage;
	}
	void set_scarum_masked_image(CTImageType::Pointer target_scarum_masked)
	{
		m_scarum_masked_image = target_scarum_masked;
	}
	CTImageType::Pointer get_scarum_masked_image()
	{
		CTImageType::Pointer clonedImage = copy_image(m_scarum_masked_image);
		return clonedImage;
	}
	void set_whole_hip_masked_image(CTImageType::Pointer target_whole_masked)
	{
		m_whole_hip_masked_image = target_whole_masked;
	}
	CTImageType::Pointer get_whole_hip_masked_image()
	{
		CTImageType::Pointer clonedImage = copy_image(m_whole_hip_masked_image);
		return clonedImage;
	}
	void set_whole_masked_image(CTImageType::Pointer target_whole_masked)
	{
		m_whole_masked_image = target_whole_masked;
	}
	CTImageType::Pointer get_whole_masked_image()
	{
		CTImageType::Pointer clonedImage = copy_image(m_whole_masked_image);
		return clonedImage;
	}
	void set_whole_reset_masked_image(CTImageType::Pointer target_whole_masked)
	{
		m_whole_reset_masked_image = target_whole_masked;
	}
	CTImageType::Pointer get_whole_reset_masked_image()
	{
		CTImageType::Pointer clonedImage = copy_image(m_whole_reset_masked_image);
		return clonedImage;
	}

	void set_femur_stem_polydata(vtkSmartPointer<vtkPolyData> femur_stem_polydata)
	{
		m_femur_stem_polydata = femur_stem_polydata;
	}
	void set_femur_head_polydata(vtkSmartPointer<vtkPolyData> femur_head_polydata)
	{
		m_femur_head_polydata = femur_head_polydata;
	}
	void set_hip_cup_polydata(vtkSmartPointer<vtkPolyData> hip_cup_polydata)
	{
		m_hip_cup_polydata = hip_cup_polydata;
	}

	void set_femur_stem_actor(vtkSmartPointer<vtkActor> femur_stem_actor)
	{
		m_femur_stem_actor = femur_stem_actor;
	}
	void set_femur_head_actor(vtkSmartPointer<vtkActor> femur_head_actor)
	{
		m_femur_head_actor = femur_head_actor;
	}
	void set_hip_cup_actor(vtkSmartPointer<vtkActor> hip_cup_actor)
	{
		m_hip_cup_actor = hip_cup_actor;
	}

	vtkSmartPointer<vtkImageData> get_render_result()
	{
		vtkSmartPointer<vtkImageData> result_image = vtkSmartPointer<vtkImageData>::New();
		result_image->DeepCopy(m_render_result);
		return result_image;
	}

	void set_left_femur_reset_matrix(Eigen::Matrix4d target_matrix)
	{
		if (m_left_femur_reset_matrix != target_matrix)
		{
			m_left_femur_reset_matrix = target_matrix;
			left_reset_matrix_change = true;
		}
		else
		{
			left_reset_matrix_change = false;
		}

	}
	void set_right_femur_reset_matrix(Eigen::Matrix4d target_matrix)
	{
		if (m_right_femur_reset_matrix != target_matrix)
		{
			m_right_femur_reset_matrix = target_matrix;
			right_reset_matrix_change = true;
		}
		else
		{
			right_reset_matrix_change = false;
		}
	}

	void set_left_femur_bbox(double* target_bbox)
	{
		m_left_femur_bounds = target_bbox;
	}
	void set_right_femur_bbox(double* target_bbox)
	{
		m_right_femur_bounds = target_bbox;
	}
	double* get_left_femur_bbox()
	{
		return m_left_femur_bounds;
	}
	double* get_right_femur_bbox()
	{
		return m_right_femur_bounds;
	}
	void set_transformed_left_femur_bbox(double* target_bbox)
	{
		m_transformed_left_femur_bounds = target_bbox;
		transformed_left_f_ref_size[0] = m_transformed_left_femur_bounds[1] - m_transformed_left_femur_bounds[0];
		transformed_left_f_ref_size[1] = m_transformed_left_femur_bounds[3] - m_transformed_left_femur_bounds[2];
		transformed_left_f_ref_size[2] = m_transformed_left_femur_bounds[5] - m_transformed_left_femur_bounds[4];
	}
	void set_transformed_right_femur_bbox(double* target_bbox)
	{
		m_transformed_right_femur_bounds = target_bbox;
		transformed_right_f_ref_size[0] = m_transformed_right_femur_bounds[1] - m_transformed_right_femur_bounds[0];
		transformed_right_f_ref_size[1] = m_transformed_right_femur_bounds[3] - m_transformed_right_femur_bounds[2];
		transformed_right_f_ref_size[2] = m_transformed_right_femur_bounds[5] - m_transformed_right_femur_bounds[4];
	}
	double* get_transformed_left_femur_bbox()
	{
		return m_transformed_left_femur_bounds;
	}
	double* get_transformed_right_femur_bbox()
	{
		return m_transformed_right_femur_bounds;
	}

	void init_drr_planner();
	void init_drr_planner_threads();
	void init_drr_reset_planner();
	void run_drr_planner(bool reset=FALSE);
	void set_angle(float rx, float ry, float rz);
	CTImageType::Pointer binarized_masked_image(CTImageType::Pointer in_ct_image, CTPixelType threshold = 1, CTPixelType constantValue = 1);
	XRayImageType::Pointer binarized_masked_image(XRayImageType::Pointer in_ct_image, CTPixelType threshold = 1, CTPixelType constantValue = 1);
	//XRayImageType::Pointer binarized_masked_image_thread(XRayImageType::Pointer in_ct_image);
	
	XRayImageType::Pointer combine_mask_image(std::vector<XRayImageType::Pointer> mask_list);
	CTImageType::Pointer combine_masked_image(std::vector<CTImageType::Pointer> mask_list);
	CTImageType::Pointer cal_masked_image(XRayImageType::Pointer image_mask, CTImageType::Pointer ctimage);
	CTImageType::Pointer copy_image(CTImageType::Pointer target_image);
	XRayImageType::Pointer copy_image(XRayImageType::Pointer target_image);
	vtkSmartPointer<vtkImageData> itk_image2_vtk(CTImageType::Pointer in_image);
	vtkSmartPointer<vtkImageData> itk_image2_vtk(XRayImageType::Pointer in_image);
	CTImageType::Pointer vtk_image2_itk(vtkImageData* in_Image);
	void write_test_result(CTImageType::Pointer ct_image, std::string out_path="./out_test.nii.gz");
	void write_test_result(XRayImageType::Pointer ct_image, string out_path = "./out_test.nii.gz");
	void write_nrrd_test_result(CTImageType::Pointer ct_image, string out_path = "./out_test.nrrd");
	void write_nrrd_test_result(XRayImageType::Pointer ct_image, string out_path = "./out_test.nrrd");

	void vtk_write_nifti(vtkImageData* target_image_data, string save_path="./vtk_image.nii.gz");
	XRayImageType::Pointer drr_overlap(std::vector<XRayImageType::Pointer> drr_stuck_list);
	XRayImageType::Pointer mask_overlap(std::vector<XRayImageType::Pointer> mask_stuck_list);
	XRayImageType::Pointer mask_overlap_filter(std::vector<XRayImageType::Pointer> drr_stuck_list);
	void vtk_polydata_to_image(vtkSmartPointer<vtkPolyData> target_polydata, vtkSmartPointer<vtkImageData> out_imagedata);

	//用于在DRR上可视化三维假体模型
	void drr_bone_implant_render(XRayImageType::Pointer target_data);
	void single_implant_render(CTImageType::Pointer target_data);
	void itk_rotation_test(CTImageType::Pointer target_data);
	void run_implant_refresh();

	//用于完成DRR复位
	template <typename type> typename type::Pointer transform_ct_image(typename type::Pointer target_image, Eigen::Matrix4d transform_matrix, std::string side="left");
	CTImageType::Pointer crop_ct_image(CTImageType::Pointer target_image, XRayImageType::Pointer target_label_image, double* bounds);
	template <typename ImageType>
	typename ImageType::PointType cal_ref_point(typename ImageType::PointType origin, double* femur_ref_point,
		Eigen::Matrix4d transform_matrix, int* ref_size);
	void convert_vtk_to_itk_bounds(double* f_bbox_vtk, double* f_bbox_itk);
	CTImageType::Pointer get_masked_image(XRayImageType::Pointer mask_image, CTImageType::Pointer ct_image);
	void process_render_result();
};