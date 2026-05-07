#ifndef DRRIMAGEPROCESS_H
#define DRRIMAGEPROCESS_H
//#include "Eigen/Core""
#include "DRRGenerationDefine.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkCenteredEuler3DTransform.h"
#include "itkNearestNeighborInterpolateImageFunction.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkGiplImageIOFactory.h"
#include "itkGDCMImageIO.h" 
#include "itkGDCMSeriesFileNames.h"  
#include "itkImageSeriesWriter.h"
#include "itkImageSeriesReader.h" 
#include "itkNumericSeriesFileNames.h" 
#include "itkRayCastInterpolateImageFunction.h"
#include "vtkImageData.h"
#include "itkVTKImageToImageFilter.h"
#include "itkImageToVTKImageFilter.h"
#include "itkFlipImageFilter.h"
#include <iostream>
#include "vtkTransform.h"

using namespace std;

class DRRImageProcess
{
private:
	float rotation_x;
	float rotation_y;
	float rotation_z;

	float camera_tx;
	float camera_ty;
	float camera_tz;

	float center_x;
	float center_y;
	float center_z;

	float ray_source_distance;

	float spacing_x;
	float spacing_y;

	int imagesize_x;
	int imagesize_y;

	float o2Dx;
	float o2Dy;

	double threshold;

	using FilterType = itk::ResampleImageFilter<ImageType, GrayImageType>;
	using InputImageRegionType = ImageType::RegionType;
	using InputImageSizeType = InputImageRegionType::SizeType;
	using InterpolatorType = itk::RayCastInterpolateImageFunction<ImageType, double>;
	using vtk2itkFilterType = itk::VTKImageToImageFilter<ImageType >;

	using FlipFilterType = itk::FlipImageFilter<GrayImageType>;
	using TransformType = itk::CenteredEuler3DTransform< double >;

	ImageType::SizeType   m_size;
	ImageType::SpacingType m_spacing;
	//TransformType::Pointer m_transform = TransformType::New();
	//double m_origin[3];  //DRR源点坐标

	ImageType::Pointer m_target_ct_image;
	TransformType::Pointer m_transform;
	TransformType::InputPointType m_center;
	ImageType::PointType m_rotation_center;



public:
	DRRImageProcess();
	~DRRImageProcess();

	Eigen::Vector3d m_corner_p0;
	Eigen::Vector3d m_corner_p1;
	Eigen::Vector3d m_corner_p2;
	Eigen::Vector3d m_corner_p3;

	Eigen::Matrix4d m_drr_transform_matrix;
	itk::Matrix<double, 3, 3> transform_matrix;
	vtkNew<vtkMatrix4x4> m_transform_matrix;
	TransformType::InputPointType m_offset;
	InterpolatorType::InputPointType m_focalpoint;
	double m_origin[3];

	GrayImageType::Pointer DRRProcess(ImageType::Pointer image);
	vtkSmartPointer<vtkImageData> itk_image2_vtk(GrayImageType::Pointer in_image);

	void setrotation_angle(float rx = 0., float ry = 0., float rz = 0.);
	void settranslation(float tx=0, float ty=0, float tz=0);
	void getrotation_angle(float& rx, float& ry, float& rz);
	void gettranlation(float& tx, float& ty, float& tz);

	//设置DRR转换相关参数信息，不设置使用默认值
	void SetParam(
		float frotation_x = 0.//float frotation_x = -90.
		, float frotation_y = 0.0
		, float frotation_z = 0.0
		, float fcamera_tx = 0.0
		, float fcamera_ty = 0.0
		, float fcamera_tz = 0.0
		, float fcenter_x = 0.0
		, float fcenter_y = 0.0
		, float fcenter_z = 0.0
		, float fray_source_distance = 900.0
		, double dthreshold = 10.0);

	void set_target_ct_image(ImageType::Pointer target_ct_image)
	{
		m_target_ct_image = target_ct_image;

		ImageType::PointType   imOrigin = m_target_ct_image->GetOrigin();//这里是ct图像左下角的原点
		ImageType::SpacingType imRes = m_target_ct_image->GetSpacing();//这里是图像的真实物理像素宽度（mm）
		InputImageRegionType imRegion = m_target_ct_image->GetBufferedRegion();
		InputImageSizeType   imSize = imRegion.GetSize();//获得图像尺寸
		ImageType::DirectionType Direction = m_target_ct_image->GetDirection();

		center_x = imOrigin[0] + (imSize[0] - 1) * imRes[0] * 0.5;
		center_y = imOrigin[1] + (imSize[1] - 1) * imRes[1] * 0.5;
		center_z = imOrigin[2] + (imSize[2] - 1) * imRes[2] * 0.5;
	};
	
	void get_spacing(float& x, float&y)
	{
		x = spacing_x;
		y = spacing_y;
	}

	void cal_projection_matrix(Eigen::Matrix3d& Intrinsic_matrix, Eigen::Matrix4d& Transform_matrix);
	void cal_projection_matrix_corner_points(Eigen::Matrix3d& Intrinsic_matrix, Eigen::Matrix4d& Trans_matrix);
	void get_drr_cornet_points(Eigen::Vector4d &corner_p0, Eigen::Vector4d& corner_p1, Eigen::Vector4d& corner_p2, Eigen::Vector4d& corner_p3);
};

#endif