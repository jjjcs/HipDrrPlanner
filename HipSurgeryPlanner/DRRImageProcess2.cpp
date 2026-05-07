
#include "DRRImageProcess.h"
#include "ImageIO.h"
#include "itkOrientImageFilter.h"
#include "itkImage.h"
#include <itkAffineTransform.h>
#include <itkTransform.h>

using namespace ImgIO;

DRRImageProcess::DRRImageProcess()
{
	rotation_x = -90.0;// rotation_x = -90.0;
	rotation_y = 0.0;
	rotation_z = 0.0;

	//the translation of camera_sensor system
	camera_tx = 0.0;
	camera_ty = 0.0;
	camera_tz = 150.0;

	center_x = 0;
	center_y = 0;
	center_z = 0;

	ray_source_distance = 900.0;

	spacing_x = 1;
	spacing_y = 1;

	imagesize_x = 512;
	imagesize_y = 512;

	o2Dx = 0;
	o2Dy = 0;

	threshold = 10.0;
	m_drr_transform_matrix.setIdentity();
}


DRRImageProcess::~DRRImageProcess()
{
}

void cal_proj_matrix( )
{
}

GrayImageType::Pointer DRRImageProcess::DRRProcess(ImageType::Pointer image)
{
	ImageType::PointType   imOrigin = image->GetOrigin();//这里是ct图像左下角的原点
	ImageType::SpacingType imRes = image->GetSpacing();//这里是图像的真实物理像素宽度（mm）
	InputImageRegionType imRegion = image->GetBufferedRegion();
	InputImageSizeType   imSize = imRegion.GetSize();//获得图像尺寸
	ImageType::DirectionType Direction = image->GetDirection();

	//直接用itk image省去了flip
	//filter设置image信息
	using FilterType = itk::ResampleImageFilter<ImageType, ImageType>;
	FilterType::Pointer filter = FilterType::New();
	filter->SetInput(image);
	filter->SetDefaultPixelValue(200);

	//定义DRR图像的欧拉转换，即在哪一个位置模拟成像
	TransformType::Pointer transform = TransformType::New();
	transform->SetComputeZYX(true);

	TransformType::OutputVectorType translation;
	translation[0] = camera_tx;
	translation[1] = camera_ty;
	translation[2] = camera_tz;  
	//度与弧度的转换常数：1.0,atan return [-pi/2, pi/2]
	const double dtr = (std::atan(1.0) * 4.0) / 180.0;
	//平移与转换矩阵  -90,0,0
	transform->SetTranslation(translation);
	transform->SetRotation(dtr * rotation_x, dtr * rotation_y, dtr * rotation_z);

	ImageType::PointType m_centerPhysicalPoint;
	m_centerPhysicalPoint[0] = center_x;
	m_centerPhysicalPoint[1] = center_y;
	m_centerPhysicalPoint[2] = center_z;
	//设置转换旋转中心,x'= R⋅(x−c) + c + t
	//c 是 Center 定义的旋转中心，
	//R 是旋转矩阵（由欧拉角计算得到）
	//t 是平移向量 Translation。
	//计算验证transform的offset是−Rc + c + t
	transform->SetCenter(m_centerPhysicalPoint);

	//上述的转换矩阵提供了如何改变模拟X光的位置
	InterpolatorType::Pointer interpolator = InterpolatorType::New();
	interpolator->SetTransform(transform);
	transform_matrix = transform->GetMatrix();
	//验证offset
	//ImageType::PointType p0 = transform_matrix * m_centerPhysicalPoint;
	//double offset_x = -p0[0] + m_centerPhysicalPoint[0] + translation[0];
	//double offset_y = -p0[1] + m_centerPhysicalPoint[1] + translation[1];
	//double offset_z = -p0[2] + m_centerPhysicalPoint[2] + translation[2];
	m_transform_matrix->Identity();

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			m_transform_matrix->SetElement(i, j, transform_matrix(i, j));
			m_drr_transform_matrix(i, j) = transform_matrix(i, j);
		}
	}

	m_offset = transform->GetOffset();
	m_transform = transform;
	m_transform_matrix->SetElement(0, 3, m_offset[0]);
	m_transform_matrix->SetElement(1, 3, m_offset[1]);
	m_transform_matrix->SetElement(2, 3, m_offset[2]);
	m_drr_transform_matrix(0, 3) = m_offset[0];
	m_drr_transform_matrix(1, 3) = m_offset[1];
	m_drr_transform_matrix(2, 3) = m_offset[2];

	// 设置一个阈值，超过了这个阈值，CT强度就会合并
	interpolator->SetThreshold(threshold);

	// Software Guide : BeginLatex
	// 射线投射法需要知道射线源的初始位置，或者是焦点，
	// 将物体放在射线源和屏幕之间，射线源和屏幕之间的距离是sid

	//光源位置
	m_focalpoint[0] = m_centerPhysicalPoint[0];
	m_focalpoint[1] = m_centerPhysicalPoint[1];
	m_focalpoint[2] = m_centerPhysicalPoint[2] - ray_source_distance / 2.;

	interpolator->SetFocalPoint(m_focalpoint);
	filter->SetInterpolator(interpolator);
	filter->SetTransform(transform);

	// 滤波器设置DRR图像的尺寸和分辨率
	//像素X和Y方向的像素数目
	m_size[0] = imagesize_x;  // number of pixels along X of the 2D DRR image
	m_size[1] = imagesize_y;  // number of pixels along Y of the 2D DRR image
	m_size[2] = 1;   // only one slice
	filter->SetSize(m_size);

	//设置像素的物理尺寸
	m_spacing[0] = spacing_x;  // pixel spacing along X of the 2D DRR image [mm]
	m_spacing[1] = spacing_y;  // pixel spacing along Y of the 2D DRR image [mm]
	m_spacing[2] = 1.0; // slice thickness of the 2D DRR image [mm]
	filter->SetOutputSpacing(m_spacing);

	//设置源点坐标,imOrigin是CTvolume中心，在此基础上
	m_origin[0] = m_centerPhysicalPoint[0] + o2Dx - spacing_x * (double)imagesize_x / 2.;
	m_origin[1] = m_centerPhysicalPoint[1] + o2Dy - spacing_y * (double)imagesize_y / 2.;
	m_origin[2] = m_centerPhysicalPoint[2] + ray_source_distance / 2.;
	m_corner_p0 << m_origin[0], m_origin[1], m_origin[2];
	m_corner_p1 << m_centerPhysicalPoint[0] + o2Dx + spacing_x * (double)imagesize_x / 2.,
		m_centerPhysicalPoint[1] + o2Dy - spacing_y * (double)imagesize_y / 2.,
		m_centerPhysicalPoint[2] + ray_source_distance / 2.;
	m_corner_p2 << m_centerPhysicalPoint[0] + o2Dx + spacing_x * (double)imagesize_x / 2.,
		m_centerPhysicalPoint[1] + o2Dy + spacing_y * (double)imagesize_y / 2.,
		m_centerPhysicalPoint[2] + ray_source_distance / 2.;
	m_corner_p3 << m_centerPhysicalPoint[0] + o2Dx - spacing_x * (double)imagesize_x / 2.,
		m_centerPhysicalPoint[1] + o2Dy + spacing_y * (double)imagesize_y / 2.,
		m_centerPhysicalPoint[2] + ray_source_distance / 2.;

	filter->SetOutputOrigin(m_origin);
	filter->Update();

	//设置Rescale,进行二维叠加时为保证还原暂时不对像素值做scale
	using RescaleFilterType = itk::RescaleIntensityImageFilter<
		ImageType, GrayImageType>;
	RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
	rescaler->SetOutputMinimum(0);
	rescaler->SetOutputMaximum(255);
	rescaler->SetInput(filter->GetOutput());
	rescaler->Update();

	//OutputImageType::SpacingType spacing = rescaler->GetOutput()->GetSpacing();
	//std::cout << spacing[0] << " " << spacing[1] << " " << spacing[2] << std::endl;
	GrayImageType::Pointer out_drr = rescaler->GetOutput();
	//out_drr->SetSpacing();

	return rescaler->GetOutput();
}

void DRRImageProcess::SetParam(float frotation_x /*= -90. */
	, float frotation_y /*= 0.0 */
	, float frotation_z /*= 0.0 */
	, float fcamera_tx /*= 0.0 */
	, float fcamera_ty /*= 0.0 */
	, float fcamera_tz /*= 0.0 */
	, float fcenter_x /*= 0.0 */
	, float fcenter_y /*= 0.0 */
	, float fcenter_z /*= 0.0 */
	, float fray_source_distance /*= 900.0 */
	, double dsthreshold /*= 10.0*/)
{
	rotation_x = frotation_x;
	rotation_y = frotation_y;
	rotation_z = frotation_z;

	camera_tx = fcamera_tx;
	camera_ty = fcamera_ty;
	camera_tz = fcamera_tz;

	center_x = fcenter_x;
	center_y = fcenter_y;
	center_z = fcenter_z;

	ray_source_distance = fray_source_distance;

	threshold = dsthreshold;
}

void DRRImageProcess::setrotation_angle(float rx, float ry, float rz)
{
	rotation_x = rx;
	rotation_y = ry;
	rotation_z = rz;

}

void DRRImageProcess::settranslation(float tx, float ty, float tz)
{
	camera_tx = tx;
	camera_ty = ty;
	camera_tz = tz;
}

void DRRImageProcess::getrotation_angle(float& o_rx, float& o_ry, float& o_rz)
{
	o_rx = rotation_x;
	o_ry = rotation_y;
	o_rz = rotation_z;
};

void DRRImageProcess::gettranlation(float& tx, float& ty, float& tz)
{
	tx = camera_tx;
	ty = camera_ty;
	tz = camera_tz;
};

vtkSmartPointer<vtkImageData> DRRImageProcess::itk_image2_vtk(GrayImageType::Pointer in_image)
{
	//write_test_result(in_image, "./test.nii.gz");
	GrayImageType::DirectionType direction = in_image->GetDirection();
	GrayImageType::DirectionType ori_direction = in_image->GetDirection();
	GrayImageType::PointType origin = in_image->GetOrigin();

	using FlipFilterType = itk::FlipImageFilter<GrayImageType>;
	FlipFilterType::Pointer flipperImage = FlipFilterType::New();
	bool flipAxes[3] = { false, true, false };
	flipperImage = FlipFilterType::New();
	flipperImage->SetFlipAxes(flipAxes);
	flipperImage->SetInput(in_image);
	flipperImage->Update();

	//using RescaleFilterType = itk::RescaleIntensityImageFilter<
	//ImageType, ImageType >;
	//RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
	//rescaler->SetOutputMinimum(0);
	//rescaler->SetOutputMaximum(255);
	//rescaler->SetInput(filter->GetOutput());
	//rescaler->Update();

	using FilterType = itk::ImageToVTKImageFilter<GrayImageType>;
	auto filter = FilterType::New();
	//filter->SetInput(flipperImage->GetOutput());
	filter->SetInput(in_image);
	filter->Update();

	vtkImageData* myvtkImageData = vtkImageData::New();
	vtkSmartPointer<vtkImageData> d = filter->GetOutput();
	double myorigin[3] = { origin[0], origin[1], origin[2] };
	//d->SetOrigin(myorigin);
	myvtkImageData->DeepCopy(d);
	return myvtkImageData;
}

void DRRImageProcess::cal_projection_matrix( Eigen::Matrix3d& Intrinsic_matrix, Eigen::Matrix4d& Trans_matrix )
{
	double fx = ray_source_distance;
	double fy = ray_source_distance;
	//坐标原点与像心对齐，物理尺度
	double cx = imagesize_x * spacing_x / 2;
	double cy = imagesize_y * spacing_y / 2;
	Intrinsic_matrix << fx, 0, cx,
		0, fy, cy,
		0, 0, 1;

	itk::Matrix<double, 3, 3> transform_matrix_tmp;
	transform_matrix_tmp = m_transform->GetMatrix();
	
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			Trans_matrix(i, j) = transform_matrix_tmp[i][j];
		}
	}

	Trans_matrix(0, 3) = m_offset[0];
	Trans_matrix(1, 3) = m_offset[1];
	Trans_matrix(2, 3) = m_offset[2];
	//std::cout << Trans_matrix << std::endl;
}

void DRRImageProcess::cal_projection_matrix_corner_points(Eigen::Matrix3d& Intrinsic_matrix, Eigen::Matrix4d& Trans_matrix)
{
	m_drr_transform_matrix;
	//直接使用transform后的角点计算
	Eigen::Vector4d focal_point_vector(m_focalpoint[0], m_focalpoint[1], m_focalpoint[2], 1);
	Eigen::Vector4d focal_point_vector_transformed = m_drr_transform_matrix * focal_point_vector;
	Eigen::Vector4d corner_p0_transformed, corner_p1_transformed, corner_p2_transformed, corner_p3_transformed;
	corner_p0_transformed = m_drr_transform_matrix * m_corner_p0.homogeneous();
	corner_p1_transformed = m_drr_transform_matrix * m_corner_p1.homogeneous();
	corner_p2_transformed = m_drr_transform_matrix * m_corner_p2.homogeneous();
	corner_p3_transformed = m_drr_transform_matrix * m_corner_p3.homogeneous();
	Eigen::Vector3d view_up_x = (corner_p0_transformed - corner_p1_transformed).head(3);
	view_up_x = view_up_x.normalized();
	Eigen::Vector3d view_up_y = (corner_p0_transformed - corner_p3_transformed).head(3);
	view_up_y = view_up_y.normalized();

	std::cout << corner_p0_transformed << "\n" << corner_p1_transformed << "\n" << corner_p2_transformed << "\n" << corner_p3_transformed << std::endl;

	Eigen::Vector4d scene_center = (corner_p0_transformed + corner_p1_transformed + corner_p2_transformed + corner_p3_transformed) / 4;
	Eigen::Vector3d forward = (scene_center.head(3) - focal_point_vector_transformed.head(3)).normalized();  // 相机看向场景中心
	Eigen::Vector3d rotation_center(center_x, center_y, center_z);
	Eigen::Vector3d forward2 = (rotation_center - focal_point_vector_transformed.head(3)).normalized();
	Eigen::Vector3d up(0, 1, 0);                    // 假设 Up 是 Y 轴
	up = view_up_y;
	Eigen::Vector4d up_transformed;
	std::cout << up.dot(forward) << std::endl;
													//Eigen::Vector3d up = m_drr_transform_matrix.block<1,3>(1,0);
	Eigen::Vector3d right = up.cross(forward).normalized();     // Right = Up × Forward
	//up = forward.cross(right).normalized();              // 重新正交化 Up

	Eigen::Matrix3d R;
	R.col(0) = right;
	R.col(1) = up;
	R.col(2) = forward;

	Eigen::Vector3d t = -R * focal_point_vector.head(3);

	// 外参矩阵 [R | t]（4x4 齐次形式）
	Trans_matrix.block<3, 3>(0, 0) = R;
	Trans_matrix.block<3, 1>(0, 3) = t;
	
	double fx = ray_source_distance;
	double fy = ray_source_distance;
	//坐标原点与像心对齐，物理尺度
	double cx = imagesize_x * spacing_x / 2;
	double cy = imagesize_y * spacing_y / 2;
	Intrinsic_matrix << fx, 0, cx,
		0, fy, cy,
		0, 0, 1;

}