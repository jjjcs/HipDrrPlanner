#include "DRRImageProcess.h"
#include "ImageIO.h"
#include "itkOrientImageFilter.h"
#include "itkImage.h"
#include <itkAffineTransform.h>
#include <itkTransform.h>

//#include "DataManager.h"
using namespace ImgIO;

DRRImageProcess::DRRImageProcess()
{
	rotation_x = -90.0;// rotation_x = -90.0;
	rotation_y = 0.0;
	rotation_z = 0.0;

	camera_tx = 0.0;
	camera_ty = 0.0;
	camera_tz = 150.0;

	//取左右侧股骨头球心中点
	double oriLeft[3] = { 0 };
	double oriRight[3] = { 0 };
	//DataManager::instance()->getAcetabulumOrigin(oriLeft, true);
	//DataManager::instance()->getAcetabulumOrigin(oriRight, false);
	////HD0007  40701339_R
	oriLeft[0] = 64.67;
	oriLeft[1] = 13.96;
	oriLeft[2] = -170.62;
	oriRight[0] = -95.517;
	oriRight[1] = 18.33;
	oriRight[2] = -186.06;
	//0004226089_L
	//oriLeft[0] = 99.285;
	//oriLeft[1] = 26.5;
	//oriLeft[2] = -720.3;
	//oriRight[0] = -67.5;
	//oriRight[1] = 30.7;
	//oriRight[2] = -726;

		//HD0040
	//oriLeft[0] = 55.06;
	//oriLeft[1] = -16.11;
	//oriLeft[2] = 454.125;
	//oriRight[0] = -112.90;
	//oriRight[1] = -23.67;
	//oriRight[2] = 463.5;
	

	center_x = (oriLeft[0] + oriRight[0]) / 2;
	center_y = (oriLeft[1] + oriRight[1]) / 2;
	center_z = (oriLeft[2] + oriRight[2]) / 2;

	ray_source_distance = 900.0;

	spacing_x = 2;
	spacing_y = 2;

	imagesize_x = 300;
	imagesize_y = 300;

	o2Dx = 0;
	o2Dy = 0;

	threshold = 10.0;
}


DRRImageProcess::~DRRImageProcess()
{
}

XRayImageType::Pointer DRRImageProcess::DRRProcess(InputImageType::Pointer image, InterpolatorType::InputPointType& myfocalpoint)
{
	InputImageType::PointType   imOrigin = image->GetOrigin();//这里是ct图像左下角的原点
	InputImageType::SpacingType imRes = image->GetSpacing();//这里是图像的真实物理像素宽度（mm）
	InputImageRegionType imRegion = image->GetBufferedRegion();
	InputImageSizeType   imSize = imRegion.GetSize();//获得图像尺寸
	InputImageType::DirectionType Direction = image->GetDirection();

	//直接用itk image省去了flip
	//filter设置image信息
	using FilterType = itk::ResampleImageFilter<InputImageType, InputImageType>;
	FilterType::Pointer filter = FilterType::New();
	filter->SetInput(image);
	filter->SetDefaultPixelValue(200);

	//定义DRR图像的欧拉转换，即在哪一个位置模拟成像
	TransformType::Pointer transform = TransformType::New();
	transform->SetComputeZYX(true);
	TransformType::OutputVectorType translation;

	translation[0] = camera_tx;
	translation[1] = camera_ty;
	translation[2] = camera_tz;  //似乎调整了DRR上下调整，camera_tz减小，drr位置向下
	//度与弧度的转换常数：1.0,atan return [-pi/2, pi/2]
	const double dtr = (std::atan(1.0) * 4.0) / 180.0;
	//平移与转换矩阵  -90,0,0
	transform->SetTranslation(translation);
	transform->SetRotation(dtr*rotation_x, dtr*rotation_y, dtr*rotation_z);

	//左右股骨头特征点中点坐标，如果没有设置的话是center_x=0,center_y=0,center_z=0
	m_center[0] = center_x;
	m_center[1] = center_y;
//	m_center[2] = center_z - 200;
	m_center[2] = center_z - 120/ imRes[2]; //图像中心位置为左右股骨头特征点中点向下12cm
	if (m_center[2] < imOrigin[2])
	{
		m_center[2] = imOrigin[2];
	}

	//设置转换中心物理坐标（旋转中心?）
	transform->SetCenter(m_center);

	//上述的转换矩阵提供了如何改变模拟X光的位置
	InterpolatorType::Pointer interpolator = InterpolatorType::New();
	interpolator->SetTransform(transform);
	transform_matrix = transform->GetMatrix();
	m_transform = transform;
	
	vtkNew<vtkMatrix4x4> my_transform_matrix;
	my_transform_matrix->Identity();

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			my_transform_matrix->SetElement(i, j, transform_matrix(i, j));
		}
	}
	// 设置一个阈值，超过了这个阈值，CT强度就会合并
	interpolator->SetThreshold(threshold);

	// Software Guide : BeginLatex
	// 射线投射法需要知道射线源的初始位置，或者是焦点，
	// 将物体放在射线源和屏幕之间，射线源和屏幕之间的距离是sid
	InterpolatorType::InputPointType focalpoint;
	InterpolatorType::InputPointType offset;
	//光源位置
	focalpoint[0] = m_center[0];
	focalpoint[1] = m_center[1];
	focalpoint[2] = m_center[2] - ray_source_distance / 2. - 500;

	interpolator->SetFocalPoint(focalpoint);
	myfocalpoint = focalpoint;
	offset = transform->GetOffset();

	my_transform_matrix->SetElement(0, 3, offset[0]);
	my_transform_matrix->SetElement(1, 3, offset[1]);
	my_transform_matrix->SetElement(2, 3, offset[2]);

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
	double origin[3];
	origin[0] = m_center[0] + o2Dx - spacing_x * ((double)imagesize_x - 1.) / 2. ;
	origin[1] = m_center[1] + o2Dy - spacing_y * ((double)imagesize_y - 1.) / 2.;
	origin[2] = m_center[2] + ray_source_distance / 2;
	drr_transform_matrix = my_transform_matrix;

	filter->SetOutputOrigin(origin);
	filter->Update();
	
	//设置Rescale,进行二维叠加时为保证还原暂时不对像素值做scale
	using RescaleFilterType = itk::RescaleIntensityImageFilter<
		InputImageType, OutputImageType>;
	RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
	rescaler->SetOutputMinimum(0);
	rescaler->SetOutputMaximum(255);
	rescaler->SetInput(filter->GetOutput());
	rescaler->Update();

	return rescaler->GetOutput();
}

vtkImageData* DRRImageProcess::DRRProcess(vtkImageData *dicom)
{
	if (dicom == nullptr)
	{
		return nullptr;
	}
	InputImageType::Pointer image;

	//vtk图像转itk图像
	vtk2itkFilterType::Pointer vtk2itkfilter = vtk2itkFilterType::New();
	vtk2itkfilter->SetInput(dicom);
	vtk2itkfilter->Update();
	image = vtk2itkfilter->GetOutput();

	//filter设置image信息
	FilterType::Pointer filter = FilterType::New();
	filter->SetInput(image);
	filter->SetDefaultPixelValue(0);

	//定义DRR图像的欧拉转换，即在哪一个位置模拟成像
	using TransformType = itk::CenteredEuler3DTransform< double >;
	TransformType::Pointer transform = TransformType::New();
	transform->SetComputeZYX(true);
	TransformType::OutputVectorType translation;

	translation[0] = camera_tx;
	translation[1] = camera_ty;
	translation[2] = camera_tz;
	//度与弧度的转换常数：1.0
	const double dtr = (std::atan(1.0) * 4.0) / 180.0;
	//平移与转换矩阵
	transform->SetTranslation(translation);
	transform->SetRotation(dtr*rotation_x, dtr*rotation_y, dtr*rotation_z);

	InputImageType::PointType   imOrigin = image->GetOrigin();//这里是图像左下角的原点
	InputImageType::SpacingType imRes = image->GetSpacing();//这里是图像的真实物理像素宽度（mm）

	InputImageRegionType imRegion = image->GetBufferedRegion();
	InputImageSizeType   imSize = imRegion.GetSize();//获得图像尺寸

	//计算图像以左下角为原点的中心点物理坐标 (CT volume的中心)
	imOrigin[0] += imRes[0] * static_cast<double>(imSize[0]) / 2.0;
	imOrigin[1] += imRes[1] * static_cast<double>(imSize[1]) / 2.0;
	imOrigin[2] += imRes[2] * static_cast<double>(imSize[2]) / 2.0;

	//图像总的中心坐标，如果没有设置的话是center_x=0,center_y=0,center_z=0 volume和两侧股骨中点的
	m_center[0] = (center_x + imOrigin[0]) / 2;
	m_center[1] = (center_y + imOrigin[1]) / 2;
	m_center[2] = (center_z + imOrigin[2]) / 2;
	//设置转换中心物理坐标
	transform->SetCenter(m_center);


	//上述的转换矩阵提供了如何改变模拟X光的位置
	InterpolatorType::Pointer interpolator = InterpolatorType::New();
	interpolator->SetTransform(transform);

	// 设置一个阈值，超过了这个阈值，CT强度就会合并
	interpolator->SetThreshold(threshold);
	
	// Software Guide : BeginLatex
	// 射线投射法需要知道射线源的初始位置，或者是焦点，
	// 将物体放在射线源和屏幕之间，射线源和屏幕之间的距离是sid
	InterpolatorType::InputPointType focalpoint;
	//光源位置
	focalpoint[0] = m_center[0];
	focalpoint[1] = m_center[1];
	focalpoint[2] = m_center[2] - ray_source_distance / 2.;
	interpolator->SetFocalPoint(focalpoint);
	
	interpolator->Print(std::cout);

	filter->SetInterpolator(interpolator);
	filter->SetTransform(transform);
	// Software Guide : EndCodeSnippet

	// Software Guide : BeginLatex
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

	//设置源点坐标
	//设置DRR源点坐标,imOrigin是CTvolume中心，在此基础上
	m_origin[0] = m_center[0] + o2Dx - spacing_x * ((double)imagesize_x - 1.) / 2.;
	m_origin[1] = m_center[1] + o2Dy - spacing_y * ((double)imagesize_y - 1.) / 2.;
	m_origin[2] = m_center[2] + ray_source_distance / 2;
	filter->SetOutputOrigin(m_origin);
	filter->Update();
	
	//设置Rescale
	using RescaleFilterType = itk::RescaleIntensityImageFilter<
		OutputImageType, OutputImageType>;
	RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
	rescaler->SetOutputMinimum(0);
	rescaler->SetOutputMaximum(255);
	rescaler->SetInput(filter->GetOutput());

	//坐标做flip转换
	FlipFilterType::Pointer flipperImage = FlipFilterType::New();
	bool flipAxes[3] = { false, true, false };
	flipperImage = FlipFilterType::New();
	flipperImage->SetFlipAxes(flipAxes);
	flipperImage->SetInput(rescaler->GetOutput());
	flipperImage->Update();

	//itk图像转vtk
	using itk2vtkFilter = itk::ImageToVTKImageFilter<XRayImageType>;
	vtkImageData *vtkimage = vtkImageData::New();
	itk2vtkFilter::Pointer itkTovtkImageFilter = itk2vtkFilter::New();
	itkTovtkImageFilter->SetInput(flipperImage->GetOutput());
	itkTovtkImageFilter->Update();
	vtkImageData *d = itkTovtkImageFilter->GetOutput();
	vtkimage->DeepCopy(d);

	return vtkimage;
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
	if (rx != 0)
	{
		rotation_x = rx;
	}
	if (ry != 0)
	{
		rotation_y = ry;
	}
	if (rz != 0)
	{
		rotation_z = rz;
	}
}

void DRRImageProcess::getrotation_angle(float& o_rx, float& o_ry, float& o_rz)
{
	o_rx = rotation_x;
	o_ry = rotation_y;
	o_rz = rotation_z;
};