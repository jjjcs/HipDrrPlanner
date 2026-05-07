#include "itkVTKImageToImageFilter.h"
#include "itkImageToVTKImageFilter.h"
#include "itkImageFileWriter.h"
#include "itkImage.h"   
#include "itkRigid3DTransform.h"
#include "itkPNGImageIOFactory.h"
#include "itkPNGImageIO.h"
#include "itkMultiplyImageFilter.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkAddImageFilter.h"

#include "itkRegionOfInterestImageFilter.h"
#include "itkLabelStatisticsImageFilter.h"
#include "itkLabelShapeKeepNObjectsImageFilter.h"

#include <vtkImageData.h>
#include <vtkImageStencil.h>
#include <vtkMetaImageWriter.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyDataToImageStencil.h>
#include <vtkSphereSource.h>
#include "vtkNIFTIImageWriter.h"
#include <vtkImageViewer2.h>
#include <vtkImageActor.h>
#include "vtkImageViewer.h"
#include "vtkRenderWindowInteractor.h"
#include <vtkImageMapper3D.h>
#include <vtkPNGWriter.h>
#include <vtkInteractorStyleImage.h>
#include <vtkTransform.h>
#include <vtkCamera.h>
#include <vtkWindowToImageFilter.h>
#include <vtkSTLWriter.h>
#include <vtkTransformPolyDataFilter.h>
#include "vtk_tools.h"
#include "HipDrrPlanner.h"
#include "DRRImageProcess.h"
#include <time.h>

#include <thread>
#include <future>


CTImageType::Pointer cal_masked_image_thread(XRayImageType::Pointer image_mask, CTImageType::Pointer ctimage)
{
	//spacing会有精度引起的误差，MultiplyImageFilter要求两个输入spacing严格相同
	CTImageType::SpacingType target_spacing = image_mask->GetSpacing();
	ctimage->SetSpacing(target_spacing);

	using FilterType = itk::MultiplyImageFilter<XRayImageType, CTImageType, CTImageType>;
	auto filter = FilterType::New();
	filter->SetInput1(image_mask);
	filter->SetInput2(ctimage);
	filter->Update();
	//write_test_result(filter->GetOutput());
	return filter->GetOutput();
}

XRayImageType::Pointer binarized_masked_image_thread(XRayImageType::Pointer in_ct_image)
{
	//将图像中所有大于threshold的像素替换为constantValue，这里将mask的255转换为1
	CTPixelType threshold = 1;
	CTPixelType constantValue = 1;
	using BinaryThresholdFilterType = itk::BinaryThresholdImageFilter<XRayImageType, XRayImageType>;
	BinaryThresholdFilterType::Pointer thresholdFilter = BinaryThresholdFilterType::New();
	thresholdFilter->SetInput(in_ct_image);
	thresholdFilter->SetLowerThreshold(threshold);
	thresholdFilter->SetInsideValue(constantValue);
	thresholdFilter->SetOutsideValue(0);
	thresholdFilter->Update();

	// 获取输出图像
	return thresholdFilter->GetOutput();
}

XRayImageType::Pointer combine_mask_image_thread(std::vector<XRayImageType::Pointer> mask_list)
{
	/************************************************************************/
	/* 函数功能：将多个mask叠加(并集)
	/* 函数参数：包含所有待叠加mask的vector
	/* 函数返回：叠加完成后的mask图像文件
	/* 函数说明：在进行DRR时将各个骨骼的mask结合在一起做一次DRR节约算力。itkOrImageFilter更合适但不在当前部署环境中，以AddImageFilter代替。
	/* 两种bad cases:规划界面未选择mask的类别则mask_list都是null pointer，此时直接用CT做DRR。只选择了部分mask类别，则部分mask_list为空
	/* 编 写 人：宋云鹏
	/* 编写时间：20240403
	/************************************************************************/
	int num_drr = mask_list.size();
	if (mask_list.size() == 1)
	{
		return mask_list.at(0);
	}

	typedef itk::AddImageFilter <XRayImageType, XRayImageType>	ADDFilterType;
	ADDFilterType::Pointer AddFilter = ADDFilterType::New();
	AddFilter->SetInput1(mask_list.at(0));
	XRayImageType::Pointer stucked_image;

	for (int i = 1; i < num_drr; i++)
	{
		AddFilter->SetInput2(mask_list.at(i));
		AddFilter->Update();
		stucked_image = AddFilter->GetOutput();
		AddFilter->SetInput1(stucked_image);
	}

	return stucked_image;
}


void HipDrrPlanner::init_drr_reset_planner()
{
/************************************************************************/
/* 函数功能：初始化在DRR上摆正股骨所需的数据，对左右侧股骨的图像数据根据传入的变换矩阵在物理空间进行变换后得到完整的CT
/* 函数参数：void
/* 函数返回：void
/* 函数说明：void
/* 编 写 人：宋云鹏
/* 编写时间：20240514
/************************************************************************/
	time_t begin_t = clock();

	if (m_left_femur_mask == nullptr || m_right_femur_mask == nullptr || m_left_hip_mask == nullptr || m_right_hip_mask == nullptr)
	{
		set_whole_reset_masked_image(m_ct_image);
	}
	else 
	{
		convert_vtk_to_itk_bounds(get_right_femur_bbox(), right_f_bbox_itk);
		convert_vtk_to_itk_bounds(get_left_femur_bbox(), left_f_bbox_itk);
		convert_vtk_to_itk_bounds(get_transformed_left_femur_bbox(), transformed_left_f_bbox_itk);
		convert_vtk_to_itk_bounds(get_transformed_right_femur_bbox(), transformed_right_f_bbox_itk);

		std::vector<CTImageType::Pointer> masked_list;
		std::vector<XRayImageType::Pointer> hip_mask_list;
		//初始化左右股骨和髋总计三个图像数据，这三个图像数据再算法中作为输入不参与变换，只需要一次计算即可
		if (m_left_femur_masked_image == nullptr)
		{
			set_left_femur_masked_image(cal_masked_image(binarized_masked_image(m_left_femur_mask), m_ct_image));
		}
		if (m_right_femur_masked_image == nullptr)
		{
			set_right_femur_masked_image(cal_masked_image(binarized_masked_image(m_right_femur_mask), m_ct_image));
		}
		if (m_whole_hip_masked_image == nullptr)
		{
			hip_mask_list.push_back(m_left_hip_mask);
			hip_mask_list.push_back(m_right_hip_mask);
			set_whole_hip_masked_image(cal_masked_image(binarized_masked_image(combine_mask_image(hip_mask_list)), m_ct_image));
		}
		masked_list.push_back(get_whole_hip_masked_image());

		if (m_cropped_left_femur_masked_image == nullptr || left_reset_matrix_change || m_left_femur_reset_masked_image == nullptr)
		{
			time_t crop_begin_t = clock();
			m_cropped_left_femur_masked_image = crop_ct_image(m_left_femur_masked_image, m_left_femur_mask, left_f_bbox_itk);
			//write_nrrd_test_result(m_cropped_left_femur_masked_image, "./cropped_left_femur_masked_image.nrrd");
			set_left_femur_reset_masked_image(transform_ct_image<CTImageType>(m_cropped_left_femur_masked_image, m_left_femur_reset_matrix, "left"));
			time_t crop_end_t = clock();
			cout << "crop amd tramsform a ct image cost " << (double)(crop_end_t - crop_begin_t) / CLOCKS_PER_SEC << " s" << endl;
		}

		masked_list.push_back(get_left_femur_reset_masked_image());

		if (m_cropped_right_femur_masked_image == nullptr || right_reset_matrix_change || m_right_femur_reset_masked_image == nullptr)
		{
			m_cropped_right_femur_masked_image = crop_ct_image(m_right_femur_masked_image, m_right_femur_mask, right_f_bbox_itk);
			//write_nrrd_test_result(m_cropped_right_femur_masked_image, "./cropped_right_femur_masked_image.nrrd");
			set_right_femur_reset_masked_image(transform_ct_image<CTImageType>(m_cropped_right_femur_masked_image, m_right_femur_reset_matrix, "right"));
		}
		masked_list.push_back(get_right_femur_reset_masked_image());

		//	write_nrrd_test_result(get_left_femur_reset_masked_image(), "./transformed_left.nrrd");
		   //write_nrrd_test_result(get_right_femur_reset_masked_image(), "./transformed_right.nrrd");

		set_whole_reset_masked_image(combine_masked_image(masked_list));
		time_t reset_end_t = clock();
		cout << "init reset mode cost " << (double)(reset_end_t - begin_t) / CLOCKS_PER_SEC << " s" << endl;
	}
	
}

void HipDrrPlanner::init_drr_planner()
{
	/************************************************************************/
	/* 函数功能：初始化DRR可视化相关内容
	/* 函数参数：void	
	/* 函数返回：void	
	/* 函数说明：void
	/* 编 写 人：宋云鹏
	/* 编写时间：20240514
	/************************************************************************/
	//time_t begin_t = clock();
	m_renderWindow->SetSize(1024, 1024);
	m_renderWindow->OffScreenRenderingOn();
	time_t end_drr_t = clock();
	//cout << "init drr cost " << (double)(end_drr_t - begin_t) / CLOCKS_PER_SEC << " s" << endl;
	//没有传入mask则使用原ct作为drr输入
	if (m_left_femur_mask == nullptr || m_right_femur_mask == nullptr || m_left_hip_mask ==nullptr || m_right_hip_mask ==nullptr)
	{
		set_whole_masked_image(m_ct_image);
	}
	else
	{
		//为方便摆正，对左右股骨和整体髋分别使用mask得到roi的ct图像数据
		std::vector<CTImageType::Pointer> masked_list;
		time_t bin_start_t = clock();
		XRayImageType::Pointer bin_left_femur_mask = binarized_masked_image(m_left_femur_mask);
		time_t bin_end_t = clock();
		cout << "binalize a mask cost " << (double)(bin_end_t - bin_start_t) / CLOCKS_PER_SEC << " s" << endl;

		CTImageType::Pointer left_femur_masked = cal_masked_image(bin_left_femur_mask, m_ct_image);
		time_t masked_end_t = clock();
		cout << "cal masked image cost " << (double)(masked_end_t - bin_end_t) / CLOCKS_PER_SEC << " s" << endl;

		//CTImageType::Pointer left_femur_masked2 = get_masked_image(m_left_femur_mask, m_ct_image);
		//time_t masked2_end_t = clock();
		//cout << "cal masked image iterator cost " << (double)(masked2_end_t - masked_end_t) / CLOCKS_PER_SEC << " s" << endl;
		//write_nrrd_test_result(left_femur_masked2, "./left_femur_masked2.nrrd");
		set_left_femur_masked_image(left_femur_masked);
		masked_list.push_back(get_left_femur_masked_image());

		set_right_femur_masked_image(cal_masked_image(binarized_masked_image(m_right_femur_mask), m_ct_image));
		masked_list.push_back(get_right_femur_masked_image());

		std::vector<XRayImageType::Pointer> hip_mask_list;
		hip_mask_list.push_back(m_left_hip_mask);
		hip_mask_list.push_back(m_right_hip_mask);
		set_whole_hip_masked_image(cal_masked_image(binarized_masked_image(combine_mask_image(hip_mask_list)), m_ct_image));
		masked_list.push_back(get_whole_hip_masked_image());

		time_t combine_start_t = clock();
		set_whole_masked_image(combine_masked_image(masked_list));
		time_t combine_end_t = clock();
		cout << "combine masked image cost " << (double)(combine_end_t - combine_start_t) / CLOCKS_PER_SEC << " s" << endl;
	}
	time_t mask_ct_t = clock();
	//cout << "cal masked ct cost " << (double)(mask_ct_t - end_drr_t) / CLOCKS_PER_SEC << " s" << endl;
	cout << "whole init process takes " << (double)(mask_ct_t - end_drr_t) / CLOCKS_PER_SEC << endl;
}

void HipDrrPlanner::init_drr_planner_threads()
{
	/************************************************************************/
	/* 函数功能：初始化DRR可视化相关内容
	/* 函数参数：void
	/* 函数返回：void
	/* 函数说明：void
	/* 编 写 人：宋云鹏
	/* 编写时间：20240514
	/************************************************************************/

	//time_t begin_t = clock();
	m_renderWindow->SetSize(512, 512);
	m_renderWindow->OffScreenRenderingOn();
	time_t end_drr_t = clock();
	//cout << "init drr cost " << (double)(end_drr_t - begin_t) / CLOCKS_PER_SEC << " s" << endl;

	//没有传入mask则使用原ct作为drr输入
	if (m_left_femur_mask == nullptr && m_right_femur_mask == nullptr && m_left_hip_mask == nullptr && m_right_hip_mask == nullptr)
	{
		set_whole_masked_image(m_ct_image);
	}
	else
	{
		//为方便摆正，对左右股骨和整体髋分别使用mask得到roi的ct图像数据
		std::vector<CTImageType::Pointer> masked_list;
		std::vector<XRayImageType::Pointer> hip_mask_list;
		hip_mask_list.push_back(m_left_hip_mask);
		hip_mask_list.push_back(m_right_hip_mask);
	

		std::future<XRayImageType::Pointer> whole_hip_mask_res = std::async(std::launch::async, combine_mask_image_thread, hip_mask_list);
		std::future<XRayImageType::Pointer> result = std::async(std::launch::async, binarized_masked_image_thread, m_left_femur_mask);
		std::future<XRayImageType::Pointer> result1 = std::async(std::launch::async, binarized_masked_image_thread, m_right_femur_mask);

		XRayImageType::Pointer m_bin_left_femur_mask = result.get();
		XRayImageType::Pointer m_bin_right_femur_mask = result1.get();
		XRayImageType::Pointer whole_hip_mask = binarized_masked_image(whole_hip_mask_res.get());


		std::future<CTImageType::Pointer> left_femur_masked_res = std::async(std::launch::async, cal_masked_image_thread, m_bin_left_femur_mask, m_ct_image);
		std::future<CTImageType::Pointer> right_femur_masked_res = std::async(std::launch::async, cal_masked_image_thread, m_bin_right_femur_mask, m_ct_image);
		std::future<CTImageType::Pointer> hip_masked_res = std::async(std::launch::async, cal_masked_image_thread, whole_hip_mask, m_ct_image);
		CTImageType::Pointer left_femur_masked = left_femur_masked_res.get();
		CTImageType::Pointer right_femur_masked = right_femur_masked_res.get();
		CTImageType::Pointer hip_masked = hip_masked_res.get();

		set_left_femur_masked_image(left_femur_masked);
		masked_list.push_back(get_left_femur_masked_image());

		set_right_femur_masked_image(right_femur_masked);
		masked_list.push_back(get_right_femur_masked_image());

	
		set_whole_hip_masked_image(hip_masked);
		masked_list.push_back(get_whole_hip_masked_image());

		set_whole_masked_image(combine_masked_image(masked_list));

	}
	time_t mask_ct_t = clock();
//	cout << "cal masked ct2 cost " << (double)(mask_ct_t - end_drr_t) / CLOCKS_PER_SEC << " s" << endl;
	cout <<  (double)(mask_ct_t - end_drr_t) / CLOCKS_PER_SEC << endl;
}

void HipDrrPlanner::run_drr_planner(bool reset)
{
	time_t start_drr_t = clock();
	DRRImageProcess drrProcess;
	drrProcess.setrotation_angle(rx, ry, rz);

	if (reset)
	{
		m_whole_drr_image = drrProcess.DRRProcess(m_whole_reset_masked_image, m_focalpoint);
		//write_test_result(m_whole_drr_image, "./whole_drr.nii.gz");
	}
	else
	{
		m_whole_drr_image = drrProcess.DRRProcess(m_whole_masked_image, m_focalpoint);
	}

	m_drr_transform_matrix = drrProcess.drr_transform_matrix;

	//write_nrrd_test_result(m_whole_drr_image, "./whole_drr.nrrd");
	time_t start_render_t = clock();

	cout << "cal drr cost " << (double)(start_render_t - start_drr_t) / CLOCKS_PER_SEC << " s" << endl;
	drr_bone_implant_render(m_whole_drr_image);

	time_t end_render_t = clock();
	cout << "render drr cost " << (double)(end_render_t - start_render_t) / CLOCKS_PER_SEC << " s" << endl;

	string c = reset ? "T" : "F";
	auto writer = vtkSmartPointer<vtkPNGWriter>::New();
	
	string out_file_dir = "screem_shot_" + c + "_" + std::to_string(rx).substr(0, std::to_string(rx).find(".", 0) + 2) + "_" +
		std::to_string(ry).substr(0, std::to_string(ry).find(".", 0) + 2) + "_" + std::to_string(rz).substr(0, std::to_string(rz).find(".", 0) + 2) + ".png";
	writer->SetFileName(out_file_dir.c_str());
	writer->SetInputData(m_render_result);
	writer->Write();
	process_render_result();
	time_t end_postprocess_t = clock();
	cout << "postprocess cost " << (double)(end_postprocess_t - end_render_t) / CLOCKS_PER_SEC << " s" << endl;
}

void HipDrrPlanner::process_render_result()
{
	int dims[3];
	m_render_result->GetDimensions(dims);
	//int width = dims[0];
	//int height = dims[1];
	//int depth = dims[2];
	//std::cout << "w :" << dims[0] << "h :" << dims[1] << "d :" << dims[2] << std::endl;
	//int scalarType = m_render_result->GetScalarType();


	for (int z = 0; z < dims[2]; z++)
	{
		for (int y = 0; y < dims[1]; y++)
		{
			for (int x = 0; x < dims[0]; x++)
			{
				unsigned char* pixel = static_cast<unsigned char*>(m_render_result->GetScalarPointer(x, y, z));
				if (pixel[0] != 0 && pixel[1] == 0 && pixel[2] == 0 )
				{
					pixel[0] = 254;
					pixel[1] = 254;
					pixel[2] = 254;
				}
			
			}
		}
	}
	m_render_result->Modified();
	auto writer = vtkSmartPointer<vtkPNGWriter>::New();

	string out_file_dir = "./test.png";
	writer->SetFileName(out_file_dir.c_str());
	writer->SetInputData(m_render_result);
	writer->Write();
}

void HipDrrPlanner::set_angle(float rotation_x, float rotation_y, float rotation_z)
{
	rx = rotation_x;
	ry = rotation_y;
	rz = rotation_z;
}

XRayImageType::Pointer HipDrrPlanner::combine_mask_image(std::vector<XRayImageType::Pointer> mask_list)
{
/************************************************************************/
/* 函数功能：将多个mask叠加(并集)
/* 函数参数：包含所有待叠加mask的vector
/* 函数返回：叠加完成后的mask图像文件
/* 函数说明：在进行DRR时将各个骨骼的mask结合在一起做一次DRR节约算力。itkOrImageFilter更合适但不在当前部署环境中，以AddImageFilter代替。
/* 两种bad cases:规划界面未选择mask的类别则mask_list都是null pointer，此时直接用CT做DRR。只选择了部分mask类别，则部分mask_list为空
/* 编 写 人：宋云鹏
/* 编写时间：20240403
/************************************************************************/
	int num_drr = mask_list.size();
	if (mask_list.size() == 1)
	{
		return mask_list.at(0);
	}

	typedef itk::AddImageFilter <XRayImageType, XRayImageType>	ADDFilterType;
	ADDFilterType::Pointer AddFilter = ADDFilterType::New();
	AddFilter->SetInput1(mask_list.at(0));
	XRayImageType::Pointer stucked_image;

	for (int i=1; i<num_drr; i++)
	{
		AddFilter->SetInput2(mask_list.at(i));
		AddFilter->Update();
		stucked_image = AddFilter->GetOutput();
	/*	string out_name = std::to_string(i)+ "stucked_mask.nii.gz";
		write_test_result(stucked_image, out_name);*/

		//ADDFilterType::Pointer AddFilter = ADDFilterType::New();
		AddFilter->SetInput1(stucked_image);
	}

	return stucked_image;
}

CTImageType::Pointer HipDrrPlanner::combine_masked_image(std::vector<CTImageType::Pointer> mask_list)
{
	/************************************************************************/
	/* 函数功能：将多个mask叠加(并集)
	/* 函数参数：包含所有待叠加mask的vector
	/* 函数返回：叠加完成后的mask图像文件
	/* 函数说明：在进行DRR时将各个骨骼的mask结合在一起做一次DRR节约算力。itkOrImageFilter更合适但不在当前部署环境中，以AddImageFilter代替。
	/* 两种bad cases:规划界面未选择mask的类别则mask_list都是null pointer，此时直接用CT做DRR。只选择了部分mask类别，则部分mask_list为空
	/* 编 写 人：宋云鹏
	/* 编写时间：20240403
	/************************************************************************/
	int num_drr = mask_list.size();
	if (mask_list.size() == 1)
	{
		return mask_list.at(0);
	}

	typedef itk::AddImageFilter <CTImageType, CTImageType>	ADDFilterType;
	ADDFilterType::Pointer AddFilter = ADDFilterType::New();
	AddFilter->SetInput1(mask_list.at(0));
	CTImageType::Pointer stucked_image;

	for (int i = 1; i < num_drr; i++)
	{
		AddFilter->SetInput2(mask_list.at(i));
		AddFilter->Update();
		stucked_image = AddFilter->GetOutput();
		AddFilter->SetInput1(stucked_image);
	}

	return stucked_image;
}

CTImageType::Pointer HipDrrPlanner::binarized_masked_image(CTImageType::Pointer in_ct_image, CTPixelType threshold, CTPixelType constantValue)
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

XRayImageType::Pointer HipDrrPlanner::binarized_masked_image(XRayImageType::Pointer in_ct_image, CTPixelType threshold, CTPixelType constantValue)
{
	//将图像中所有大于threshold的像素替换为constantValue，这里将mask的255转换为1
	using BinaryThresholdFilterType = itk::BinaryThresholdImageFilter<XRayImageType, XRayImageType>;
	BinaryThresholdFilterType::Pointer thresholdFilter = BinaryThresholdFilterType::New();
	thresholdFilter->SetInput(in_ct_image);
	thresholdFilter->SetLowerThreshold(threshold);
	thresholdFilter->SetInsideValue(constantValue);
	thresholdFilter->SetOutsideValue(0);
	thresholdFilter->Update();

	// 获取输出图像
	XRayImageType::Pointer outputImage = thresholdFilter->GetOutput();
	//write_test_result(outputImage);

	return outputImage;
}



CTImageType::Pointer HipDrrPlanner::cal_masked_image(XRayImageType::Pointer image_mask, CTImageType::Pointer ctimage)
{
	//spacing会有精度引起的误差，MultiplyImageFilter要求两个输入spacing严格相同
	CTImageType::SpacingType target_spacing = image_mask->GetSpacing();
	ctimage->SetSpacing(target_spacing);

	using FilterType = itk::MultiplyImageFilter<XRayImageType, CTImageType, CTImageType>;
	auto filter = FilterType::New();
	filter->SetInput1(image_mask);
	filter->SetInput2(ctimage);
	filter->Update();
	//write_test_result(filter->GetOutput());
	return filter->GetOutput();
}

CTImageType::Pointer HipDrrPlanner::copy_image(CTImageType::Pointer target_image)
{
	using DuplicatorType = itk::ImageDuplicator<CTImageType>;
	auto duplicator = DuplicatorType::New();
	duplicator->SetInputImage(target_image);
	duplicator->Update();
	CTImageType::Pointer clonedImage = duplicator->GetOutput();
	return clonedImage;
}

XRayImageType::Pointer HipDrrPlanner::copy_image(XRayImageType::Pointer target_image)
{
	using DuplicatorType = itk::ImageDuplicator<XRayImageType>;
	auto duplicator = DuplicatorType::New();
	duplicator->SetInputImage(target_image);
	duplicator->Update();
	XRayImageType::Pointer clonedImage = duplicator->GetOutput();
	return clonedImage;
}


vtkSmartPointer<vtkImageData> HipDrrPlanner::itk_image2_vtk(CTImageType::Pointer in_image)
{
	//write_test_result(in_image, "./test.nii.gz");
	CTImageType::DirectionType direction = in_image->GetDirection();
	CTImageType::DirectionType ori_direction = m_ct_image->GetDirection();
	CTImageType::PointType origin =  in_image->GetOrigin();

	using FlipFilterType = itk::FlipImageFilter<CTImageType>;
	FlipFilterType::Pointer flipperImage = FlipFilterType::New();
	bool flipAxes[3] = { false, true, false };
	flipperImage = FlipFilterType::New();
	flipperImage->SetFlipAxes(flipAxes);
	flipperImage->SetInput(in_image);
	flipperImage->Update();

//	using RescaleFilterType = itk::RescaleIntensityImageFilter<
//	CTImageType, CTImageType >;
//RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
//rescaler->SetOutputMinimum(0);
//rescaler->SetOutputMaximum(255);
//rescaler->SetInput(filter->GetOutput());
//rescaler->Update();

	using FilterType = itk::ImageToVTKImageFilter<CTImageType>;
	auto filter = FilterType::New();
	//filter->SetInput(flipperImage->GetOutput());
	filter->SetInput(in_image);
	filter->Update();

	vtkImageData *myvtkImageData = vtkImageData::New();
	vtkSmartPointer<vtkImageData> d  = filter->GetOutput();
	double myorigin[3] = {origin[0], origin[1], origin[2]};
	//d->SetOrigin(myorigin);
	myvtkImageData->DeepCopy(d);
	return myvtkImageData;
}

//将vtkimagedata转换为itkimagedata
CTImageType::Pointer HipDrrPlanner::vtk_image2_itk(vtkImageData* in_Image)
{
	using FilterType = itk::VTKImageToImageFilter<CTImageType>;
	FilterType::Pointer filter = FilterType::New();
	filter->SetInput(in_Image);
	filter->Update();
	CTImageType::Pointer itkImage = filter->GetOutput();
	
	return itkImage;
}

void HipDrrPlanner::write_test_result(CTImageType::Pointer ct_image, string out_path)
{
	using WriterType = itk::ImageFileWriter<CTImageType>;
	using ImageIOType = itk::NiftiImageIO;

	WriterType::Pointer writer = WriterType::New();
	ImageIOType::Pointer niiIO = ImageIOType::New();

	writer->SetInput(ct_image);
	writer->SetFileName(out_path);
	writer->UseCompressionOn();
	writer->SetImageIO(niiIO);
	writer->Update();
}

void HipDrrPlanner::write_nrrd_test_result(CTImageType::Pointer ct_image, string out_path)
{
	using NRRD_WriterFilter = itk::ImageFileWriter<CTImageType>;
	NRRD_WriterFilter::Pointer writer = NRRD_WriterFilter::New();

	itk::NrrdImageIOFactory::RegisterOneFactory();
	writer->SetInput(ct_image);
	writer->SetFileName(out_path);
	writer->Update();

	//CTImageType::Pointer inputImage = writer->GetOutput();
	//return inputImage;
}

void HipDrrPlanner::write_nrrd_test_result(XRayImageType::Pointer ct_image, string out_path)
{
	using NRRD_WriterFilter = itk::ImageFileWriter<XRayImageType>;
	NRRD_WriterFilter::Pointer writer = NRRD_WriterFilter::New();

	itk::NrrdImageIOFactory::RegisterOneFactory();
	writer->SetInput(ct_image);
	writer->SetFileName(out_path);
	writer->Update();

	//CTImageType::Pointer inputImage = writer->GetOutput();
	//return inputImage;
}

void HipDrrPlanner::write_test_result(XRayImageType::Pointer ct_image, string out_path)
{
	using WriterType = itk::ImageFileWriter<XRayImageType>;
	using ImageIOType = itk::NiftiImageIO;

	WriterType::Pointer writer = WriterType::New();
	ImageIOType::Pointer niiIO = ImageIOType::New();

	writer->SetInput(ct_image);
	writer->SetFileName(out_path);
	writer->UseCompressionOn();
	writer->SetImageIO(niiIO);
	writer->Update();
}

vtkSmartPointer<vtkImageData> HipDrrPlanner::itk_image2_vtk(XRayImageType::Pointer in_image)
{
	//write_test_result(in_image, "./test.nii.gz");
	XRayImageType::DirectionType direction = in_image->GetDirection();
	XRayImageType::DirectionType ori_direction = m_ct_image->GetDirection();
	XRayImageType::PointType origin = in_image->GetOrigin();

	using FlipFilterType = itk::FlipImageFilter<XRayImageType>;
	FlipFilterType::Pointer flipperImage = FlipFilterType::New();
	bool flipAxes[3] = { false, true, false };
	flipperImage = FlipFilterType::New();
	flipperImage->SetFlipAxes(flipAxes);
	flipperImage->SetInput(in_image);
	flipperImage->Update();

	//	using RescaleFilterType = itk::RescaleIntensityImageFilter<
	//	CTImageType, CTImageType >;
	//RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
	//rescaler->SetOutputMinimum(0);
	//rescaler->SetOutputMaximum(255);
	//rescaler->SetInput(filter->GetOutput());
	//rescaler->Update();

	using FilterType = itk::ImageToVTKImageFilter<XRayImageType>;
	auto filter = FilterType::New();
	//filter->SetInput(flipperImage->GetOutput());
	filter->SetInput(in_image);
	filter->Update();

	vtkImageData *myvtkImageData = vtkImageData::New();
	vtkSmartPointer<vtkImageData> d = filter->GetOutput();
	double myorigin[3] = { origin[0], origin[1], origin[2] };
	//d->SetOrigin(myorigin);
	myvtkImageData->DeepCopy(d);
	return myvtkImageData;
}

XRayImageType::Pointer HipDrrPlanner::mask_overlap_filter(std::vector<XRayImageType::Pointer> drr_stuck_list)
{
	///************************************************************************/
	///* 函数功能：将各个骨骼的CT mask合并为一个整体的mask
	///* 函数参数：包含所有待叠加mask的vector,unsigned shor
	///* 函数返回：叠加完成后的mask图像文件,unsigned shor
	///* 函数说明：直接用ImageRegionIterator太慢，四张CT需要8s以上
	///* 编 写 人：宋云鹏
	///* 编写时间：20240407
	///************************************************************************/
	int num_drr = drr_stuck_list.size();
	m_whole_drr_image = copy_image(drr_stuck_list.at(0));

	itk::ImageRegionIterator<XRayImageType> it0(m_whole_drr_image, m_whole_drr_image->GetLargestPossibleRegion());
	std::vector<itk::ImageRegionIterator<XRayImageType>> drr_iterators;

	for (int m = 1; m < drr_stuck_list.size(); m++)
	{
		XRayImageType::Pointer img_tmp = drr_stuck_list.at(m);
		itk::ImageRegionIterator<XRayImageType> it(img_tmp, img_tmp->GetLargestPossibleRegion());
		drr_iterators.push_back(it);
	}

	while (!drr_iterators.at(0).IsAtEnd())
	{
		for (int j = 0; j < drr_iterators.size(); j++)
		{
			it0.Set(it0.Get() + drr_iterators.at(j).Get());
			++drr_iterators.at(j);
		}
		++it0;
	}

	return m_whole_drr_image;
}

XRayImageType::Pointer HipDrrPlanner::mask_overlap(std::vector<XRayImageType::Pointer> drr_stuck_list)
{
	///************************************************************************/
	///* 函数功能：将各个骨骼的CT mask合并为一个整体的mask
	///* 函数参数：包含所有待叠加mask的vector,unsigned shor
	///* 函数返回：叠加完成后的mask图像文件,unsigned shor
	///* 函数说明：直接用ImageRegionIterator太慢，四张CT需要8s以上
	///* 编 写 人：宋云鹏
	///* 编写时间：20240407
	///************************************************************************/
	int num_drr = drr_stuck_list.size();
	m_whole_drr_image = copy_image(drr_stuck_list.at(0));

	itk::ImageRegionIterator<XRayImageType> it0(m_whole_drr_image, m_whole_drr_image->GetLargestPossibleRegion());
	std::vector<itk::ImageRegionIterator<XRayImageType>> drr_iterators;

	for (int m = 1; m < drr_stuck_list.size(); m++)
	{
		XRayImageType::Pointer img_tmp = drr_stuck_list.at(m);
		itk::ImageRegionIterator<XRayImageType> it(img_tmp, img_tmp->GetLargestPossibleRegion());
		drr_iterators.push_back(it);
	}

	while (!drr_iterators.at(0).IsAtEnd())
	{
		for (int j = 0; j < drr_iterators.size(); j++)
		{
			it0.Set(it0.Get() + drr_iterators.at(j).Get());
			++drr_iterators.at(j);
		}
		++it0;
	}

	return m_whole_drr_image;
}


XRayImageType::Pointer HipDrrPlanner::drr_overlap(std::vector<XRayImageType::Pointer> drr_stuck_list)
{
///************************************************************************/
///* 函数功能：分别对每个股骨component做drr，总的drr就是各个部位drr加在一起后。
///* 函数参数：包含所有待叠加drr的vector,unsigned shor
///* 函数返回：叠加完成后的drr图像文件,unsigned shor
///* 函数说明：unsigned shor类型的drr在按位相加时会发生溢出，因此要进行平均值相加：val/num_images
///* 编 写 人：宋云鹏
///* 编写时间：20240407
///************************************************************************/
 	int num_drr = drr_stuck_list.size();
	m_whole_drr_image = copy_image(drr_stuck_list.at(0));

	using RescaleFilterType = itk::RescaleIntensityImageFilter<XRayImageType, XRayImageType>;
	RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
	rescaler->SetOutputMinimum(0);
	rescaler->SetOutputMaximum(255/num_drr);
	rescaler->SetInput(m_whole_drr_image);
	rescaler->Update();
	m_whole_drr_image = rescaler->GetOutput();

	itk::ImageRegionIterator<XRayImageType> it0(m_whole_drr_image, m_whole_drr_image->GetLargestPossibleRegion());
	std::vector<itk::ImageRegionIterator<XRayImageType>> drr_iterators;

	for (int m = 1; m < drr_stuck_list.size(); m++)
	{
		XRayImageType::Pointer img_tmp = drr_stuck_list.at(m);
		itk::ImageRegionIterator<XRayImageType> it(img_tmp, img_tmp->GetLargestPossibleRegion());
		drr_iterators.push_back(it);
	}

	while (!drr_iterators.at(0).IsAtEnd())
	{
		for (int j = 0; j < drr_iterators.size(); j++)
		{
			it0.Set(it0.Get() + drr_iterators.at(j).Get()/num_drr);
			++drr_iterators.at(j);
		}
		++it0;
	}

	rescaler->SetOutputMinimum(0);
	rescaler->SetOutputMaximum(255);
	rescaler->SetInput(m_whole_drr_image);
	rescaler->Update();

	/*typedef itk::PNGImageIO PNGImageIOType;
	PNGImageIOType::Pointer rawImageIO = PNGImageIOType::New();

	using WriterType = itk::ImageFileWriter<CTImageType>;
	WriterType::Pointer writer = WriterType::New();
	writer->SetImageIO(rawImageIO);
	writer->SetFileName(std::to_string(rx).substr(0, std::to_string(rx).find(".", 0) + 2) + "_" + std::to_string(ry).substr(0, std::to_string(ry).find(".", 0) + 2) +
		std::to_string(rz).substr(0, std::to_string(rz).find(".", 0) + 2) + ".png");
	writer->SetInput(rescaler->GetOutput());
	writer->Update();*/

	//write_test_result(m_whole_drr_image, "./whole.nii.gz");
	//write_test_result(rescaler->GetOutput(), "./whole2.nii.gz");
	return rescaler->GetOutput();
}

void HipDrrPlanner::vtk_polydata_to_image(vtkSmartPointer<vtkPolyData> target_polydata, vtkSmartPointer<vtkImageData> out_imagedata)
{
	//vtkNew<vtkActor> target_polydata_actor;
	//vtkNew<vtkPolyDataMapper> boundary_mapper;
	//boundary_mapper->SetInputData(target_polydata);
	//target_polydata_actor->SetMapper(boundary_mapper);
	//std::vector<vtkActor*> all_actors;
	//all_actors.push_back(target_polydata_actor);
	//show_actors(all_actors);

	vtkNew<vtkImageData> whiteImage;
	//double bounds[6];
	//target_polydata->GetBounds(bounds);
	//const CTImageType::SpacingType& m_x_spacing = m_ct_image->GetSpacing();
	double spacing[3]; // desired volume spacing
	spacing[0] = m_ori_spacing[0];
	spacing[1] = m_ori_spacing[1];
	spacing[2] = m_ori_spacing[2];
	whiteImage->SetSpacing(spacing);

	// compute dimensions
	int dim[3];
	dim[0] = m_ori_size[0];
	dim[1] = m_ori_size[1];
	dim[2] = m_ori_size[2];
	whiteImage->SetDimensions(dim);
	whiteImage->SetExtent(0, dim[0] - 1, 0, dim[1] - 1, 0, dim[2] - 1);

	double origin[3];
	CTImageType::PointType ori_origin = m_ct_image->GetOrigin();
	//origin[0] = bounds[0] + spacing[0] / 2;
	//origin[1] = bounds[2] + spacing[1] / 2;
	//origin[2] = bounds[4] + spacing[2] / 2;
	origin[0] = ori_origin[0];
	origin[1] = ori_origin[1];
	origin[2] = ori_origin[2];
	whiteImage->SetOrigin(origin);
	whiteImage->AllocateScalars(VTK_SHORT, 1);

	// Fill the image with foreground voxels:
	unsigned char inval = 355;
	unsigned char outval = 0;
	vtkIdType count = whiteImage->GetNumberOfPoints();
	for (vtkIdType i = 0; i < count; ++i)
	{
		whiteImage->GetPointData()->GetScalars()->SetTuple1(i, inval);
	}

	// polygonal data --> image stencil:
	vtkNew<vtkPolyDataToImageStencil> pol2stenc;
	pol2stenc->SetInputData(target_polydata);
	pol2stenc->SetOutputOrigin(origin);
	pol2stenc->SetOutputSpacing(spacing);
	pol2stenc->SetOutputWholeExtent(whiteImage->GetExtent());
	pol2stenc->Update();

	// Cut the corresponding white image and set the background:
	vtkNew<vtkImageStencil> imgstenc;
	imgstenc->SetInputData(whiteImage);
	imgstenc->SetStencilConnection(pol2stenc->GetOutputPort());
	imgstenc->ReverseStencilOff();
	imgstenc->SetBackgroundValue(outval);
	imgstenc->Update();

	out_imagedata->DeepCopy(imgstenc->GetOutput());
}

void HipDrrPlanner::single_implant_render(CTImageType::Pointer target_data)
{
	CTImageType::SizeType size = target_data->GetLargestPossibleRegion().GetSize();
	vtkSmartPointer<vtkImageData> target_data_vtk = itk_image2_vtk(target_data);
	vtkNew<vtkImageActor> vtk_drr_actor;
	vtk_drr_actor->GetMapper()->SetInputData(target_data_vtk);

	vtkNew<vtkTransform> drr_image_transform;
	drr_image_transform->SetMatrix(m_drr_transform_matrix);
	vtk_drr_actor->SetUserTransform(drr_image_transform);

	float my_focal_point[3] = { (float)m_focalpoint[0], (float)m_focalpoint[1], (float)m_focalpoint[2] };
	float focal_point_new[3];
	drr_image_transform->TransformPoint(my_focal_point, focal_point_new);

	vtkNew<vtkActor> focal_point_actor;
	createSphereActor(focal_point_new, 5, 1, "Pink", focal_point_actor);

	//计算相机的focal point
	double drr_image_origin[3];
	double drr_image_origin_new[3];
	vtk_drr_actor->GetMapper()->GetInput()->GetOrigin(drr_image_origin);
	drr_image_transform->TransformPoint(drr_image_origin, drr_image_origin_new);

	drr_image_origin_new[2] = drr_image_origin_new[2] - size[0] / 2;
	drr_image_origin_new[0] = drr_image_origin_new[0] + size[0] / 2;

	vtkSmartPointer<vtkRenderer> originalRenderer = vtkSmartPointer<vtkRenderer>::New();

	//originalRenderer->AddActor(m_femur_stem_actor);
	//originalRenderer->AddActor(m_femur_head_actor);
	originalRenderer->AddActor(m_hip_cup_actor);

	vtkNew<vtkCamera> my_camera;
	my_camera->SetPosition(focal_point_new[0], focal_point_new[1], focal_point_new[2]);
	my_camera->SetFocalPoint(drr_image_origin_new[0], drr_image_origin_new[1], drr_image_origin_new[2]);
	my_camera->SetViewUp(0, 0, 1);

	double clip_max_new = abs(drr_image_origin_new[1] - focal_point_new[1]) *1.1;
	my_camera->SetClippingRange(0.01, clip_max_new);
	originalRenderer->SetActiveCamera(my_camera);

	vtkNew<vtkRenderWindow> renderWindow;
	renderWindow->SetSize(512, 512);
	renderWindow->AddRenderer(originalRenderer);
	renderWindow->OffScreenRenderingOn();

	vtkNew<vtkRenderWindowInteractor> iren;
	//	iren->Initialize();
	iren->SetRenderWindow(renderWindow);
	//iren->Start();
	renderWindow->Render();

	// 输出屏幕截屏
	vtkNew<vtkWindowToImageFilter> window_to_image_filter;
	window_to_image_filter->SetInput(renderWindow);
	window_to_image_filter->SetScale(1); // image quality
	window_to_image_filter->Update();
	m_implant_render_result = window_to_image_filter->GetOutput();

	auto writer = vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetFileName("./implant_screem_shot.png");
	writer->SetInputConnection(window_to_image_filter->GetOutputPort());
	writer->Write();

}

void HipDrrPlanner::drr_bone_implant_render(XRayImageType::Pointer target_data)
{
	XRayImageType::SizeType size = target_data->GetLargestPossibleRegion().GetSize();
	XRayImageType::SpacingType spacing = target_data->GetSpacing();

	vtkSmartPointer<vtkImageData> target_data_vtk = itk_image2_vtk(target_data);
	vtkNew<vtkImageActor> vtk_drr_actor;
	vtk_drr_actor->GetMapper()->SetInputData(target_data_vtk);

	vtkNew<vtkTransform> drr_image_transform;
	drr_image_transform->SetMatrix(m_drr_transform_matrix);
	vtk_drr_actor->SetUserTransform(drr_image_transform);
	
	vtkSmartPointer<vtkRenderer> originalRenderer  = vtkSmartPointer<vtkRenderer>::New();

	double f_stem_color[3];
	double f_head_color[3];
	double hip_cup_color[3];

	if (m_femur_stem_actor != nullptr)
	{
		m_femur_stem_actor->GetProperty()->GetColor(f_stem_color);
		m_femur_stem_actor->GetProperty()->SetColor(255, 0, 0);
	}
	if (m_femur_head_actor != nullptr)
	{
		m_femur_head_actor->GetProperty()->GetColor(f_head_color);
		m_femur_head_actor->GetProperty()->SetColor(255, 0, 0);
	}
	if (m_hip_cup_actor != nullptr)
	{
		m_hip_cup_actor->GetProperty()->GetColor(hip_cup_color);
		m_hip_cup_actor->GetProperty()->SetColor(255, 0, 0);
	}

	originalRenderer->AddActor(m_femur_stem_actor);
	originalRenderer->AddActor(m_femur_head_actor);
	originalRenderer->AddActor(m_hip_cup_actor);
	originalRenderer->AddActor(vtk_drr_actor); 

	//设置相机和window参数
	float my_focal_point[3] = { (float)m_focalpoint[0], (float)m_focalpoint[1], (float)m_focalpoint[2] };
	float focal_point_new[3];
	drr_image_transform->TransformPoint(my_focal_point, focal_point_new);

	//计算相机的focal point
	double drr_image_origin[3];
	double drr_image_origin_new[3];

	vtk_drr_actor->GetMapper()->GetInput()->GetOrigin(drr_image_origin);
	drr_image_transform->TransformPoint(drr_image_origin, drr_image_origin_new);

	drr_image_origin_new[2] = drr_image_origin_new[2] - size[1]*spacing[1] / 2;
	drr_image_origin_new[0] = drr_image_origin_new[0] + size[0]*spacing[0] / 2;

	vtkNew<vtkCamera> my_camera;
	double view_angle;
	my_camera->SetViewAngle(25);
	my_camera->SetPosition(focal_point_new[0], focal_point_new[1], focal_point_new[2]);
	my_camera->SetFocalPoint(drr_image_origin_new[0], drr_image_origin_new[1], drr_image_origin_new[2]);
	my_camera->SetViewUp(0, 0, 1);

	double clip_max_new = abs(drr_image_origin_new[1] - focal_point_new[1]) *1.5;
	my_camera->SetClippingRange(0.01, clip_max_new);
	originalRenderer->SetActiveCamera(my_camera);

	    //vtkNew<vtkRenderWindow> m_renderWindow2;
//        m_renderWindow->OffScreenRenderingOn();
    time_t start_window_init_t = clock();
    m_renderWindow->AddRenderer(originalRenderer);
    m_renderWindow->SetSize(800, 800);
    //m_renderWindow->OffScreenRenderingOff();
    vtkNew<vtkRenderWindowInteractor> iren;
    iren->SetRenderWindow(m_renderWindow);
	m_renderWindow->Render();

	// 输出屏幕截屏
	time_t start_w2i_t = clock();
	vtkNew<vtkWindowToImageFilter> window_to_image_filter;
	window_to_image_filter->SetInput(m_renderWindow);
	//window_to_image_filter->SetInputBufferTypeToRGB();
	window_to_image_filter->SetScale(1); // image quality
	window_to_image_filter->Update();
	m_render_result = window_to_image_filter->GetOutput();
	time_t end_w2i_t = clock();
	cout << "vtkWindowToImageFilter cost " << (double)(end_w2i_t - start_w2i_t) / CLOCKS_PER_SEC << " s" << endl;

	if (m_femur_stem_actor!=nullptr) m_femur_stem_actor->GetProperty()->SetColor(f_stem_color);
	if (m_femur_head_actor != nullptr)	m_femur_head_actor->GetProperty()->SetColor(f_head_color);
	if (m_femur_stem_actor != nullptr) 	m_hip_cup_actor->GetProperty()->SetColor(hip_cup_color);
}

void HipDrrPlanner::itk_rotation_test(CTImageType::Pointer target_data)
{
	vtkSmartPointer<vtkImageData> target_data_vtk = itk_image2_vtk(target_data);

	vtkNew<vtkActor> target_polydata_actor;
	vtkNew<vtkPolyDataMapper> boundary_mapper;
	boundary_mapper->SetInputData(m_femur_stem_polydata);
	target_polydata_actor->SetMapper(boundary_mapper);
	std::vector<vtkActor*> all_actors;
//	all_actors.push_back(target_polydata_actor);
//	show_actors(all_actors);

	vtkSmartPointer<vtkRenderer> render0 = vtkSmartPointer<vtkRenderer>::New();
	render0->SetLayer(1);
//	render0->AddActor(target_polydata_actor);


	vtkImageViewer *viewer = vtkImageViewer::New();
	vtkRenderWindowInteractor *interactor = vtkRenderWindowInteractor::New();
	//viewer->GetRenderer()->AddActor();
	viewer->SetInputData(target_data_vtk);
	viewer->SetupInteractor(interactor);

//	viewer->GetRenderWindow()->SetNumberOfLayers(2);
//	viewer->GetRenderWindow()->AddRenderer(render0);
	viewer->GetRenderWindow()->SetSize(500, 500);//set window size
	viewer->SetColorWindow(255); //set window color
	viewer->SetColorLevel(128);   //set the level of window
	viewer->Render();

	//interactor->Initialize();
	interactor->Start();

}

void HipDrrPlanner::vtk_write_nifti(vtkImageData* target_image_data, string save_path)
{
	vtkNew<vtkNIFTIImageWriter> writer;
	writer->SetFileName(save_path.c_str());
	writer->SetInputData(target_image_data);
	writer->Write();
}

void HipDrrPlanner::run_implant_refresh()
{
	drr_bone_implant_render(m_whole_drr_image);
}

template <typename ImageType>
typename ImageType::Pointer HipDrrPlanner::transform_ct_image(typename ImageType::Pointer target_image, Eigen::Matrix4d transform_matrix, std::string side)
{
///************************************************************************/
///* 函数功能：对图像数据按旋转矩阵在物理空间做变换
///* 函数参数：待变换图像数据，变换矩阵，股骨左右侧
///* 函数返回：变换后的图像矩阵
///* 函数说明：由于itk和vtk原理不同，将齐次转移矩阵拆为旋转和平移，为了提高效率计算roi缩小采样区域
///* 编 写 人：宋云鹏
///* 编写时间：20240510
///************************************************************************/
	typename ImageType::PointType ref_origin;
	double bone_ref_point[3];
	double* target_transformed_f_bbox_itk;
	CTImageType::SizeType output_size;

	if (side == "right")
	{
		target_transformed_f_bbox_itk = transformed_right_f_bbox_itk;

		//取世界坐标系上vtk的bbox中心点计算旋转后输出图像的中心点保证roi不会跑出图像区域
		bone_ref_point[0] = (m_right_femur_bounds[0] + m_right_femur_bounds[1]) / 2;
		bone_ref_point[1] = (m_right_femur_bounds[2] + m_right_femur_bounds[3]) / 2;
		bone_ref_point[2] = (m_right_femur_bounds[4] + m_right_femur_bounds[5]) / 2;
		//使用股骨头参考点技术差值输出的图像数据原点参考点
		ref_origin = cal_ref_point<CTImageType>(m_ori_origin, bone_ref_point, transform_matrix, transformed_right_f_ref_size);
	}
	else
	{
		target_transformed_f_bbox_itk = transformed_left_f_bbox_itk;

		bone_ref_point[0] = (m_left_femur_bounds[0] + m_left_femur_bounds[1]) / 2;
		bone_ref_point[1] = (m_left_femur_bounds[2] + m_left_femur_bounds[3]) / 2;
		bone_ref_point[2] = (m_left_femur_bounds[4] + m_left_femur_bounds[5]) / 2;
	
		ref_origin = cal_ref_point<CTImageType>(m_ori_origin, bone_ref_point, transform_matrix, transformed_left_f_ref_size);
	}
	//直接在vtk上取旋转后的bbox计算对应的itkbbox，基于此计算输出ct尺寸应为最小图像尺寸
	output_size[0] = target_transformed_f_bbox_itk[3] - target_transformed_f_bbox_itk[0];
	output_size[1] = target_transformed_f_bbox_itk[4] - target_transformed_f_bbox_itk[1];
	output_size[2] = target_transformed_f_bbox_itk[5] - target_transformed_f_bbox_itk[2];

	typename ImageType::PointType ori_diff = ref_origin - m_ori_origin;
	using ResampleImageFilterType = itk::ResampleImageFilter<ImageType, ImageType>;
	auto resample = ResampleImageFilterType::New();
	resample->SetInput(target_image);
	//resample->SetReferenceImage(target_image);
	//resample->UseReferenceImageOn();
	
	resample->SetDefaultPixelValue(10);
	resample->SetSize(output_size);
	resample->SetOutputSpacing(m_ori_spacing);
	resample->SetOutputOrigin(ref_origin);
	resample->SetOutputDirection(m_ori_direction);

	using InterpolatorType = itk::NearestNeighborInterpolateImageFunction<ImageType>;
	auto interpolator = InterpolatorType::New();
	resample->SetInterpolator(interpolator);

	using TransformType = itk::AffineTransform<double, 3>;
	auto transform = TransformType::New();

	// get transform parameters from MatrixType
	TransformType::ParametersType params(12);
	for (int i = 0; i < 9; i++) {
		params[i] = transform_matrix(i % 3, i / 3); // R is the rotation matrix of type
	}
	params[9] = 0;
	params[10] = 0;
	params[11] = 0;

	transform->SetParameters(params);
	resample->SetTransform(transform);

	time_t finish_t = clock();
	resample->Update();
	//计算平移参数
	typename ImageType::Pointer out_image = resample->GetOutput();
	using VectorType = itk::Vector<double, 3>;
	VectorType translate;
	translate[0] = -transform_matrix(0, 3) - ori_diff[0];
	translate[1] = -transform_matrix(1, 3) - ori_diff[1];
	translate[2] = -transform_matrix(2, 3) - ori_diff[2];

	auto translate_transform = TransformType::New();
	translate_transform->SetTranslation(translate);

	auto translate_resample = ResampleImageFilterType::New();
	translate_resample->SetInput(out_image);
	translate_resample->SetTransform(translate_transform);
	translate_resample->SetInterpolator(interpolator);

	//translate_resample->SetReferenceImage(out_image);
	//translate_resample->UseReferenceImageOn();
	translate_resample->SetSize(m_ori_size);
	translate_resample->SetOutputSpacing(m_ori_spacing);
	translate_resample->SetOutputOrigin(out_image->GetOrigin());
	translate_resample->SetOutputDirection(m_ori_direction);
	translate_resample->SetDefaultPixelValue(0);
	translate_resample->Update();
	//为保证图像数据拼接手动设定统一的输出原点
	typename ImageType::Pointer back_translated_image = translate_resample->GetOutput();
	back_translated_image->SetOrigin(m_ori_origin);

	return back_translated_image;
}

CTImageType::Pointer HipDrrPlanner::crop_ct_image(CTImageType::Pointer target_image, XRayImageType::Pointer target_label_image, double* bounds)
{
	//bounds [xmin,ymin,zmin,xmax,ymax,zmax]
	//time_t start_t = clock();
	//using LabelStatisticsImageFilterType = itk::LabelStatisticsImageFilter<CTImageType, XRayImageType>;
	//LabelStatisticsImageFilterType::Pointer labelStatisticsFilter = LabelStatisticsImageFilterType::New();
	//labelStatisticsFilter->SetInput(target_image);
	//labelStatisticsFilter->SetLabelInput(target_label_image);
	//labelStatisticsFilter->Update();

	//// Get the region of non-zero pixels
	CTImageType::RegionType region = target_image ->GetLargestPossibleRegion();
	CTImageType::RegionType::IndexType start;
	CTImageType::RegionType::IndexType end;

	//LabelStatisticsImageFilterType::BoundingBoxType bbox = labelStatisticsFilter->GetBoundingBox(255);
	//time_t end_t = clock();
	//cout << "compute bbox costs:" << (double)(end_t - start_t) / CLOCKS_PER_SEC << " s" << endl;

	start[0] = bounds[0];
	start[1] = bounds[1];
	start[2] = bounds[2];

	end[0] = bounds[3];
	end[1] = bounds[4];
	end[2] = bounds[5];

	CTImageType::RegionType desiredRegion;
	desiredRegion.SetIndex(start);
	desiredRegion.SetUpperIndex(end);

	// Crop the image
	using RegionOfInterestImageFilterType = itk::RegionOfInterestImageFilter<CTImageType, CTImageType>;
	RegionOfInterestImageFilterType::Pointer roiFilter = RegionOfInterestImageFilterType::New();
	roiFilter->SetRegionOfInterest(desiredRegion);
	roiFilter->SetInput(target_image);
	roiFilter->Update();

	return roiFilter->GetOutput();
}

template <typename ImageType>
typename ImageType::PointType HipDrrPlanner::cal_ref_point(typename ImageType::PointType origin, double* femur_ref_point,
	Eigen::Matrix4d transform_matrix, int* ref_size)
{
///************************************************************************/
///* 函数功能：计算第一次旋转重采样后输出图像数据的原点
///* 函数参数：原ct的原点，目标参考点作为输出图像中心，摆正变换矩阵，变换后输出图像数据的大小
///* 函数返回：旋转变换后输出图像的原点
///* 函数说明：为保证效率最高resampler的输出要尽可能小，在本函数使用bbox精确计算包含roi的最小的图像尺寸
///* 编 写 人：宋云鹏
///* 编写时间：20240510
///************************************************************************/
	//计算输出图像的参考点，输入是原ct图像数据
	//bbox: [xmin,ymin,zmin,xmax,ymax,zmax]
	typename ImageType::PointType ref_origin;

	Eigen::Vector3d origin_eg = { origin[0], origin[1] , origin[2] };
	Eigen::Vector3d femur_ref_point_eg = { femur_ref_point[0], femur_ref_point[1] , femur_ref_point[2] };

	Eigen::Matrix4d rotation_matrix;
	rotation_matrix = transform_matrix;
	rotation_matrix(0, 3) = 0;
	rotation_matrix(1, 3) = 0;
	rotation_matrix(2, 3) = 0;

	//输出图像为以bbox中心，大小缩小为原理的coef倍
	Eigen::Vector3d ref_translate = { static_cast<double>(ref_size[0] / 2 +1) , static_cast<double>(ref_size[1] / 2 + 1),
		static_cast<double>(ref_size[2] / 2 + 1) };
	Eigen::Vector4d femur_ref_point_homo = femur_ref_point_eg.homogeneous().transpose();
	Eigen::Vector4d transformed_femur_ref_point_homo = rotation_matrix * femur_ref_point_homo;

	Eigen::Vector3d transformed_femur_ref_point(transformed_femur_ref_point_homo(0), transformed_femur_ref_point_homo(1), transformed_femur_ref_point_homo(2));
	Eigen::Vector3d ct_ref_point_tmp = transformed_femur_ref_point - ref_translate;

	ref_origin[0] = ct_ref_point_tmp(0);
	ref_origin[1] = ct_ref_point_tmp(1);
	ref_origin[2] = ct_ref_point_tmp(2);

	return  ref_origin;
}

void HipDrrPlanner::convert_vtk_to_itk_bounds(double* f_bbox_vtk, double* f_bbox_itk)
{
	//output [xmin,ymin,zmin,xmax,ymax,zmax]
	CTImageType::PointType f_bbox_min;
	CTImageType::PointType f_bbox_max;
	CTImageType::IndexType f_bbox_min_idx;
	CTImageType::IndexType f_bbox_max_idx;

	for (int i = 0; i < 3; i++)
	{
		//vtk with sequence: (xmin,xmax, ymin,ymax, zmin,zmax)
		f_bbox_min[i] = f_bbox_vtk[2 * i];
		f_bbox_max[i] = f_bbox_vtk[2 * i + 1];
	}
	m_ct_image->TransformPhysicalPointToIndex(f_bbox_min, f_bbox_min_idx);
	m_ct_image->TransformPhysicalPointToIndex(f_bbox_max, f_bbox_max_idx);

	for (int j = 0; j < 6; j++)
	{
		if (j < 3)
		{
			f_bbox_itk[j] = f_bbox_min_idx[j];
		}
		else
		{
			f_bbox_itk[j] = f_bbox_max_idx[j - 3];
		}
	}
}

CTImageType::Pointer HipDrrPlanner::get_masked_image(XRayImageType::Pointer mask_image, CTImageType::Pointer ct_image)
{
	// 创建一个新的图像实例，复制原始图像的Region（尺寸）和Spacing（间距）等信息  
	CTImageType::Pointer emptyImage = CTImageType::New();
	emptyImage->SetRegions(ct_image->GetLargestPossibleRegion());
	emptyImage->SetSpacing(ct_image->GetSpacing());
	emptyImage->SetOrigin(ct_image->GetOrigin());
	emptyImage->SetDirection(ct_image->GetDirection());
	emptyImage->Allocate();
	emptyImage->FillBuffer(0.0); // 填充为 0.0

//	write_nrrd_test_result(emptyImage, "./emptyImage.nrrd");
	using IteratorType = itk::ImageRegionIterator<XRayImageType>;
	using CTIteratorType = itk::ImageRegionIterator<CTImageType>;

	CTIteratorType empty_image_iterator(emptyImage, emptyImage->GetRequestedRegion());
	CTIteratorType ct_image_iterator(ct_image, ct_image->GetRequestedRegion());
	IteratorType it(mask_image, mask_image->GetRequestedRegion());
	for (it.GoToBegin(); !it.IsAtEnd(); ++it)
	{
		// 访问并处理当前像素  
		XRayImageType::PixelType pixelValue = it.Get();
		if (pixelValue != 0)
		{
			empty_image_iterator.Set(ct_image_iterator.Get());
		}
		++empty_image_iterator;
		++ct_image_iterator;
	
	}
	return emptyImage;
}