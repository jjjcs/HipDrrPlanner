#pragma once
//
#include <itkGDCMImageIO.h>
#include <itkGDCMSeriesFileNames.h>
#include <itkImageSeriesReader.h>
#include "itkNrrdImageIOFactory.h"
#include <itkNiftiImageIO.h>
#include "itkImage.h"
#include "itkCastImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include <vtkSTLReader.h>

namespace ImgIO 
{
	using CTPixelType = signed short;
	using CTImageType = itk::Image<CTPixelType, 3>;
	using XRayImageType = itk::Image<unsigned char, 3>;
	using ReaderType = itk::ImageSeriesReader<CTImageType>;
	using ImageIOType = itk::GDCMImageIO;
	using NamesGeneratorType = itk::GDCMSeriesFileNames;

	using PointType = itk::Point<float, 3>;


	class MyImageIO
	{

	public:
		CTImageType::Pointer readNrrdFile(std::string NrrdPath)
		{
			using NRRD_ReaderFilter = itk::ImageFileReader<CTImageType>;
			NRRD_ReaderFilter::Pointer reader = NRRD_ReaderFilter::New();

			itk::NrrdImageIOFactory::RegisterOneFactory();
			reader->SetFileName(NrrdPath);
			reader->Update();

			CTImageType::Pointer inputImage = reader->GetOutput();
			return inputImage;
		}

		XRayImageType::Pointer readNrrdMaskFile(std::string NrrdPath)
		{
			using NRRD_ReaderFilter = itk::ImageFileReader<XRayImageType>;
			NRRD_ReaderFilter::Pointer reader = NRRD_ReaderFilter::New();

			itk::NrrdImageIOFactory::RegisterOneFactory();
			reader->SetFileName(NrrdPath);
			reader->Update();

			XRayImageType::Pointer inputImage = reader->GetOutput();
			return inputImage;
		}

		CTImageType::Pointer readDicomFiles(std::string dicomDir)
		{
			ImageIOType::Pointer gdcmIO = ImageIOType::New();
			NamesGeneratorType::Pointer namesGenerator = NamesGeneratorType::New();
			namesGenerator->SetUseSeriesDetails(true);
			namesGenerator->AddSeriesRestriction("0008|0021");
			namesGenerator->SetInputDirectory(dicomDir);
			const itk::SerieUIDContainer _existSeries = namesGenerator->GetSeriesUIDs();
			std::cout << "dicom series count: " << _existSeries.size() << std::endl;
			int realIdx = 0;
			size_t maxLength = 0;
			for (int i = 0; i < _existSeries.size(); ++i)
			{
				const ReaderType::FileNamesContainer& _filenames = namesGenerator->GetFileNames(_existSeries[i]);
				std::cout << "series " << i << " files count: " << _filenames.size() << std::endl;
				if (_filenames.size() > maxLength)
				{
					maxLength = _filenames.size();
					realIdx = i;
				}
				//std::cout << "best idx: " << realIdx << ",  length: " << maxLength << std::endl;
			}
			//const ReaderType::FileNamesContainer& filenames = namesGenerator->GetInputFileNames();
			const ReaderType::FileNamesContainer& filenames = namesGenerator->GetFileNames(_existSeries[realIdx]);
			std::size_t numberofFileNames = filenames.size();
			std::cout << "dicom files count: " << numberofFileNames << std::endl;

			ReaderType::Pointer reader = ReaderType::New();
			reader->SetImageIO(gdcmIO);
			reader->SetFileNames(filenames);
			try
			{
				reader->Update();
			}
			catch (const itk::ExceptionObject& e)
			{
				std::cerr << "exception in file reader" << std::endl;
				//std::cerr << e << std::endl;
				return CTImageType::Pointer();
			}
			CTImageType::Pointer inputImage = reader->GetOutput();
			CTImageType::RegionType region = inputImage->GetLargestPossibleRegion();
			CTImageType::IndexType start = region.GetIndex();
			CTImageType::SizeType size = region.GetSize();
			return inputImage;
		}

		vtkSmartPointer<vtkPolyData> load_STL(std::string STL_FILE_PATH)
		{
			vtkSmartPointer<vtkPolyData> polyData;

			vtkNew<vtkSTLReader> reader;
			reader->SetFileName(STL_FILE_PATH.c_str());
			reader->Update();
			polyData = reader->GetOutput();

			return polyData;
		}

	};
}