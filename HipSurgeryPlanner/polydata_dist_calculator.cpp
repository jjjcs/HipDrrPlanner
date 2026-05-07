#include <vtkKdTree.h>
#include <vtkPoints.h>
#include <vtkNamedColors.h>
#include "ImageIO.h"
#include<time.h>
#include <vtkLocator.h>
#include <vtkDistancePolyDataFilter.h>
#include <vtkImplicitPolyDataDistance.h>
#include <vtkCleanPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkDataArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>

using namespace ImgIO;

double cal_dist(double* p0, double* p1)
{
	double res = sqrt(pow(p0[0] - p1[0], 2) + pow(p0[1] - p1[1], 2) + pow(p0[2] - p1[2], 2));
	return res;
}

void calpolydatadist(vtkSmartPointer<vtkPolyData> data0, vtkSmartPointer<vtkPolyData> data1, double pt1[], double pt2[], double& minDist)
{

	std::cout << "input1 point number" << data0->GetNumberOfPoints() << std::endl;
	std::cout << "input2 point number" << data1->GetNumberOfPoints() << std::endl;
	double* point_tmp;
	double* min_point_target;
	double* min_point_source;
	double min_dist = 10000;
	double dist_tmp;
	int source_point_id;
	int min_source_point_id;
	int min_target_point_id;

	clock_t start, finish;  // clock_t为时钟计时单元数
	start = clock();

	vtkSmartPointer<vtkPoints> source_points;
	source_points = data0->GetPoints();

	vtkSmartPointer<vtkKdTree> kdTree =
		vtkSmartPointer<vtkKdTree>::New();
	kdTree->BuildLocatorFromPoints(source_points);


	int num_points = data1->GetNumberOfPoints();
	for (int i = 0; i < num_points; i++)
	{
		point_tmp = data1->GetPoint(i);

		source_point_id = kdTree->FindClosestPoint(point_tmp[0], point_tmp[1], point_tmp[2], dist_tmp);

		if (dist_tmp < min_dist)
		{
			min_dist = dist_tmp;
			min_target_point_id = i;
			min_source_point_id = source_point_id;
		}
	}

	min_point_source = data0->GetPoint(min_source_point_id);
	min_point_target = data1->GetPoint(min_target_point_id);
	minDist = min_dist;
	memcpy(pt1, min_point_source, 3 * sizeof(double));
	memcpy(pt2, min_point_target, 3 * sizeof(double));

	finish = clock();
	cout << "代码运行花费时间为：" << double(finish - start) / CLOCKS_PER_SEC << "s" << endl;  //时间计算过程
	std::cout << "最小距离为：" << min_dist << std::endl;

	std::cout << "最小距source id：" << min_source_point_id << std::endl;
	std::cout << "最小距source点：" << min_point_source[0] << " " << min_point_source[1] << " " << min_point_source[2] << " " << std::endl;
	std::cout << "最小距target id：" << min_target_point_id << std::endl;
	std::cout << "最小距target点：" << min_point_target[0] << " " << min_point_target[1] << " " << min_point_target[2] << " " << std::endl;
}

void calpolydatadist_filter(vtkSmartPointer<vtkPolyData> data0, vtkSmartPointer<vtkPolyData> data1)
{
	clock_t start, finish;  // clock_t为时钟计时单元数
	start = clock();

	vtkNew<vtkDistancePolyDataFilter> distanceFilter;

	vtkNew<vtkCleanPolyData> clean1;
	clean1->SetInputData(data0);

	vtkNew<vtkCleanPolyData> clean2;
	clean2->SetInputData(data1);

	distanceFilter->SetInputConnection(0, clean1->GetOutputPort());
	distanceFilter->SetInputConnection(1, clean2->GetOutputPort());
	distanceFilter->Update();

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputConnection(distanceFilter->GetOutputPort());

	finish = clock();
	cout << "代码运行花费时间为：" << double(finish - start) / CLOCKS_PER_SEC << "ms" << endl;  //时间计算过程

	vtkDataArray* x = distanceFilter->GetOutput()->GetPointData()->GetScalars();
	std::cout << *x << std::endl;
	mapper->SetScalarRange(
		distanceFilter->GetOutput()->GetPointData()->GetScalars()->GetRange()[0],
		distanceFilter->GetOutput()->GetPointData()->GetScalars()->GetRange()[1]);
}

bool calcMinDistBetweenPloyData(vtkSmartPointer<vtkPolyData> input1, vtkSmartPointer<vtkPolyData> input2, double pt1[], double pt2[], double& minDist)
{
	std::cout << "input1 point number" << input1->GetNumberOfPoints() << std::endl;
	std::cout << "input2 point number" << input2->GetNumberOfPoints() << std::endl;

	vtkNew<vtkCleanPolyData> clean1;
	clean1->SetInputData(input1);

	vtkNew<vtkCleanPolyData> clean2;
	clean2->SetInputData(input2);
	clock_t start, finish;  // clock_t为时钟计时单元数
	start = clock();
	vtkNew<vtkDistancePolyDataFilter> distanceFilter;

	distanceFilter->SetInputConnection(0, clean1->GetOutputPort());
	distanceFilter->SetInputConnection(1, clean2->GetOutputPort());

	//ULONGLONG start = GetTickCount64();
	distanceFilter->Update();
	//ULONGLONG end = GetTickCount64();
	//qDebug() << "vtkDistancePolyDataFilter used time (ms)" << end - start;

	vtkPolyData *distanOut = distanceFilter->GetOutput();
	//	vtkDataArray* distArray = distanOut->GetPointData()->GetScalars();
	std::cout << distanOut->GetNumberOfPoints() << std::endl;


	vtkPolyData * inPts = distanOut;
	vtkPoints* Pts = distanOut->GetPoints();
	vtkPointData* pd = distanOut->GetPointData();
	if (!pd->GetScalars() || !distanOut)
	{
		std::cerr << "No input data" << std::endl;
		return false;
	}

	vtkDataArray*  inVectors = pd->GetScalars();
	vtkIdType  numPts = inPts->GetNumberOfPoints();
	vtkPoints  *newPts = vtkPoints::New();
	newPts->SetNumberOfPoints(numPts);

	// Loop over all points, adjusting locations
	vtkIdType indexMin = -1;
	double valueMin = 10000;
	double point[3];

	for (vtkIdType ptId = 0; ptId < numPts; ptId++)
	{
		double* x = inPts->GetPoint(ptId);
		double* v = inVectors->GetTuple(ptId);

		if (*v < valueMin) {
			indexMin = ptId;
			valueMin = *v;
			for (int i = 0; i < 3; i++)
			{
				point[i] = x[i];
			}
		}
	}

	std::cout << "min val id= " << indexMin << " min dist val= " << valueMin << std::endl;
	std::cout << "min dist point object 1 postion= " << point[0] << "  " << point[1] << "  " << point[2] << std::endl;

	memcpy(pt1, point, 3 * sizeof(double));

	vtkSmartPointer<vtkImplicitPolyDataDistance> implicitPolyDataDistance =
		vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	implicitPolyDataDistance->SetInput(clean2->GetOutput());

	double closetPoint[3];
	double* pt0 = point;
	double signedDistance = implicitPolyDataDistance->EvaluateFunctionAndGetClosestPoint(pt0, closetPoint);

	std::cout << "min dist point on object 2 closetPoint= " << closetPoint[0] << "  " << closetPoint[1] << "  " << closetPoint[2] << std::endl;
	std::cout << "min dist point on object 2 signedDistance= " << signedDistance << std::endl;

	memcpy(pt2, closetPoint, 3 * sizeof(double));
	minDist = signedDistance;

	finish = clock();
	cout << "代码运行花费时间为：" << double(finish - start) / CLOCKS_PER_SEC << "ms" << endl;  //时间计算过程

	return true;
}