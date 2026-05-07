#pragma once
#include <vtkPolydata.h>

void calpolydatadist(vtkSmartPointer<vtkPolyData> data0, vtkSmartPointer<vtkPolyData> data1, double pt1[], double pt2[], double& minDist);
void calpolydatadist_filter(vtkSmartPointer<vtkPolyData> data0, vtkSmartPointer<vtkPolyData> data1);
bool calcMinDistBetweenPloyData(vtkSmartPointer<vtkPolyData> input1, vtkSmartPointer<vtkPolyData> input2, double pt1[], double pt2[], double& minDist);