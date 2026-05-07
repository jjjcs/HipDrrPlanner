#pragma once
#include "vtkAutoInit.h" 
VTK_MODULE_INIT(vtkRenderingOpenGL2); // VTK was built with vtkRenderingOpenGL2
VTK_MODULE_INIT(vtkInteractionStyle);
#include <vtkSTLReader.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkStripper.h>
#include <itkeigen/Eigen/Core>
#include <fstream>
#include <string>

#include <sstream>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>
#include <vtkNamedColors.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkMatrix3x3.h>
#include <vtkMath.h>
#include <vtkVector.h>
#include <vtkPolyLine.h>
#include <vtkDataArray.h>
#include <vtkPlane.h>
#include "shlwapi.h"
//#include"vtk_tools.h"
using namespace std;
using namespace Eigen;

string color = "Cornsilk";



bool isFileExists_ifstream(string& name) {
	ifstream f(name.c_str());
	return f.good();
}


vtkVector3d dot(vtkNew<vtkMatrix3x3>& matrix_in, vtkVector3d vector_in)
{
	double x = matrix_in->GetElement(0, 0)*vector_in(0) + matrix_in->GetElement(0, 1)*vector_in(1) + matrix_in->GetElement(0, 2)*vector_in(2);
	double y = matrix_in->GetElement(1, 0)*vector_in(0) + matrix_in->GetElement(1, 1)*vector_in(1) + matrix_in->GetElement(1, 2)*vector_in(2);
	double z = matrix_in->GetElement(2, 0)*vector_in(0) + matrix_in->GetElement(2, 1)*vector_in(1) + matrix_in->GetElement(2, 2)*vector_in(2);
	vtkVector3d vector_out(-x, -y, -z);

	return vector_out;
}


// for visualization of the result
void createLineActor(Eigen::Matrix3d points_array, float opacity, string color, float line_width, vtkActor* actor)
{
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkPoints> points_tmp;
	vtkNew<vtkPolyLine> poly_line;

	points_tmp->InsertNextPoint(points_array.row(0)[0], points_array.row(0)[0], points_array.row(0)[0]);
	points_tmp->InsertNextPoint(points_array.row(1)[0], points_array.row(1)[0], points_array.row(1)[0]);
	poly_line->GetPointIds()->SetNumberOfIds(2);
	poly_line->GetPointIds()->SetId(0, 0);
	poly_line->GetPointIds()->SetId(1, 1);

	vtkNew<vtkCellArray> cells;
	cells->InsertNextCell(poly_line);
	vtkNew<vtkPolyData> poly_data;
	poly_data->SetPoints(points_tmp);
	poly_data->SetLines(cells);

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(poly_data);

	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());
	actor->GetProperty()->SetOpacity(opacity);
	actor->GetProperty()->SetLineWidth(line_width);
}

void createLineActor(std::vector<Eigen::Vector3d> points, vtkActor* actor, float opacity, string color, float line_width)
{
	int num_points = points.size();
	Eigen::Vector3d p0 = points.at(num_points-2);
	Eigen::Vector3d p1 = points.at(num_points-1);
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkPoints> points_tmp;
	vtkNew<vtkPolyLine> poly_line;

	points_tmp->InsertNextPoint(p0[0], p0[1], p0[2]);
	points_tmp->InsertNextPoint(p1[0], p1[1], p1[2]);
	poly_line->GetPointIds()->SetNumberOfIds(2);
	poly_line->GetPointIds()->SetId(0, 0);
	poly_line->GetPointIds()->SetId(1, 1);

	vtkNew<vtkCellArray> cells;
	cells->InsertNextCell(poly_line);
	vtkNew<vtkPolyData> poly_data;
	poly_data->SetPoints(points_tmp);
	poly_data->SetLines(cells);

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(poly_data);

	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());
	actor->GetProperty()->SetOpacity(opacity);
	actor->GetProperty()->SetLineWidth(line_width);
}

void createLineActor(Eigen::Vector3d normal, Eigen::Vector3d center, vtkActor* actor, float opacity = 1, string color = "Red", float line_width = 5)
{
	Eigen::Matrix3d points_array;
	Eigen::Vector3d p0;
	Eigen::Vector3d p1;
	p0 << center[0], center[1], center[2];
	p1 << center[0] + 40 * normal[0], center[1] + 40 * normal[1], center[2] + 40 * normal[2];
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkPoints> points_tmp;
	vtkNew<vtkPolyLine> poly_line;

	points_tmp->InsertNextPoint(p0[0], p0[1], p0[2]);
	points_tmp->InsertNextPoint(p1[0], p1[1], p1[2]);
	poly_line->GetPointIds()->SetNumberOfIds(2);
	poly_line->GetPointIds()->SetId(0, 0);
	poly_line->GetPointIds()->SetId(1, 1);

	vtkNew<vtkCellArray> cells;
	cells->InsertNextCell(poly_line);
	vtkNew<vtkPolyData> poly_data;
	poly_data->SetPoints(points_tmp);
	poly_data->SetLines(cells);

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(poly_data);

	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());
	actor->GetProperty()->SetOpacity(opacity);
	actor->GetProperty()->SetLineWidth(line_width);
}


void createSphereActor(Eigen::Vector3d center, vtkActor* actor, float radius = 3, float opacity = 1, string color = "Red")
{
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkSphereSource> sphere;

	sphere->SetCenter(center[0], center[1], center[2]);
	sphere->SetRadius(radius);
	sphere->SetPhiResolution(100);
	sphere->SetThetaResolution(100);

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputConnection(sphere->GetOutputPort());

	actor->SetMapper(mapper);
	//actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());
	actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());
	actor->GetProperty()->SetOpacity(opacity);
};

void createSphereActor(float center[3], float radius, float opacity, string color, vtkActor* actor)
{
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkSphereSource> sphere;

	sphere->SetCenter(center[0], center[1], center[2]);
	sphere->SetRadius(radius);
	sphere->SetPhiResolution(100);
	sphere->SetThetaResolution(100);
	sphere->Update();

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(sphere->GetOutput());
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());
	actor->GetProperty()->SetOpacity(opacity);
};


void createSphereActor(double center[3], float radius, float opacity, string color, vtkActor* actor)
{
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkSphereSource> sphere;

	sphere->SetCenter(center[0], center[1], center[2]);
	sphere->SetRadius(radius);
	sphere->SetPhiResolution(100);
	sphere->SetThetaResolution(100);
	sphere->Update();

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(sphere->GetOutput());
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());
	actor->GetProperty()->SetOpacity(opacity);
};

void show_actors(vector<vtkActor*> actors)
{
	vtkNew<vtkRenderer> ren;
	for (int i = 0; i < actors.size(); i++)
	{
		auto x = actors.at(i);
		ren->AddActor(actors.at(i));
	};

	vtkNew<vtkRenderWindow> win;
	win->AddRenderer(ren);
	win->SetWindowName("show spine ");

	vtkNew<vtkRenderWindowInteractor> iren;
	iren->SetRenderWindow(win);
	vtkNew<vtkInteractorStyleTrackballCamera> style;
	iren->SetInteractorStyle(style);

	ren->ResetCamera();
	win->Render();
	iren->Initialize();
	iren->Start();
}


void createAxisActor(Eigen::MatrixXd points_array, string color, vtkActor* actor)
{
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkPoints> points;
	points->InsertNextPoint(points_array(0, 0), points_array(0, 1), points_array(0, 2));
	points->InsertNextPoint(points_array(1, 0), points_array(1, 1), points_array(1, 2));
	vtkNew<vtkPolyLine> poly_line;
	poly_line->GetPointIds()->SetNumberOfIds(2);
	for (unsigned int i = 0; i < 2; i++)
	{
		poly_line->GetPointIds()->SetId(i, i);
	}

	vtkNew<vtkCellArray> cells;
	cells->InsertNextCell(poly_line);

	vtkNew<vtkPolyData> poly_data;
	poly_data->SetPoints(points);
	poly_data->SetLines(cells);

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(poly_data);

	actor->SetMapper(mapper);
	actor->GetProperty()->SetLineWidth(10);
	actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());
}


void createCoordActor(Eigen::Matrix3d coordinate, Eigen::Vector3d origin, vector<vtkActor*>& actors, float opacity, int length)
{
	string x_color = "DarkRed";
	string y_color = "DarkGreen";
	string z_color = "DarkBlue";
	// create axis actor using two points on a line
	Eigen::MatrixXd x_points(2, 3);
	Eigen::MatrixXd y_points(2, 3);
	Eigen::MatrixXd z_points(2, 3);
	std::cout << origin << std::endl;

	x_points.row(0) = origin;
	x_points.row(1) = length * coordinate.row(0) + x_points.row(0);
	//std::cout << x_points << std::endl;

	static vtkNew<vtkActor> x_actor;
	createAxisActor(x_points, x_color, x_actor);
	x_actor->GetProperty()->SetOpacity(opacity);
	actors.push_back(x_actor);

	y_points.row(0) = origin;
	y_points.row(1) = length * coordinate.row(1) + y_points.row(0);
	//std::cout << y_points << std::endl;

	static  vtkNew<vtkActor> y_actor;
	createAxisActor(y_points, y_color, y_actor);
	y_actor->GetProperty()->SetOpacity(opacity);
	actors.push_back(y_actor);

	z_points.row(0) = origin;
	z_points.row(1) = length * coordinate.row(2) + z_points.row(0);
	//std::cout << z_points << std::endl;

	static  vtkNew<vtkActor> z_actor;
	createAxisActor(z_points, z_color, z_actor);
	z_actor->GetProperty()->SetOpacity(opacity);
	actors.push_back(z_actor);
	//show_actors(actors);
}

void createLineActor(std::vector<Eigen::Vector3d> points, vtkActor* actor, float opacity = 1, string color = "Red", float line_width = 5);
void createBBoxactors(double bounds[6], vector<vtkActor*>& all_actors)
{
	//bounds: xmin,xmax, ymin,ymax, zmin,zmax
	vtkNew<vtkNamedColors> colors;
	Eigen::Vector3d cur_point = Eigen::Vector3d::Zero();
	vector<Eigen::Vector3d> points;

	for (int i = 0; i < 2; i++)
	{
		for (int j = 2; j < 4; j++)
		{
			for (int k = 4; k < 6; k++)
			{
				cur_point << bounds[i], bounds[j], bounds[k];
				auto p = vtkActor::New();
				createSphereActor(cur_point, p, 2, 1, "Red");
				all_actors.push_back(p);
			}
		}
	}

	vector<Eigen::Vector3d> points_lower;
	for (int i = 0; i < 2; i++)
	{
		for (int j = 2; j < 4; j++)
		{
			cur_point << bounds[i], bounds[j], bounds[4];
			points_lower.push_back(cur_point);
			cur_point << bounds[i], bounds[j], bounds[5];
			points_lower.push_back(cur_point);
			auto line_actor = vtkActor::New();
			createLineActor(points_lower, line_actor);
			all_actors.push_back(line_actor);
		}
	}

	for (int i = 0; i < 2; i++)
	{
		for (int k = 4; k < 6; k++)
		{
			cur_point << bounds[i], bounds[2], bounds[k];
			points_lower.push_back(cur_point);
			cur_point << bounds[i], bounds[3], bounds[k];
			points_lower.push_back(cur_point);
			auto line_actor = vtkActor::New();
			createLineActor(points_lower, line_actor);
			all_actors.push_back(line_actor);
		}
	}

	for (int j = 2; j < 4; j++)
	{
		for (int k = 4; k < 6; k++)
		{
			cur_point << bounds[0], bounds[j], bounds[k];
			points_lower.push_back(cur_point);
			cur_point << bounds[1], bounds[j], bounds[k];
			points_lower.push_back(cur_point);
			auto line_actor = vtkActor::New();
			createLineActor(points_lower, line_actor);
			all_actors.push_back(line_actor);
		}
	}
}