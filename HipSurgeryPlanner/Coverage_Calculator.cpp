#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkNamedColors.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkClipClosedSurface.h>
#include <vtkClipPolyData.h>
#include <vtkImplicitFunction.h>
#include <vtkSphere.h>
#include <vtkSphereSource.h>
#include <vtkCutter.h>
#include <vtkMassProperties.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataNormals.h>
#include <vtkPlane.h>
#include <vtkPlaneCollection.h>
#include <vtkCamera.h>
#include <vtkDiskSource.h>
#include <vtkArcSource.h>
#include <vtkStripper.h>
#include <vtkKochanekSpline.h>
#include <vtkSplineFilter.h>
#include <vtkTubeFilter.h>
#include <vtkPlaneSource.h>
#include <vtkLightCollection.h>
#include <vtkLight.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#include "Coverage_Calculator.h"
#include "ImageIO.h"
using namespace ImgIO;
# define M_PI 3.14159265358979323846 

double Coverage_Calculator::cal_coverage()
{
	//render_projection();
	double lower_surface_area = get_lower_surface_area();
	double intersection_surface_area = get_intersection_surface_area();

    //计算半球表面积
	std::cout;
	double cup_sphere_area = 4 * M_PI * m_cup_sphere_radius * m_cup_sphere_radius / 2;
	double coverage_rate = (intersection_surface_area - lower_surface_area) / cup_sphere_area;

	return coverage_rate;
};

double Coverage_Calculator::get_lower_surface_area()
{
	vtkNew<vtkSphereSource> mySphere;
	mySphere->SetPhiResolution(50);
	mySphere->SetThetaResolution(50);
	mySphere->SetRadius(m_cup_sphere_radius);
	mySphere->SetCenter(m_cup_sphere_center[0], m_cup_sphere_center[1], m_cup_sphere_center[2]);
	mySphere->Update();
	
	vtkNew<vtkSphere> sphere2;
	sphere2->SetRadius(m_cup_sphere_radius);
	sphere2->SetCenter(m_cup_sphere_center[0], m_cup_sphere_center[1], m_cup_sphere_center[2]);

	vtkNew<vtkClipPolyData> clipper;
	clipper->SetClipFunction(sphere2);
	clipper->SetInputData(m_target_hip_polydata);
	clipper->InsideOutOn();
	clipper->Update();

	//使用髋臼杯半圆平面为截面再清理一下结果
	vtkNew<vtkPlane> plane;
	plane->SetOrigin(m_cup_sphere_center);
	plane->SetNormal(m_cup_z_axis);

	vtkNew<vtkClipPolyData> clipper2;
	clipper2->SetInputData(clipper->GetOutput());
	clipper2->SetClipFunction(plane);
	clipper2->SetValue(0);
	clipper2->Update();

	vtkNew<vtkActor> hip_actor;
	vtkNew<vtkPolyDataMapper>  hip_mapper;
	hip_mapper->SetInputData(mySphere->GetOutput());
	hip_actor->SetMapper(hip_mapper);
	hip_actor->GetProperty()->SetOpacity(0.5);

	vtkNew<vtkActor> sphere_actor;
	vtkNew<vtkPolyDataMapper>  sphere_mapper;
	sphere_mapper->SetInputData(clipper2->GetOutput());
	sphere_actor->SetMapper(sphere_mapper);
	//intersection_actor->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());
	//intersection_actor->GetProperty()->SetOpacity(0.5);

	vtkSmartPointer< vtkTriangleFilter > triangleFilter = vtkSmartPointer< vtkTriangleFilter >::New();
	triangleFilter->SetInputData(clipper2->GetOutput());
	triangleFilter->Update();

	vtkSmartPointer< vtkMassProperties > polygonProperties = vtkSmartPointer< vtkMassProperties >::New();
	polygonProperties->SetInputData(triangleFilter->GetOutput());
	polygonProperties->Update();
	double area = polygonProperties->GetSurfaceArea();
	std::cout << "lower surface area is :" << area << std::endl;

	//vtkNew<vtkRenderWindow> renderWindow;
	//vtkSmartPointer<vtkRenderer> render = vtkSmartPointer<vtkRenderer>::New();
	//render->AddActor(hip_actor);
	//render->AddActor(sphere_actor);

	//vtkNew<vtkRenderWindowInteractor> iren;

	//iren->SetRenderWindow(renderWindow);

	//renderWindow->AddRenderer(render);
	//renderWindow->Render();
	//iren->Initialize();
	//iren->Start();

	return area;
}

double Coverage_Calculator::get_intersection_surface_area()
{
	//使用髋臼杯半圆平面为截面再清理一下结果
	//vtkNew<vtkPlane> plane;
	//plane->SetOrigin(m_cup_sphere_center);
	//plane->SetNormal(m_cup_z_axis);

	//vtkNew<vtkPlaneCollection> planes;
	//planes->AddItem(plane);

	//vtkNew<vtkClipClosedSurface> clipper;
	//clipper->SetInputData(m_intersection_polydata);
	//clipper->SetClippingPlanes(planes);
	//clipper->Update();

	vtkNew<vtkPlane> plane;
	plane->SetOrigin(m_cup_sphere_center);
	plane->SetNormal(-m_cup_z_axis[0], -m_cup_z_axis[1], -m_cup_z_axis[2]);

	vtkNew<vtkClipPolyData> clipper2;
	clipper2->SetInputData(m_intersection_polydata);
	clipper2->SetClipFunction(plane);
	clipper2->SetValue(0);
	clipper2->Update();

	vtkSmartPointer< vtkTriangleFilter > triangleFilter = vtkSmartPointer< vtkTriangleFilter >::New();
	triangleFilter->SetInputData(clipper2->GetOutput());
	triangleFilter->Update();

	//vtkNew<vtkPolyDataNormals> normals;
	////normals->SetInputConnection(triF->GetOutputPort());
	//normals->SetInputData(triangleFilter->GetOutput());
	//normals->ConsistencyOn();
	//normals->SplittingOff();

	vtkSmartPointer< vtkMassProperties > polygonProperties = vtkSmartPointer< vtkMassProperties >::New();
	polygonProperties->SetInputData(triangleFilter->GetOutput());
	polygonProperties->Update();
	double area = polygonProperties->GetSurfaceArea();
	std::cout << "intersection surface area is :" << area << std::endl;

	vtkNew<vtkActor> hip_actor;
	vtkNew<vtkPolyDataMapper>  hip_mapper;
	hip_mapper->SetInputData(clipper2->GetOutput());
	hip_actor->SetMapper(hip_mapper);
	hip_actor->GetProperty()->SetOpacity(0.5);
	
	vtkNew<vtkRenderWindow> renderWindow;
	vtkSmartPointer<vtkRenderer> render = vtkSmartPointer<vtkRenderer>::New();
	render->AddActor(hip_actor);

	vtkNew<vtkRenderWindowInteractor> iren;

	iren->SetRenderWindow(renderWindow);

	renderWindow->AddRenderer(render);
	renderWindow->Render();
	iren->Initialize();
	iren->Start();

	return area;
}

void Coverage_Calculator::render_projection()
{
	vtkNew<vtkNamedColors> colors;
	vtkSmartPointer<vtkActor> circle_actor = draw_cirlce(m_cup_sphere_center, m_cup_z_axis, m_cup_sphere_radius);

	vtkNew<vtkSphereSource> mySphere;
	mySphere->SetPhiResolution(500);
	mySphere->SetThetaResolution(500);
	mySphere->SetRadius(m_cup_sphere_radius);
	mySphere->SetCenter(m_cup_sphere_center[0], m_cup_sphere_center[1], m_cup_sphere_center[2]);
	mySphere->Update();

	vtkNew<vtkPlane> plane1;
	plane1->SetOrigin(m_cup_sphere_center);
	plane1->SetNormal(m_cup_z_axis);

	vtkNew<vtkPlaneCollection> planes;
	planes->AddItem(plane1);

	vtkNew<vtkClipClosedSurface> sclipper;
	sclipper->SetInputData(mySphere->GetOutput());
	sclipper->SetClippingPlanes(planes);
	sclipper->SetActivePlaneId(0);
	sclipper->Update();

	vtkNew<vtkActor> mySphere_actor;
	vtkNew<vtkPolyDataMapper>  mySphere_mapper;
	mySphere_mapper->SetInputData(sclipper->GetOutput());
	mySphere_actor->SetMapper(mySphere_mapper);
	mySphere_actor->GetProperty()->SetOpacity(0.4);
	mySphere_actor->GetProperty()->SetColor(colors->GetColor3d("Yellow").GetData());

	MyImageIO MyIO;
	vtkSmartPointer<vtkPolyData> hip_bone_polydata = MyIO.load_STL("D:\\data\\hip\\hospital\\doctor\\473010_R\\projectfiles\\bone2.stl");
	vtkNew<vtkActor> hip_bone_actor;
	vtkNew<vtkPolyDataMapper> hip_bone_mapper;
	hip_bone_mapper->SetInputData(hip_bone_polydata);
	hip_bone_actor->SetMapper(hip_bone_mapper);


	vtkNew<vtkSphere> sphere2;
	sphere2->SetRadius(m_cup_sphere_radius);
	sphere2->SetCenter(m_cup_sphere_center[0], m_cup_sphere_center[1], m_cup_sphere_center[2]);

	vtkNew<vtkClipPolyData> clipper;
	clipper->SetClipFunction(sphere2);
	clipper->SetInputData(m_target_hip_polydata);
	clipper->InsideOutOn();
	clipper->Update();

	vtkNew<vtkPlane> plane;
	plane->SetOrigin(m_cup_sphere_center);
	plane->SetNormal(m_cup_z_axis);

	vtkNew<vtkClipPolyData> clipper2;
	clipper2->SetInputData(m_intersection_polydata);
	clipper2->SetClipFunction(plane);
	clipper2->SetValue(0);
	clipper2->Update();

	vtkNew<vtkActor> hip_actor;
	vtkNew<vtkPolyDataMapper> hip_mapper;
	hip_mapper->SetInputData(clipper2->GetOutput());
	hip_actor->SetMapper(hip_mapper);
//	hip_actor->GetProperty()->SetOpacity(0.9);

	vtkNew<vtkActor> clipped_lower_surface_actor;
	vtkNew<vtkPolyDataMapper>  clipped_lower_surface_mapper;
	clipped_lower_surface_mapper->SetInputData(clipper2->GetOutput());
	clipped_lower_surface_actor->SetMapper(clipped_lower_surface_mapper);
	clipped_lower_surface_actor->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());

	vtkNew<vtkCamera> my_camera;
	my_camera->SetPosition(m_cup_sphere_center[0] - 500*m_cup_z_axis[0], m_cup_sphere_center[1] - 500*m_cup_z_axis[1], m_cup_sphere_center[2] - 500*m_cup_z_axis[2]);
	my_camera->SetFocalPoint(m_cup_sphere_center[0], m_cup_sphere_center[1], m_cup_sphere_center[2]);
	my_camera->ParallelProjectionOn();
	my_camera->SetParallelScale(30.);
	vtkSmartPointer<vtkRenderer> render = vtkSmartPointer<vtkRenderer>::New();
	vtkNew<vtkRenderWindow> renderWindow;
	renderWindow->SetSize(256, 256);

	vtkSmartPointer<vtkLight> greenLight = vtkSmartPointer<vtkLight>::New();
	//greenLight->SetColor(0, 1, 0);
	greenLight->SetPosition(m_cup_sphere_center);
	greenLight->SetFocalPoint(render->GetActiveCamera()->GetFocalPoint());
	//render->AddLight(greenLight);

	//vtkLightCollection* originalLights = render->GetLights();
	//std::cout << "Originally there are " << originalLights->GetNumberOfItems()
	//	<< " lights." << std::endl;

	render->SetActiveCamera(my_camera);
//	render->AddActor(clipped_lower_surface_actor);
//	render->AddActor(mySphere_actor);
	render->AddActor(circle_actor);
	//render->AddActor(hip_bone_actor);
	vtkNew<vtkRenderWindowInteractor> iren;
	iren->SetRenderWindow(renderWindow);

	renderWindow->AddRenderer(render);
	renderWindow->OffScreenRenderingOn();
	renderWindow->Render();
	//std::cout << "Originally there are " << originalLights->GetNumberOfItems()
	//	<< " lights." << std::endl;
	//iren->Initialize();
	//iren->Start();

	vtkNew<vtkWindowToImageFilter> window_to_image_filter;
	window_to_image_filter->SetInput(renderWindow);
	window_to_image_filter->SetScale(1); // image quality
	window_to_image_filter->Update();

	auto writer = vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetFileName("./femur_cup_screem_shot.png");
	writer->SetInputConnection(window_to_image_filter->GetOutputPort());
	writer->Write();
};

vtkSmartPointer<vtkActor> Coverage_Calculator::draw_cirlce(double* center, double* normal, double radius)
{
	vtkNew<vtkSphereSource> mySphere;
	mySphere->SetPhiResolution(500);
	mySphere->SetThetaResolution(500);
	mySphere->SetRadius(radius);
	mySphere->SetCenter(center);
	mySphere->Update();	

	vtkNew<vtkPlane> plane;
	plane->SetNormal(normal);
	plane->SetOrigin(center);

	vtkNew<vtkCutter> cutter;
	cutter->SetInputData(mySphere->GetOutput());
	cutter->SetCutFunction(plane);
//	cutter->GenerateValues(1, 0.0, 0.0);

	vtkNew<vtkStripper> stripper;
	stripper->SetInputConnection(cutter->GetOutputPort());

	vtkNew<vtkKochanekSpline> spline;
	spline->SetDefaultTension(.5);

	vtkNew<vtkSplineFilter> sf;
	sf->SetInputConnection(stripper->GetOutputPort());
	sf->SetSubdivideToSpecified();
	sf->SetNumberOfSubdivisions(50);
	sf->SetSpline(spline);
	sf->GetSpline()->ClosedOn();

	vtkNew<vtkTubeFilter> tubes;
	tubes->SetInputConnection(sf->GetOutputPort());
	tubes->SetNumberOfSides(8);
	tubes->SetRadius(0.1);

	vtkNew<vtkPolyDataMapper> linesMapper;
	linesMapper->SetInputConnection(tubes->GetOutputPort());
	linesMapper->ScalarVisibilityOff();

	vtkNew<vtkActor> myCircle_actor;
	myCircle_actor->SetMapper(linesMapper);

	return myCircle_actor;
	/*vtkNew<vtkRenderWindow> renderWindow;
	vtkSmartPointer<vtkRenderer> render = vtkSmartPointer<vtkRenderer>::New();

	render->AddActor(mySphere_actor);
	vtkNew<vtkRenderWindowInteractor> iren;

	iren->SetRenderWindow(renderWindow);

	renderWindow->AddRenderer(render);
	renderWindow->Render();
	iren->Initialize();
	iren->Start();*/
}