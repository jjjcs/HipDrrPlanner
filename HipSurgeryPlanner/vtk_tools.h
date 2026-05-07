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
using namespace std;
using namespace Eigen;

vtkVector3d dot(vtkNew<vtkMatrix3x3>& matrix_in, vtkVector3d vector_in);
void createLineActor(Eigen::Matrix3d points_array, float opacity, string color, float line_width, vtkActor* actor);
void createLineActor(Eigen::Vector3d normal, Eigen::Vector3d center, vtkActor* actor, float opacity = 1, string color = "Red", float line_width = 5);
void createLineActor(std::vector<Eigen::Vector3d> points, vtkActor* actor, float opacity = 1, string color = "Red", float line_width = 5);
void createSphereActor(Eigen::Vector3d center, vtkActor* actor, float radius = 3, float opacity = 1, string color = "Red");
void createSphereActor(float center[3], float radius, float opacity, string color, vtkActor* actor);
void createSphereActor(double center[3], float radius, float opacity, string color, vtkActor* actor);
void show_actors(vector<vtkActor*> actors);
void createAxisActor(Eigen::MatrixXd points_array, string color, vtkActor* actor);
void createCoordActor(Eigen::Matrix3d coordinate, Eigen::Vector3d origin, vector<vtkActor*>& actors, float opacity, int length);
void createBBoxactors(double bounds[6], vector<vtkActor*>& all_actors);
