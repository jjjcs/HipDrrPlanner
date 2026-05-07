#pragma once
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

/************************************************************************/
/* 类功能：计算髋臼杯覆盖率
/* 使用说明：1.创建实例
/*			 2.计算传入的intersection表面积和基于相同半球切面的下表面表面积之差，除以半球表面积
/*			 3.调用cal_coverage计算髋臼杯覆盖率
/* 编 写 人：宋云鹏
/* 编写时间：2024年04月16日 16时00分
/************************************************************************/

class Coverage_Calculator
{
private:
	vtkSmartPointer<vtkPolyData> m_intersection_polydata;
	vtkSmartPointer<vtkPolyData> m_cup_polydata;
	vtkSmartPointer<vtkPolyData> m_target_hip_polydata;

	double m_cup_sphere_center[3];
	double m_cup_sphere_radius;
	double m_cup_z_axis[3];

	double get_lower_surface_area();
	double get_intersection_surface_area();

public:
	void set_cup_sphere_center(double x, double y, double z)
	{
		m_cup_sphere_center[0] = x;
		m_cup_sphere_center[1] = y;
		m_cup_sphere_center[2] = z;
	}
	void set_cup_sphere_radius(double r)
	{
		m_cup_sphere_radius = r;
	}
	void set_cup_z_axis(double x, double y, double z)
	{
		m_cup_z_axis[0] = x;
		m_cup_z_axis[1] = y;
		m_cup_z_axis[2] = z;
	}

	void set_intersection_polydata(vtkSmartPointer<vtkPolyData> target_intersection_polydata)
	{
		m_intersection_polydata = target_intersection_polydata;
	}
	void set_cup_polydata(vtkSmartPointer<vtkPolyData> target_cup_polydata)
	{
		m_cup_polydata = target_cup_polydata;
	}
	void set_hip_polydata(vtkSmartPointer<vtkPolyData> target_hip_polydata)
	{
		m_target_hip_polydata = target_hip_polydata;
	}

	double cal_coverage();
	void render_projection();
	vtkSmartPointer<vtkActor> draw_cirlce(double* center, double* normal, double radius);
};