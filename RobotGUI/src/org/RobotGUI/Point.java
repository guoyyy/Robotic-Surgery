package org.RobotGUI;

public class Point {
	double x;
	double y;
	double z;
	public Point() {
		// TODO Auto-generated constructor stub
		x=0;
		y=0;
		z=0;
	}
	
	public void set(double x, double y, double z){
		this.x=x;
		this.y=y;
		this.z=z;
	}
	public double[] toMatrix(){
		double []result=new double[3];
		result[0]=this.x;
		result[1]=this.y;
		result[2]=this.z;
		return result;
	}
}
