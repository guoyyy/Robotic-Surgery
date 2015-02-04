package org.RobotGUI;

public class Vector {
	double Rx;
	double Ry;
	double Rz;
	public Vector() {
		// TODO Auto-generated constructor stub
		Rx=0;
		Ry=0;
		Rz=0;
	}
	public void set(double Rx, double Ry, double Rz){
		this.Rx=Rx;
		this.Ry=Ry;
		this.Rz=Rz;
	}
	public double[] toMatrix(){
		double []result=new double[3];
		result[0]=this.Rx;
		result[1]=this.Ry;
		result[2]=this.Rz;
		return result;
	}
	public void normalize(){
		double sum=Math.sqrt(Rx*Rx+Ry*Ry+Rz*Rz);
		Rx=Rx/sum;
		Ry=Ry/sum;
		Rz=Rz/sum;
	}

}
