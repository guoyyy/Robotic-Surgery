package edu.cwru.eecs.ros.api;

public class DirVector {
	public double Rx;
	public double Ry;
	public double Rz;
	public DirVector() {
		// TODO Auto-generated constructor stub
		Rx=0;
		Ry=0;
		Rz=0;
	}
	
	public DirVector(double rx, double ry, double rz) {
		Rx = rx;
		Ry = ry;
		Rz = rz;
		normalize();
	}
	//copy constructor
	public DirVector(DirVector another){
		this.Rx=another.Rx;
		this.Ry=another.Ry;
		this.Rz=another.Rz;
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
