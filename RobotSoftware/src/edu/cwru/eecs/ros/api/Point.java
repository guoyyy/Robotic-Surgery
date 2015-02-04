package edu.cwru.eecs.ros.api;

public class Point {

    public double x;
    public double y;
    public double z;

    public Point() {
        // TODO Auto-generated constructor stub
        x = 0;
        y = 0;
        z = 0;
    }

    public Point(double x, double y, double z) {
    	this.x = x;
    	this.y = y;
    	this.z = z;
    }
    public Point(Point another){
		this.x=another.x;
		this.y=another.y;
		this.z=another.z;
	}
    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double[] toMatrix() {
        double[] result = new double[3];
        result[0] = this.x;
        result[1] = this.y;
        result[2] = this.z;
        return result;
    }

    public void move(double dist, DirVector direct) {
        x = x + dist * direct.Rx;
        y = y + dist * direct.Ry;
        z = z + dist * direct.Rz;
    }

    public void move2(double dist, double[] direct) {
        x = x + dist * direct[0];
        y = y + dist * direct[1];
        z = z + dist * direct[2];
    }
}
