package org.RobotGUI;
/**
 * @author Zhuofu
 *
 */
import matlabcontrol.MatlabInvocationException;
import matlabcontrol.RemoteMatlabProxy;

public class RobotProxy {
	private Point homePosition, currentPosition, readyPosition;
	private Vector homeVector, currentVector, readyDirection;
	private boolean isInitialized;
	int error;// error flag;
	RemoteMatlabProxy Mproxy;// The proxy used to communicate with Matlab
	robotStatus status;
	//
	public RobotProxy(RemoteMatlabProxy proxy) {
		Mproxy = proxy;

		homePosition = new Point();
		homeVector = new Vector();
		currentPosition = new Point();
		currentVector = new Vector();
		readyPosition=new Point();
		readyDirection=new Vector();
		isInitialized = false;
		error = 2;
	}

	void initializeRobot() throws MatlabInvocationException {
		// if initialize failed return result =1, otherwise result=0;
		// Object result = Mproxy.returningEval("initializeRobot", 1);
		String a = null;
		Mproxy.setVariable(a, 1);
		Object result = Mproxy.returningEval("RobotInitialization", 1);
		double[] array = (double[]) result;
		error = (int) array[0];//
		if (error==0){
		homePosition.set(-13.2566, -197.9928, 298.5063);
		homeVector.set(0, 0, 1);
		isInitialized = true;}
	}

	double[][] moveNeedle(double[] targetposition, double[] targetdirection,
			double speed) {

		Object[] args = new Object[5];
		args[0] = targetposition;
		args[1] = targetdirection;
		args[2] = homePosition.toMatrix();
		args[3] = homeVector.toMatrix();
		args[4] = speed;
		
		try {
			Object result=Mproxy.returningFeval("MoveNeedle2", args, 1);
			double[][] result1 = ParseResult.formatResult(result,46);
			status=new robotStatus(result1);
		
			//robotStatus
			//readyPosition
			//readyDirection
			return result1;
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("Wrong");
			return null;
		}
	}
	void relativeMove(){}
	void insert(){}
	void moveHome(){}
	void lastCommandStatus(){}
	
	//
	robotStatus getCurrentStatus(){
		return status;
	}
	
		

	// assistant method
	public void currentPositionUpdate(double x, double y, double z) {
		this.currentPosition.set(x, y, z);
	}

	public void currentDirectionUpdate(double Rx, double Ry, double Rz) {
		this.currentVector.set(Rx, Ry, Rz);
	}

	public Point getCurrentPosition() {
		return this.currentPosition;
	}

	public Vector getCurrentDirection() {
		return this.currentVector;
	}
}
