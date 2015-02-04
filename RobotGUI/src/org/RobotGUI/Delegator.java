package org.RobotGUI;
/**
 * @author Zhuofu
 *
 */
import matlabcontrol.MatlabConnectionException;
import matlabcontrol.MatlabInvocationException;
import matlabcontrol.RemoteMatlabProxy;
import matlabcontrol.RemoteMatlabProxyFactory;

/**
 * 
 */

/**
 * @author Zhuofu
 * 
 */
public class Delegator implements GUIListener {
	RobotProxy _Robot; // delegator is a listener to the robot
	RobotGUI _gui; // gui is a listener to the delegator, delegator is also a
					// listener of gui
	RemoteMatlabProxy _proxy;

	/**
	 * @param args
	 * @throws MatlabConnectionException
	 * @throws MatlabInvocationException
	 */

	public Delegator() {
		// constructor
	}

	public void proxyRegister(RemoteMatlabProxy proxy) {
		_proxy = proxy;
		_Robot = new RobotProxy(proxy);
	}

	public void GuiRegister(RobotGUI gui) {
		_gui = gui;
	}

	public static void main(String[] args) throws MatlabConnectionException,
			MatlabInvocationException {
		RemoteMatlabProxyFactory factory = new RemoteMatlabProxyFactory();
		RemoteMatlabProxy proxy = factory.getProxy(220000);
		Delegator delegator = new Delegator();
		delegator.proxyRegister(proxy);
		// delegator.handleInitialization();
		// System.out.println(delegator._Robot.error);
//		Object[] arg = new Object[3];
		double[] as = {1,2,3};
		double[] as2={4,5,6};
		//arg[0] = as;
		//arg[1]=as2;
		double speed=0.1;
		//arg[3]=speed;
		//Object result = proxy.returningFeval("matTest", arg, 1);
		System.out.println("ready");
		delegator._Robot.moveNeedle(as, as2, speed);
		System.out.println("END");
	}

	
	public void handleMoveAction() {
		
		double[] targetposition=_gui.getTargetPosition().toMatrix();
		double[] targetdirection=_gui.getTargetDirection().toMatrix();
		//double[] currentposition=_Robot.getCurrentPosition().toMatrix();
		//double[] currentdirection=_Robot.getCurrentDirection().toMatrix();
		//_Robot.moveTest()
		//convert to array.
		double speed=0.1;
		_gui.sendMessage("Moving to the ready Position");
		double [][]a=_Robot.moveNeedle(targetposition,targetdirection,speed);
		_gui.sendMessage("Finish");
		
		//
		//move

	}

	public void handleInitialization() throws MatlabInvocationException {

		_Robot.initializeRobot();
		// check the Robot error flag
		if (_Robot.error == 1) {
			// _gui.ReturnTextSet("error! initialize failed");
			_gui.sendMessage("error! initialization failed");
		} else {
			_gui.sendMessage("Robot is successfully initialized");
		}

	}

	@Override
	public void cancleChangeNeedleDirection() {
		// TODO Auto-generated method stub

	}

	public void changeNeedleDirection(double alpha,double beta) {
		// TODO Auto-generated method stub
		double z=Math.cos(alpha*Math.PI/180);
		double y=Math.sin(alpha*Math.PI/180)*Math.sin(beta*Math.PI/180);
		double x=Math.sin(alpha*Math.PI/180)*Math.cos(beta*Math.PI/180);
		double []targetdirection={x,y,z};
		Vector needleDirection = new Vector();	
		
		_gui.sendMessage("Adjusting Needle orientation...");
		//double [][]a=_Robot.moveNeedle(targetposition,targetdirection,speed);
		_gui.sendMessage("Finish");
		
	}
	

}
