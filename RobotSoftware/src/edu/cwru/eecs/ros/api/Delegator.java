package edu.cwru.eecs.ros.api;

import matlabcontrol.MatlabConnectionException;
import matlabcontrol.MatlabInvocationException;
import matlabcontrol.MatlabProxy;
import matlabcontrol.MatlabProxyFactory;

/**
 * 
 */
/**
 * @author Zhuofu
 * 
 */
public class Delegator implements GUIListener {

	RobotProxy m_Robot; // delegator is a listener to the robot
	RobotGUI m_gui; // gui is a listener to the delegator, delegator is also a
	// listener of gui
	MatlabProxy m_proxy;

	/**
	 * @param args
	 * @throws MatlabConnectionException
	 * @throws MatlabInvocationException
	 */
	public Delegator() {
		// constructor
	}

	public void proxyRegister(MatlabProxy proxy) {
		m_proxy = proxy;
		m_Robot = new RobotProxy(proxy);
	}

	public void GuiRegister(RobotGUI gui) {
		m_gui = gui;
	}

	public static void main(String[] args) throws MatlabConnectionException,
			MatlabInvocationException {
		MatlabProxyFactory factory = new MatlabProxyFactory();
		MatlabProxy proxy = factory.getProxy();
		Delegator delegator = new Delegator();
		delegator.proxyRegister(proxy);
		// delegator.handleInitialization();
		// System.out.println(delegator._Robot.error);
		Object[] arg = new Object[3];
		double[] as = { -13, -200, 300 };
		double[][] as2 = { { 0, 0, 1.1 }, { 2, 2, 2 } };
		arg[0] = as;
		arg[1] = as2;
		double pausetime = 0.001;
		arg[2] = pausetime;
		Object result = proxy.returningFeval("matTest", 3, arg);
		Object[] a = (Object[]) result;
		double[] b = (double[]) a[0];
		System.out.println("ready");
		// delegator._Robot.moveNeedle(as, as2, pausetime);
		System.out.println("END");
	}

	public void handleInitialization() throws MatlabInvocationException {

		m_Robot.initializeRobot();
		// check the Robot error flag
		if (m_Robot.getError() == 1) {
			// _gui.ReturnTextSet("error! initialize failed");
			m_gui.sendMessage("error! initialization failed");
		} else {
			// _gui.sendMessage("Initialization Success");
		}

	}

	public void handleMoveAction() {

		double[] targetposition = m_gui.getTagetPosition().toMatrix();
		double[] targetdirection = m_gui.getTargetDirection().toMatrix();
		// double[] currentposition=_Robot.getCurrentPosition().toMatrix();
		// double[] currentdirection=_Robot.getCurrentDirection().toMatrix();
		// _Robot.moveTest()
		// convert to array.
		double pausetime = 100;
		m_gui.sendMessage("Moving to the ready Position");
		boolean flag = m_Robot.moveNeedle(targetposition, targetdirection,
				pausetime);
		if (flag == true) {
			String Output = outputStatus();
			m_gui.sendMessage(Output);
		} else {
			m_gui.sendMessage("Error! Needle movement failed");
		}
	}

	public void handleInsertion() {

		double dist = m_gui.getDepth();
		// double[] currentposition=_Robot.getCurrentPosition().toMatrix();
		// double[] currentdirection=_Robot.getCurrentDirection().toMatrix();
		// _Robot.moveTest()
		// convert to array.
		double pausetime = 100;

		boolean flag = m_Robot.insert(dist, pausetime);
		if (flag == true) {
			String Output = outputStatus();
			m_gui.sendMessage("Needle Insertion Finish");
			m_gui.sendMessage(Output);
		} else {
			m_gui.sendMessage("Error! Needle Insertion failed");
		}
	}

	@Override
	/*
	 * public void cancleChangeNeedleDirection() { // TODO Auto-generated method
	 * stub }
	 */
	public void handleExtraction() {
		double pausetime = 100;
		boolean flag = m_Robot.extractNeedle();
		if (flag == true) {
			String Output = outputStatus();
			m_gui.sendMessage("Needle Extraction Finish");
			m_gui.sendMessage(Output);

		} else {
			m_gui.sendMessage("Error! Needle extraction failed");
		}
	}

	public void handleMoveHome() {
		double pausetime = 100;
		boolean flag = m_Robot.moveHome();
		if (flag == true) {
			String Output = outputStatus();
			m_gui.sendMessage("Needle Extraction Finish");
			m_gui.sendMessage(Output);

		} else {
			m_gui.sendMessage("Error! Needle extraction failed");
		}
	}

	public void changeNeedleDirection(double alpha, double beta) {
		// TODO Auto-generated method stub
		double z = Math.cos(alpha * Math.PI / 180);
		double y = Math.sin(alpha * Math.PI / 180)
				* Math.sin(beta * Math.PI / 180);
		double x = Math.sin(alpha * Math.PI / 180)
				* Math.cos(beta * Math.PI / 180);
		double[] targetdirection = { x, y, z };
		double pausetime = 100;
		boolean flag = false;

		// boolean flag = _Robot.changeDirection(targetdirection, pausetime);
		if (flag == true) {
			String Output = outputStatus();
			m_gui.sendMessage(Output);
		} else {
			m_gui.sendMessage("This function is under building ");
		}

	}

	public void handleSequenceMove() {
		double[] targetposition = m_gui.getTagetPosition().toMatrix();
		double[] targetdirection = m_gui.getTargetDirection().toMatrix();
		double dist = m_gui.getDepth();
		double pausetime1 = 0;
		double pausetime2 = 1;
		boolean flag = false;
		boolean checkflag = false;
		int debuglevel = 1;
		if (dist == 0) {
			m_gui.sendMessage("Insert distance can not be 0.");
		} else {
			double[][] result = m_Robot.singleInsertion(targetposition,
					targetdirection, dist, pausetime1, pausetime2, debuglevel,
					checkflag);
			if (result.length > 0) {
				flag = true;
			}
			if (flag == true) {
				String Output = outputStatus();
				m_gui.sendMessage("Sequence Finish");
				m_gui.sendMessage(Output);
			} else {
				m_gui.sendMessage("Error! Sequence move failed. ");
			}
		}
	}

	public void handleMultipleInsertion() {
		double[] targetposition = m_gui.getTagetPosition().toMatrix();
		double[] targetdirection = m_gui.getTargetDirection().toMatrix();
		double dist = m_gui.getDepth();
		double pausetime = 100;
		boolean flag = false;
		double[][] result = multipleInsertion1(targetposition, targetdirection,
				dist, pausetime);
		if (result != null) {
			String Output = outputStatus();
			m_gui.sendMessage("Sequence Finish");
			m_gui.sendMessage(Output);
		} else {
			m_gui.sendMessage("Error! Sequence move failed.");
		}
	}

	public double[][] multipleInsertion1(double[] targetposition,
			double[] targetdirection, double dist, double pausetime /*
																	 * default
																	 * 100
																	 */) {
		m_Robot.reset();
		m_Robot.turnOnSequenceMove();
		// _Robot.
		m_Robot.moveNeedle(targetposition, targetdirection, pausetime);
		System.out.println("Ref: Move needle done!");
		m_Robot.insert(dist, pausetime);
		System.out.println("Ref: Insert done!");
		m_Robot.extractNeedle();
		System.out.println("Ref: Extract done!");
		m_Robot.moveHome();
		System.out.println("Ref: Move home done!");
		double[][] result = m_Robot.runsimmulator();
		return result;
	}

	public void handleReset() {
		boolean flag = m_Robot.reset();
		if (flag == true) {
			String Output = outputStatus();
			m_gui.sendMessage("Reset Finish");

		} else {
			m_gui.sendMessage("Error! Reset failed");
		}
	}

	public void handleDataCollection() {
		m_Robot.turnOnDataCollection();
	}

	private String outputStatus() {
		// _Robot.status;
		String Position = "Needle Position: "
				+ "X "
				+ Double.toString(m_Robot.getCurrentStatus().actualNeedleTipPosition.x)
				+ "Y "
				+ Double.toString(m_Robot.getCurrentStatus().actualNeedleTipPosition.y)
				+ "Z "
				+ Double.toString(m_Robot.getCurrentStatus().actualNeedleTipPosition.z)
				+ "\n";

		String Direction = "Needle Direction: " + "Rx "
				+ Double.toString(m_Robot.getCurrentStatus().actualR.Rx)
				+ "Ry "
				+ Double.toString(m_Robot.getCurrentStatus().actualR.Ry)
				+ "Rz "
				+ Double.toString(m_Robot.getCurrentStatus().actualR.Rz) + "\n";
		String Force = "Needle Force: "
				+ Double.toString(m_Robot.getCurrentStatus().needleForce)
				+ "\n";
		String Depth = "Needle Depth: "
				+ Double.toString(m_Robot.getCurrentStatus().needleDepth)
				+ "\n";
		String Output = Position + Direction + Force + Depth;

		return Output;
	}
}
