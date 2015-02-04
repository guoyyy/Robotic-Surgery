package edu.cwru.eecs.ros.api;

import java.io.File;
//import java.util.Vector;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.text.DecimalFormat;
import java.util.logging.Level;
import java.util.logging.Logger;

import matlabcontrol.MatlabConnectionException;
import matlabcontrol.MatlabInvocationException;
import matlabcontrol.MatlabProxy;
import matlabcontrol.MatlabProxyFactory;
import matlabcontrol.MatlabProxyFactoryOptions;

import edu.cwru.eecs.ros.api.Point;
import edu.cwru.eecs.ros.api.RobotProxy;

//import util.Config;
public class DataGenerator {

	private RobotProxy m_robot;
	private MatlabProxy m_proxy;
	private String m_cwd;

	public DataGenerator() {
		// Set options for matlab proxy factory
		MatlabProxyFactoryOptions options = new MatlabProxyFactoryOptions.Builder()
				.setUsePreviouslyControlledSession(true).build();

		// Create a proxy, which we will use to control MATLAB
		MatlabProxyFactory proxyFactory = new MatlabProxyFactory(options);
		try {
			m_proxy = proxyFactory.getProxy();
			m_robot = new RobotProxy(m_proxy);
			m_cwd = new File("./PD_simulator/trunk/").getAbsolutePath();
			cmdCD(m_cwd);

			m_robot.initializeRobot();

		} catch (MatlabConnectionException e) {
			System.err.println("Create Matlab Proxy failed!");
			e.printStackTrace();
		} catch (MatlabInvocationException e) {
			System.err.println("Initialize robot or change to cwd failed!");
			e.printStackTrace();
		}
	}

	public double[][] genData(String filename, int index)
			throws MatlabInvocationException {
		// The block corners of the gel is:
		// [0 -240 360;
		// 0 -180 360;
		// 0 -240 340;
		// 0 -180 340;
		// -20 -240 360;
		// -20 -180 360;
		// -20 -240 340;
		// -20 -180 340];
		// Sample the end point inside the gel.

		/** phome: -10.7505 -206.2838 330.8692 */
		/** rhome -0.0327, 0.0020, 0.9995 */
		// To generate sweep-type error, use one of the following sets of
		// parameters
		// %sweep error #1
		// % ptarget = [-9.4233, -216.5671, 338.8466]; %the ready point
		// % pendpoint = ptarget + [0 .1 20];% the endpoint
		// % ptarget = phome + [0 0 1];% insert direction
		// By calculatting |pendpoint-ptarget| to get the insertdist
		// For example,
		// pReady.set(-9.4233, -216.5671, 338.8466);
		// pEnd.set(pReady.x+0, pReady.y+0.1, pReady.z+20);
		// insertDist = Math.sqrt(0+0.1*0.1+20*20);
		// % %sweep error #2
		// % ptarget = [-12 -175 345]; %the ready point
		// % pendpoint = ptarget + [0 0 10];the endpoint
		// %
		// % %sweep error #3
		// % ptarget = [-22 -190 350]; %the ready point
		// % pendpoint = ptarget + [3 0 9];the endpoint
		// %
		// % %sweep error #4
		// % ptarget = [1 -190 345]; %the ready point
		// % pendpoint = ptarget + [0 0 5];the endpoint
		// %
		// % %sweep error #4
		// % ptarget = [1 -190 345]; %the ready point
		// % pendpoint = ptarget + [0 0 5];the endpoint
		// %
		// % %sweep error #5 - note that this one will probably be difficult to
		// detect
		// % ptarget = [-8 -210 342]; %the ready point
		// % pendpoint = ptarget + [0 0 5];the endpoint
		Point pReady = new Point();
		// pReady.set(pEnd.x - insertDist * rEnd.Rx,
		// pEnd.y - insertDist * rEnd.Ry, pEnd.z - insertDist * rEnd.Rz);
		pReady.set(-8.0922697, -203.80370234, 339.98304503);
		DirVector rReady=new DirVector( 0.00308453, -0.04070429, 0.99776009);
		rReady.normalize();
		 if (!m_robot.checkValidPose(pReady, rReady)) {
		 System.out
		 .println("This needle pose of is invalid at ready position.");
		 return null;
		 }
		double insertDist =  6.50224753;//+Math.random() * 2;;
		Point pEnd = new Point(pReady);

		DirVector rEnd = new DirVector(rReady);
		// chek the validity of needle pose.
		pEnd.move2(insertDist, rEnd.toMatrix());
		 if (!m_robot.checkValidPose(pEnd, rEnd)) {
		 System.out.println("This needle pose is invalid at end position.");
		 return null;
		 }
		 
		/** Get insertdist */
		insertDist = m_robot.getNeedleDepth(pEnd.toMatrix(),
				rEnd.toMatrix());
		if (insertDist == 0) {
			System.out.println("The end point is outside the gel");
			return null;
		}
		
		// Get Ready Position & Check the validity of Ready Position
		
		double pausetime1 = 0;//pause the needle when reaching the ready point
		double pausetime2 = 0;//pause the needle when reaching the end point
		// Uncomment the following statements to generate encoder failure
		int encoderNumToFail = 0;// indicates which encoder fail.value range
									// [1-5], corresponding to 5 joint angle.
		double timeOfEncoderFail = 0;// encoder failure time.
		m_robot.generateEncoderFailure(encoderNumToFail, timeOfEncoderFail);

		return genData(pReady.toMatrix(), rEnd.toMatrix(), insertDist,
				pausetime1, pausetime2, filename, index);
	}

	private double[][] genData(double[] pTarget, double[] rTarget,
			double insertDist, double pausetime1, double pausetime2, String filename, int index)
			throws MatlabInvocationException {
		m_robot.nameDateFile(filename, index);
		cmdCD(m_cwd);

		int debuglevel = 1; // debug mode swich. debuglevel=1 turn on debug
							// mode, matlab will show
		// trajectory graphs after each simulation. debuglevel=0, turn off
		boolean checkflag = false;// check trajectory flag.
		// checkflag=ture, the whole reference trajectory will be checked before
		// simulation.
		// check trajectory may take a long time.
		double[][] result = m_robot.singleInsertion(pTarget, rTarget,
				insertDist, pausetime1, pausetime2, debuglevel, checkflag);
		// String path=request.getRealPath("/example/filetest");

		File file = new File(filename);
		DecimalFormat df = new DecimalFormat("0.#####E0");
		try {
			RandomAccessFile rf = new RandomAccessFile(filename, "rw");
			try {
				// System.out.println(rf.getFilePointer());
				rf.writeBytes("Ready Point: " + df.format(pTarget[0]) + " "
						+ df.format(pTarget[1]) + " " + df.format(pTarget[2])
						+ "  " + "Direction: " + df.format(rTarget[0]) + " "
						+ df.format(rTarget[1]) + " " + df.format(rTarget[2])
						+ " " + "Insert-Distance: " + df.format(insertDist));
				rf.close();//

			} catch (IOException ex) {
				Logger.getLogger(DataGenerator.class.getName()).log(
						Level.SEVERE, null, ex);
			}
		} catch (FileNotFoundException ex) {
			Logger.getLogger(DataGenerator.class.getName()).log(Level.SEVERE,
					null, ex);
		}

		//cmdClear();
		return result;
	}

	private void cmdCD(String path) throws MatlabInvocationException {
		String cmd = "cd(\'" + path + "\')";
		m_proxy.eval(cmd);
	}

	private void cmdClear() throws MatlabInvocationException {
		String cmd = "clear";
		m_proxy.eval(cmd);
	}

	public void open() {
		m_robot.turnOnDataCollection();
	}

	public void close() {
		m_robot.turnOffDataCollection();
		if (m_proxy != null) {
			m_proxy.disconnect();
		}
	}

	public static void main(String[] args) throws MatlabConnectionException,
			MatlabInvocationException {
		DataGenerator dataGenerator = new DataGenerator();
		
		dataGenerator.open();
		for (int i = 3002; i < 3003; i++) {
			String dir = "RawTrajectory" + (i);
			String fileName = "raw_trajectory/" + dir + ".raw";
			double[][] result = dataGenerator.genData(fileName, i); // Generate
																	// data of
																	// single
																	// insertion
																	// (move
																	// needle,
																	// insert,
																	// extract
																	// needle,
																	// move)

			if (result == null) {
				System.out.println("Fail to generate trajectory");
			} else {
				System.out.println("Data:" + result.length + "*"
						+ result[0].length);
				System.out.println("Head: " + result[0]);
			}
		}
		dataGenerator.close();

	}
}
