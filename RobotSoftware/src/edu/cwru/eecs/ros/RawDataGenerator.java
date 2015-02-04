package edu.cwru.eecs.ros;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.text.DecimalFormat;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;

import matlabcontrol.MatlabConnectionException;
import matlabcontrol.MatlabInvocationException;
import matlabcontrol.MatlabProxy;
import matlabcontrol.MatlabProxyFactory;
import matlabcontrol.MatlabProxyFactoryOptions;

import edu.cwru.eecs.ros.api.DataGenerator;
import edu.cwru.eecs.ros.api.DirVector;
import edu.cwru.eecs.ros.api.Point;
import edu.cwru.eecs.ros.api.RobotProxy;

//import util.Config;
public class RawDataGenerator {

	public static Point P_HOME = new Point(-10.7505, -206.2838, 330.8692);
	public static DirVector R_HOME = new DirVector(-0.0327, 0.0020, 0.9995);

	// sweep error #1
	public static Vector<Point> P_READY_SWEEP_ERR;
	public static Vector<DirVector> R_READY_SWEEP_ERR;
	public static Vector<Double> INSERT_DIST_ERR;
	static {
		P_READY_SWEEP_ERR = new Vector<Point>();
		R_READY_SWEEP_ERR = new Vector<DirVector>();
		INSERT_DIST_ERR = new Vector<Double>();
		// sweep error #1
		P_READY_SWEEP_ERR.add(new Point(-9.4233, -216.5671, 338.8466));
		R_READY_SWEEP_ERR.add(new DirVector(0, 0.1, 20));
		INSERT_DIST_ERR.add(Math.sqrt(0 + 0.1 * 0.1 + 20 * 20));

		// sweep error #2
		P_READY_SWEEP_ERR.add(new Point(-12, -175, 345));
		R_READY_SWEEP_ERR.add(new DirVector(0, 0, 10));
		INSERT_DIST_ERR.add(10.0);

		// sweep error #3
		P_READY_SWEEP_ERR.add(new Point(-22, -190, 350));
		R_READY_SWEEP_ERR.add(new DirVector(3, 0, 9));
		INSERT_DIST_ERR.add(Math.sqrt(9 + 81));

		// sweep error #4
		P_READY_SWEEP_ERR.add(new Point(1, -190, 345));
		R_READY_SWEEP_ERR.add(new DirVector(0, 0, 5));
		INSERT_DIST_ERR.add(5.0);

		// sweep error #5
		P_READY_SWEEP_ERR.add(new Point(-8, -210, 342));
		R_READY_SWEEP_ERR.add(new DirVector(0, 0, 5));
		INSERT_DIST_ERR.add(5.0);
	}

	private RobotProxy m_robot;
	private MatlabProxy m_proxy;
	private String m_cwd;

	public RawDataGenerator() {
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

	/**
	 * result[0] pTarget result[1] rTarget == rEnd result[2] insertDist
	 * result[3] pEnd
	 * 
	 * @return
	 */
	private double[][] samplingParameters() {
		// The block corners of the outside gel is: [-20, 0] [-240, -180] [340,
		// 360]
		// The block corner of the inside gel is: [-12, -7] [-220, -200] [345,
		// 350]

		// Sample the end point inside the gel.
		/** phome: -10.7505 -206.2838 330.8692 */
		/** rhome -0.0327, 0.0020, 0.9995 */

		int endTrial = 0;
		int rEndTrial = 0;
		Point pEnd = new Point();
		edu.cwru.eecs.ros.api.DirVector rEnd = new edu.cwru.eecs.ros.api.DirVector();
		Point pReady = new Point();

		while (endTrial < 10) {
			endTrial++;
			// randomly sample a pEnd
			// pEnd.set(Math.random()*20-20, Math.random()*60-240,
			// Math.random()*20+340);
			pEnd.set(Math.random() * 5 - 12, Math.random() * 20 - 220,
					Math.random() * 5 + 345);
			// pick a proper rEnd
			rEndTrial = 0;
			while (rEndTrial < 10) {
				rEndTrial++;
				// TODO: sample rEnd
				// rEnd.set(0.00, 0.000, 1);
				// rEnd.set(-0.0327, 0.0020, 0.9995); // rHome
				rEnd.set(Math.random() - 0.5, Math.random() - 0.5, 1);
				rEnd.normalize();
				// check pEnd, rEnd
				if (m_robot.checkValidPose(pEnd, rEnd)) {
					// now have a good pEnd and rEnd
					// rEndTrial=0;
					// get a good rEnd, pick a proper pReady
					double depth = m_robot.getNeedleDepth(pEnd.toMatrix(),
							rEnd.toMatrix());
					double insertDist = depth + Math.random() * 2; // magic
																	// number 2
					pReady.set(pEnd.x - insertDist * rEnd.Rx, pEnd.y
							- insertDist * rEnd.Ry, pEnd.z - insertDist
							* rEnd.Rz);
					// check pReady, rReady (==rEnd)
					if (m_robot.checkValidPose(pReady, rEnd)) {
						// DONE!
						// endTrial=0;
						double[][] parameters = new double[4][3];
						parameters[0] = pReady.toMatrix();
						parameters[1] = rEnd.toMatrix();
						parameters[2][0] = insertDist;
						parameters[3] = pEnd.toMatrix();
						System.err.println(arrayToString(parameters));
						return parameters;
					}
				}
			}
		}
		// fail after 10*10 trials!
		return null;
	}

	public String arrayToString(double[][] arrays) {
		StringBuffer sb = new StringBuffer();
		for (int i = 0; i < arrays.length; i++) {
			for (int j = 0; j < arrays[0].length; j++)
				sb.append(arrays[i][j] + "\t");
			sb.append("\n");
		}
		return sb.toString();
	}

	public double[][] testParameter() {
		Point pEnd = new Point();
		pEnd.set(-10.7505, -206.2838, 342.8692);

		edu.cwru.eecs.ros.api.DirVector rEnd = new edu.cwru.eecs.ros.api.DirVector();
		rEnd.set(0.00, 0.000, 1);
		rEnd.normalize();
		// chek the validity of needle pose.
		if (!m_robot.checkValidPose(pEnd, rEnd)) {
			System.err.println("This needle pose is invalid at end position.");
			return null;
		}
		/** Get insertdist */
		double insertDist = m_robot.getNeedleDepth(pEnd.toMatrix(),
				rEnd.toMatrix());
		if (insertDist == 0) {
			System.err.println("The end point is outside the gel");
			return null;
		}
		// Get Ready Position & Check the validity of Ready Position
		Point pReady = new Point();
		pReady.set(pEnd.x - insertDist * rEnd.Rx,
				pEnd.y - insertDist * rEnd.Ry, pEnd.z - insertDist * rEnd.Rz);

		if (!m_robot.checkValidPose(pReady, rEnd)) { // rReady=rEnd
			System.out
					.println("This needle pose of is invalid at ready position.");
			return null;
		}
		double[][] parameters = new double[4][3];
		parameters[0] = pReady.toMatrix();
		parameters[1] = rEnd.toMatrix();
		parameters[2][0] = insertDist;
		parameters[3] = pEnd.toMatrix();
		return parameters;
	}

	/**
	 * Generate normal data of single insertion
	 * 
	 * @param dir
	 * @return
	 * @throws MatlabInvocationException
	 */
	public double[][] genData(String dir, int index)
			throws MatlabInvocationException {
		double[][] parameters = samplingParameters();
		if (parameters == null) {
			System.out.println("Sampling failed!");
			return null;
		}
		return genData(parameters[0], parameters[1], parameters[2][0], dir,
				index);
	}

	/**
	 * generate single insertion data with sweep error.
	 * 
	 * @return
	 * @throws MatlabInvocationException
	 */
	public double[][] genDataSweepError(String dir, int index)
			throws MatlabInvocationException {
		return genData(P_READY_SWEEP_ERR.get(index).toMatrix(),
				R_READY_SWEEP_ERR.get(index).toMatrix(),
				INSERT_DIST_ERR.get(index), dir, index);
	}

	/**
	 * generate single insertion data with encoder failure.
	 * 
	 * @param dir
	 * @param index
	 * @param motorIndex
	 * @param failureTime
	 * @return
	 */
	public double[][] genDataEncoderFailure(String dir, int index,
			int motorIndex, double failureTime)
			throws MatlabInvocationException {
		m_robot.generateEncoderFailure(motorIndex, failureTime);
		double[][] data = genData(dir, index);
		m_robot.generateEncoderFailure(0, 0);
		return data;
	}

	/**
	 * 
	 * generate data with given parameters.
	 * 
	 * @param pTarget
	 * @param rTarget
	 * @param insertDist
	 * @param filename
	 * @param index
	 * @return
	 * @throws MatlabInvocationException
	 */
	private double[][] genData(double[] pTarget, double[] rTarget,
			double insertDist, String filename, int index)
			throws MatlabInvocationException {
		m_robot.nameDateFile(filename, index);
		cmdCD(m_cwd);
		double speed = 100;
		int debuglevel = 1; // debug mode switch. debuglevel=1 turn on debug
							// mode, matlab will show
		// trajectory graphs after each simulation. debuglevel=0, turn off
		double pausetime1 = 0;
		double pausetime2 = 0;
		boolean checkflag = false;// check trajectory flag.
		// checkflag=ture, the whole reference trajectory will be checked before
		// simulation.
		// check trajectory may take a long time.

		double[][] result = m_robot.singleInsertion(pTarget, rTarget,
				insertDist, pausetime1, pausetime2, debuglevel, checkflag);
		// String path=request.getRealPath("/example/filetest");
		// File file = new File(filename);

		DecimalFormat df = new DecimalFormat("#.########");
		try {
			RandomAccessFile rf = new RandomAccessFile(filename, "rw");
			try {
				// System.out.println(rf.getFilePointer());
				rf.writeBytes("End Point: " + df.format(pTarget[0]) + " "
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
		cmdClear();
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
}
