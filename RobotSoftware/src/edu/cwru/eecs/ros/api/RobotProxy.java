package edu.cwru.eecs.ros.api;

import java.util.logging.Level;
import java.util.logging.Logger;
import matlabcontrol.MatlabInvocationException;
import matlabcontrol.MatlabProxy;

//Robot API
public class RobotProxy {

	private Point homePosition, currentPosition, readyPosition;
	private DirVector homeVector, currentVector, readyVector;
	private boolean isInitialized;
	private int isError;// error flag;
	double turnaroundTime;// for single insertion
	private String errorType;
	private MatlabProxy mProxy;// The proxy used to communicate with Matlab
	private RobotStatus status;
	private boolean isInside;
	// private SoftwareStateProfile state;
	private RefTrajectory ref;
	private NewConsumer consumer;
	private NewProducer producer;
	private NewQueue queue = new NewQueue();
	private boolean isCollectData = false;
	private int fileindex;
	private int numEncoderToFail = 0;
	private double encodrFailureTime = 0;
	private double[] maxAcc = new double[3];
	private double ts;
	private int dataDimension;

	//

	public RobotProxy(MatlabProxy proxy) {
		mProxy = proxy;
		isInitialized = false;
		isError = 2;
		turnaroundTime = 0;
		errorType = null;
		isInside = false;
		// state = new SoftwareStateProfile();
		ts = 0.01;
		ref = new RefTrajectory(ts);
		dataDimension=147;
		maxAcc[0] = 25;
		maxAcc[1] = 25;
		maxAcc[2] = 10;
	}

	public void initializeRobot() throws MatlabInvocationException {
		// If initialize failed return result =1, otherwise result=0;
		// Object result = Mproxy.returningEval("initializeRobot", 1);
		homePosition = new Point();
		homeVector = new DirVector();
		currentPosition = new Point();
		currentVector = new DirVector();
		readyPosition = new Point();
		readyVector = new DirVector();

		Object[] results = mProxy.returningEval("API_initializeRobot", 1);
		setupConstants();//set up the constant variables in matlab workspace
		Object result = results[0];
		isError = (int) (((double[]) result)[0]);
		if (isError == 0) {
			homePosition.set(-10.7505, -206.2838, 330.8692);
			homeVector.set(-0.0327, 0.0020, 0.9995);
			currentPosition.set(-10.7505, -206.2838, 330.8692);
			currentVector.set(-0.0327, 0.0020, 0.9995);
			status = new RobotStatus();
			isInitialized = true;
		}
	}

	public boolean moveNeedle(double[] targetposition,
			double[] targetdirection, double pausetime) {
		//
		double maxahome = maxAcc[0];
		double maxwdot = maxAcc[2];
		SoftwareState swstate = new SoftwareState();
		swstate.set("move-needle", maxahome, targetposition, targetdirection,
				isInside);
		readyPosition.set(targetposition[0], targetposition[1],
				targetposition[2]);
		readyVector.set(targetdirection[0], targetdirection[1],
				targetdirection[2]);
		swstate.fieldInf(homePosition, homeVector, currentPosition,
				currentVector, readyPosition, readyVector);
		swstate.setPauseTime(pausetime);

		//
		Object[] args = new Object[8];
		args[0] = homePosition.toMatrix();
		args[1] = homeVector.toMatrix();
		args[2] = targetposition;
		args[3] = targetdirection;
		args[4] = maxahome;
		args[5] = maxwdot;
		args[6] = pausetime;
		args[7] = ts;

		try {
			Object result = mProxy.returningFeval("API_moveNeedle", 4, args);

			ref.gettrajectory(result);

			// Producer store the software state of move needle action into the
			// state queue
			if (isCollectData == true) {
				swstate.setEstimateTimeLength(getTimeLength(result));
				// state.profile.add(swstate);
				try {
					swstate.setRefNeedleDepth(getRefNeedleDepth(ref));
					swstate.addAddtionalRefInfo(currentPosition.toMatrix(),
							targetposition, result);
					swstate.setStartAndEnd(result);// record the begin time and
													// end time of this action.
					producer.ProduceSoftwareState(swstate);
				} catch (InterruptedException ex) {
					Logger.getLogger(RobotProxy.class.getName()).log(
							Level.SEVERE, null, ex);
				}
			}

			if (ref.dosequence == true) {
				// If this action is part of a set of sequence operations,
				// update the cuurent needle positon
				// and direction and set up ready position and direction
				currentPosition.set(targetposition[0], targetposition[1],
						targetposition[2]);
				currentVector.set(targetdirection[0], targetdirection[1],
						targetdirection[2]);
				return true;
			} else {
				Object[] args2 = new Object[7];
				args2[0] = targetposition;
				args2[1] = targetdirection;
				args2[2] = 1;
				args2[3] = ref.time;
				args2[4] = ref.TotalTime;
				args2[5] = ref.SABiRx;
				args2[6] = ref.Rneedle;
				Object[] result1 = mProxy.returningFeval("API_simulateRobot",
						1, args2);
				double[][] result2 = ParseResult.formatResult(result1[0], 46);
				status.update(result2);
				readyPosition.set(status.actualNeedleTipPosition.x,
						status.actualNeedleTipPosition.y,
						status.actualNeedleTipPosition.z);
				readyVector.set(status.actualR.Rx, status.actualR.Ry,
						status.actualR.Rz);
				currentPosition.set(status.actualNeedleTipPosition.x,
						status.actualNeedleTipPosition.y,
						status.actualNeedleTipPosition.z);
				currentVector.set(status.actualR.Rx, status.actualR.Ry,
						status.actualR.Rz);
				return true;
			}
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("Wrong");
			return false;
		}
	}

	public boolean insert(double dist, double pausetime) {
		// Software stae collection
		double maxa = maxAcc[1];
		double maxwdot = maxAcc[2];
		SoftwareState swstate = new SoftwareState();

		double[] targetposition = {
				currentPosition.x + currentVector.Rx * dist,
				currentPosition.y + currentVector.Ry * dist,
				currentPosition.z + currentVector.Rz * dist };
		double[] targetdirection = currentVector.toMatrix();
		swstate.set("insert-needle", maxa, targetposition, targetdirection,
				isInside);
		swstate.setPauseTime(pausetime);
		swstate.setDist(dist);
		swstate.fieldInf(homePosition, homeVector, currentPosition,
				currentVector, readyPosition, readyVector);

		// Get the reference trajectory
		Object[] args = new Object[8];
		args[0] = currentPosition.toMatrix();
		args[1] = currentVector.toMatrix();
		args[2] = dist;
		args[3] = ref.time;
		args[4] = maxa;
		args[5] = maxwdot;
		args[6] = pausetime;
		args[7] = ts;

		try {
			Object result = mProxy.returningFeval("API_insertNeedle", 4, args);

			ref.gettrajectory(result);

			// Producer store the software state of needle insertion into the
			// state queue
			if (isCollectData == true) {
				swstate.setRefNeedleDepth(getRefNeedleDepth(ref));
				getTimeLength(result);
				swstate.setEstimateTimeLength(getTimeLength(result));
				// state.profile.add(swstate);
				try {
					swstate.addAddtionalRefInfo(currentPosition.toMatrix(),
							targetposition, result);
					swstate.setStartAndEnd(result);// record the begin time and
													// end time of this action.
					producer.ProduceSoftwareState(swstate);
				} catch (InterruptedException ex) {
					Logger.getLogger(RobotProxy.class.getName()).log(
							Level.SEVERE, null, ex);
					System.out.println("Producer Interrupted");
				}
			}

			if (ref.dosequence == true) {
				// If this action is part of a set of sequence operations,
				// update the cuurent needle positon and direction
				currentPosition.set(targetposition[0], targetposition[1],
						targetposition[2]);
				currentVector.set(targetdirection[0], targetdirection[1],
						targetdirection[2]);
				return true;
			} else {
				Object[] args2 = new Object[7];
				args2[0] = targetposition;
				args2[1] = targetdirection;
				args2[2] = 1;
				args2[3] = ref.time;
				args2[4] = ref.TotalTime;
				args2[5] = ref.SABiRx;
				args2[6] = ref.Rneedle;
				Object[] result1 = mProxy.returningFeval("API_simulateRobot",
						1, args2);
				double[][] result2 = ParseResult.formatResult(result1[0], 46);
				status.update(result2);

				currentPosition.set(status.actualNeedleTipPosition.x,
						status.actualNeedleTipPosition.y,
						status.actualNeedleTipPosition.z);
				currentVector.set(status.actualR.Rx, status.actualR.Ry,
						status.actualR.Rz);
				return true;
			}

		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("Wrong");
			return false;
		}
	}

	public boolean extractNeedle() {
		double maxa = maxAcc[1];
		double maxwdot = maxAcc[2];
		SoftwareState swstate = new SoftwareState();
		swstate.set("extract-needle", maxa, readyPosition.toMatrix(),
				readyVector.toMatrix(), isInside);
		swstate.fieldInf(homePosition, homeVector, currentPosition,
				currentVector, readyPosition, readyVector);

		Object[] args = new Object[8];

		args[0] = currentPosition.toMatrix();
		args[1] = currentVector.toMatrix();
		args[2] = readyPosition.toMatrix();
		args[3] = readyVector.toMatrix();
		args[4] = ref.time;
		args[5] = maxa;
		args[6] = maxwdot;
		args[7] = ts;

		if (ref.time.length > 0) {
			turnaroundTime = ref.time[ref.time.length - 1];
		} else {
			turnaroundTime = 0;
		}
		try {
			Object result = mProxy.returningFeval("API_extractNeedle", 4, args);

			ref.gettrajectory(result);

			// Producer store the software state of needle extraction into the
			// state queue
			if (isCollectData == true) {
				swstate.setRefNeedleDepth(getRefNeedleDepth(ref));
				swstate.setEstimateTimeLength(getTimeLength(result));
				// state.profile.add(swstate);
				try {
					swstate.addAddtionalRefInfo(currentPosition.toMatrix(),
							readyPosition.toMatrix(), result);
					swstate.setStartAndEnd(result);// record the begin time and
													// end time of this action.
					producer.ProduceSoftwareState(swstate);
				} catch (InterruptedException ex) {
					Logger.getLogger(RobotProxy.class.getName()).log(
							Level.SEVERE, null, ex);
				}
			}

			if (ref.dosequence == true) {
				// If this action is part of a set of sequence operations,
				// update the cuurent needle positon and direction
				currentPosition.set(readyPosition.x, readyPosition.y,
						readyPosition.z);
				currentVector.set(readyVector.Rx, readyVector.Ry,
						readyVector.Rz);
				return true;
			} else {
				// simulate needle extraction with simulink
				Object[] args2 = new Object[7];
				args2[0] = readyPosition.toMatrix();
				args2[1] = readyVector.toMatrix();
				args2[2] = 1;
				args2[3] = ref.time;
				args2[4] = ref.TotalTime;
				args2[5] = ref.SABiRx;
				args2[6] = ref.Rneedle;
				Object[] result1 = mProxy.returningFeval("API_simulateRobot",
						1, args2);
				double[][] result2 = ParseResult.formatResult(result1[0], 46);
				status.update(result2);
				currentPosition.set(status.actualNeedleTipPosition.x,
						status.actualNeedleTipPosition.y,
						status.actualNeedleTipPosition.z);
				currentVector.set(status.actualR.Rx, status.actualR.Ry,
						status.actualR.Rz);

				return true;
			}
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("Wrong");
			return false;
		}
	}

	public boolean moveHome() {
		double maxahome = maxAcc[0];
		double maxwdot = maxAcc[2];
		SoftwareState swstate = new SoftwareState();
		swstate.set("move-home", maxahome, homePosition.toMatrix(),
				homeVector.toMatrix(), isInside);
		swstate.fieldInf(homePosition, homeVector, currentPosition,
				currentVector, readyPosition, readyVector);
		// state.profile.add(swstate);
		double pausetime=1.0;
		swstate.setPauseTime(pausetime);

		Object[] args = new Object[8];
		args[0] = readyPosition.toMatrix();
		args[1] = readyVector.toMatrix();
		args[2] = homePosition.toMatrix();
		args[3] = homeVector.toMatrix();
		args[4] = ref.time;
		args[5] = maxahome;// default speed
		args[6] = maxwdot;
		args[7] = ts;

		try {
			Object result = mProxy.returningFeval("API_moveHome", 4, args);

			ref.gettrajectory(result);

			// Producer store the software state of moving home into the state
			// queue
			if (isCollectData == true) {
				swstate.setRefNeedleDepth(getRefNeedleDepth(ref));
				swstate.setEstimateTimeLength(getTimeLength(result));
				// state.profile.add(swstate);
				try {
					swstate.addAddtionalRefInfo(currentPosition.toMatrix(),
							homePosition.toMatrix(), result);
					swstate.setStartAndEnd(result);// record the begin time and
													// end time of this action.
					producer.ProduceSoftwareState(swstate);
				} catch (InterruptedException ex) {
					Logger.getLogger(RobotProxy.class.getName()).log(
							Level.SEVERE, null, ex);
				}
			}

			if (ref.dosequence == true) {
				return true;
			} else {
				// Simulate move home with simulink
				Object[] args2 = new Object[7];
				args2[0] = homePosition.toMatrix();
				args2[1] = homeVector.toMatrix();
				args2[2] = 1;
				args2[3] = ref.time;
				args2[4] = ref.TotalTime;
				args2[5] = ref.SABiRx;
				args2[6] = ref.Rneedle;
				Object[] result1 = mProxy.returningFeval("API_simulateRobot",
						1, args2);
				double[][] result2 = ParseResult.formatResult(result1[0], 46);
				status.update(result2);

				currentPosition.set(status.actualNeedleTipPosition.x,
						status.actualNeedleTipPosition.y,
						status.actualNeedleTipPosition.z);
				currentVector.set(status.actualR.Rx, status.actualR.Ry,
						status.actualR.Rz);
				return true;
			}
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("Wrong");
			return false;
		}
	}

	public void relativeMove(double delta, double azimuth, double elevation) {
	}

	public boolean relativeMove2(double[] targetposition,
			double[] targetdirection, double speed) {
		// profile the software state
		SoftwareState swstate = new SoftwareState();
		swstate.set("relative-move", speed, targetposition, targetdirection,
				isInside);
		swstate.fieldInf(homePosition, homeVector, currentPosition,
				currentVector, readyPosition, readyVector);
		// state.profile.add(swstate);
		//
		Object[] args = new Object[6];
		args[0] = currentPosition.toMatrix();
		args[1] = currentVector.toMatrix();
		args[2] = targetposition;
		args[3] = targetdirection;
		args[4] = speed;
		args[5] = ref.time;
		try {
			Object result = mProxy.returningFeval("API_relativeMove", 4, args);

			ref.gettrajectory(result);

			// Producer store the software state of move needle action into the
			// state queue
			if (isCollectData == true) {
				swstate.setEstimateTimeLength(getTimeLength(result));
				// state.profile.add(swstate);
				try {
					swstate.addAddtionalRefInfo(currentPosition.toMatrix(),
							targetposition, result);
					swstate.setStartAndEnd(result);// record the begin time and
													// end time of this action.
					producer.ProduceSoftwareState(swstate);
				} catch (InterruptedException ex) {
					Logger.getLogger(RobotProxy.class.getName()).log(
							Level.SEVERE, null, ex);
				}
			}
			if (ref.dosequence == true) {
				currentPosition.set(targetposition[0], targetposition[1],
						targetposition[2]);
				currentVector.set(targetdirection[0], targetdirection[1],
						targetdirection[2]);
				readyPosition.set(targetposition[0], targetposition[1],
						targetposition[2]);
				readyVector.set(targetdirection[0], targetdirection[1],
						targetdirection[2]);
				return true;
			} else {
				double[][] result2 = runsimmulator();
				status.update(result2);
				readyPosition.set(status.actualNeedleTipPosition.x,
						status.actualNeedleTipPosition.y,
						status.actualNeedleTipPosition.z);
				readyVector.set(status.actualR.Rx, status.actualR.Ry,
						status.actualR.Rz);
				currentPosition.set(status.actualNeedleTipPosition.x,
						status.actualNeedleTipPosition.y,
						status.actualNeedleTipPosition.z);
				currentVector.set(status.actualR.Rx, status.actualR.Ry,
						status.actualR.Rz);
				return true;
			}
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("Wrong");
			return false;
		}
	}

	public double[][] runsimmulator() {
		System.out.println("simulator is running");
		Object[] args2 = new Object[10];
		args2[0] = readyPosition.toMatrix();
		args2[1] = readyVector.toMatrix();
		args2[2] = ref.time;
		args2[3] = ref.TotalTime;
		args2[4] = ref.SABiRx;
		args2[5] = ref.Rneedle;
		args2[6] = turnaroundTime;
		args2[7] = numEncoderToFail;
		args2[8] = encodrFailureTime;
		args2[9] = ts;

		Object[] result1;
		SoftwareState swstate = new SoftwareState();
		String error;
		// state.profile.add(swstate);

		try {
			
			result1 = mProxy.returningFeval("API_simulateRobot", 2, args2);
																			 
			error = (String) result1[1];
			if (error.length() > 1) {
				errorType = error;
				double[][] fakeHardware = new double[1][1];
				try {
					swstate.setHardwareData(fakeHardware);
					producer.ProduceSoftwareState(swstate);
				} catch (InterruptedException ex) {
					Logger.getLogger(RobotProxy.class.getName()).log(
							Level.SEVERE, null, ex);
				}
				return null;
			}
			double[][] result2 = ParseResult.formatResult(result1[0],dataDimension);
			try {
				
				swstate.setHardwareData(result2);
				producer.ProduceSoftwareState(swstate);
			} catch (InterruptedException ex) {
				Logger.getLogger(RobotProxy.class.getName()).log(Level.SEVERE,
						null, ex);
				System.out.println("Producer Interrupted");
			}
			return result2;
		} catch (MatlabInvocationException e) {
			e.printStackTrace();
			System.out.println("Matlab Error");
			return null;
		}
	}

	public double[][] singleInsertion(double[] readyposition,
			double[] readydirection, double dist, double pausetime1,double pausetime2 /*
																	 * default
																	 * 100
																	 */,
			int debuglevel, boolean checkflag) {
		reset();
		try {
			setupGlobalVar();
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		ref.turnonsequcemove();
		moveNeedle(readyposition, readydirection, pausetime1);
		boolean isValid=checkRefTrajectory();
		if (!isValid)
		{	
			System.out.println("single insertion failed due to " + errorType);
			System.out.println(errorType + ": Joint angles is outof boundary");
			return null;
		}
		System.out.println("Ref: Move needle done!");
		
		insert(dist, pausetime2);
		System.out.println("Ref: Insert done!");
		extractNeedle();
		System.out.println("Ref: Extract done!");
		moveHome();
		System.out.println("Ref: Move home done!");
		// double[] enddirection = readydirection;
		Point pend = new Point();
		pend.set(readyposition[0], readyposition[1], readyposition[2]);
		pend.move2(dist, readydirection);
//		if (checkflag) {
//			// do check reference trajectory
//			boolean valid = checkRefTrajectory();
//			if (!valid) {
//				System.out.println(errorType);
//			}
//		}

		double[][] result2 = runsimmulator();
		if (result2 == null) {
			System.out.println("single insertion failed due to " + errorType);
		} else {
			if (debuglevel > 0) {
				plotDebugGraph(pend); // show debug graphs
			}
			if (checkflag) {// do check actual trajectory}
			}

			status.update(result2);
			currentPosition.set(status.actualNeedleTipPosition.x,
					status.actualNeedleTipPosition.y,
					status.actualNeedleTipPosition.z);
			currentVector.set(status.actualR.Rx, status.actualR.Ry,
					status.actualR.Rz);
		}
		ref.setdefault();//clear the memory of reference trajectory
		return result2;
	}

	public double[][] MultipleInsertion(double[][] tps, double[][] tds,
			double[] indists, double pausetime /* default 100 */,
			int debuglevel, boolean checkflag) {
		// home to ready
		reset();
		turnOnSequenceMove();
		double[] targetposition = new double[3];
		double[] targetdirection = new double[3];
		// double speed = 100;
		double dist = 0;
		// _Robot.
		for (int i = 0; i < indists.length; i++) {
			for (int j = 0; j < 3; j++) {
				targetposition[j] = tps[i][j];
				targetdirection[j] = tds[i][j];
			}
			dist = indists[i];
			if (i == 0) {
				moveNeedle(targetposition, targetdirection, pausetime);
				System.out.println("Ref: Move needle done!");
			} else {
				relativeMove2(targetposition, targetdirection, pausetime);
				System.out.println("Ref: Move for another insertion!");
			}
			insert(dist, pausetime);
			System.out.println("Ref: Insert done!");
			extractNeedle();
			System.out.println("Ref: Extract done!");
			if (i == indists.length - 1) {
				moveHome();
				System.out.println("Ref: Move home done!");
			}
		}
		Point pend = new Point();
		pend.set(tps[0][0], tps[0][1], tps[0][2]);
		pend.move2(indists[0], tds[0]);

		double[][] result2 = runsimmulator();
		if (result2 == null) {
			System.out.println("single insertion failed due to " + errorType);
		} else {
			if (debuglevel > 0) {
				plotDebugGraph(pend); // show debug graphs
			}
			if (checkflag) {// do check actual trajectory}
			}

			status.update(result2);
			currentPosition.set(status.actualNeedleTipPosition.x,
					status.actualNeedleTipPosition.y,
					status.actualNeedleTipPosition.z);
			currentVector.set(status.actualR.Rx, status.actualR.Ry,
					status.actualR.Rz);
		}
		ref.setdefault();
		return result2;
	}

	public boolean checkValidPose(Point pneedle, DirVector rneedle) {
		Object[] args2 = new Object[2];
		args2[0] = pneedle.toMatrix();
		args2[1] = rneedle.toMatrix();
		Object[] result;
		boolean isValid;
		String error;
		try {
			result = mProxy.returningFeval("API_checkPoint", 2, args2);
			error = (String) result[1];
			double[] checkReturn = (double[]) result[0];

			if (checkReturn[0] > 0) {
				isValid = true;
			} else {
				isValid = false;
				errorType = error;
				System.out.println("ERROR TYPE:" + errorType);
			}
			return isValid;
		} catch (MatlabInvocationException e) {
			e.printStackTrace();
			System.out.println("Error: matlab exception");
			return false;
		}
	}

	public boolean checkRefTrajectory() {
		Object[] args2 = new Object[2];
		args2[0] = ref.SABiRx;
		args2[1] = ref.Rneedle;
		Object[] result;
		boolean isValid;

		String error;
		try {
			result = mProxy.returningFeval("API_checkRefTrajectory", 2, args2);
			double[] checkReturn = (double[]) result[0];
			error = (String) result[1];
			if (checkReturn[0] > 0) {
				isValid = true;
			} else {
				isValid = false;
				errorType = error;
			}
			return isValid;
		} catch (MatlabInvocationException e) {
			e.printStackTrace();
			System.out.println("Error: matlab exception");
			return false;
		}
	}

	public void plotDebugGraph(Point pend) {
		Object[] args2 = new Object[6];
		args2[0] = homePosition.toMatrix();
		args2[1] = readyPosition.toMatrix();
		args2[2] = pend.toMatrix();
		args2[3] = ref.SABiRx;
		args2[4] = fileindex;
		args2[5] = ts;
		Object[] result;

		try {
			result = mProxy.returningFeval("API_debugGraphs", 1, args2);
			double[] checkReturn = (double[]) result[0];
			if (checkReturn[0] > 0) {
				errorType = "Fail to plot debug graphs, reference trajectory or actual trajector is lost";
			}
		} catch (MatlabInvocationException e) {
			e.printStackTrace();
			System.out.println("Error: matlab exception");
		}
	}

	

	public boolean reset() {
		homePosition.set(-10.7505, -206.2838, 330.8692);
		homeVector.set(-0.0327, 0.0020, 0.9995);
		currentPosition.set(-10.7505, -206.2838, 330.8692);
		currentVector.set(-0.0327, 0.0020, 0.9995);
		status = new RobotStatus();
		isInitialized = true;
		ref.setdefault();
		isError = 0;
		errorType = null;
		turnaroundTime = 0;

		try {
 			Object[] result = mProxy.returningEval("API_resetRobot", 1);
			isError = (int) (((double[]) result[0])[0]);
			if (isError == 1) {
				System.out.println("Reset System");
			}
		} catch (MatlabInvocationException e) {
			e.printStackTrace();
			System.out.println("Error: matlab exception");
		}
		return true;
	}

	private void setupConstants() throws MatlabInvocationException {
		mProxy.eval("setupConstants");
	}
	private void setupGlobalVar() throws MatlabInvocationException {
		mProxy.eval("setupGlobals");
	}
	public void turnOnSequenceMove() {
		ref.turnonsequcemove();
	}

	public void lastCommandStatus() {
	}

	//
	public RobotStatus getCurrentStatus() {
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

	public DirVector getCurrentDirection() {
		return this.currentVector;
	}

	public int getError() {
		return this.isError;
	}

	public void turnOnDataCollection() {
		producer = new NewProducer(queue);
		consumer = new NewConsumer(queue);
		isCollectData = true;
		// new Thread(producer).start();
		// new Thread(consumer).start();
		producer.start();
		consumer.start();
	}

	public void turnOffDataCollection() {
		SoftwareState sw2 = new SoftwareState();
		sw2 = new SoftwareState();
		sw2.setEndSignal();
		if (producer == null) {
			System.out.println("Data collection is already closed");
		} else {
			try {
				producer.ProduceSoftwareState(sw2);
			} catch (InterruptedException ex) {
				Logger.getLogger(RobotProxy.class.getName()).log(Level.SEVERE,
						null, ex);
			}

			System.out.println("Data collection closed");
		}
		isCollectData = false;
	}

	public void nameDateFile(String dir, int index) {
		if (consumer == null) {
			System.out.println("consumer is not initialized");
		} else {

			fileindex = index;
			consumer.setDir(dir);
		}
	}

	public void generateEncoderFailure(int numOfEncoder, double failureTime) {
		numEncoderToFail = numOfEncoder;
		encodrFailureTime = failureTime;

	}
	
	public double getNeedleDepth(double[] pneedle, double[] rneedle) {
		Object[] args2 = new Object[2];
		args2[0] = pneedle;
		args2[1] = rneedle;
		Object[] result;
		try {
			result = mProxy.returningFeval("API_getNeedleDepth", 1, args2);
			double[] checkReturn = (double[]) result[0];
			return checkReturn[0];
		} catch (MatlabInvocationException e) {
			e.printStackTrace();
			System.out.println("Error: matlab exception");
			return -1;
		}

	}
	public double[] getRefNeedleDepth(RefTrajectory ref){
		double []rn= new double[3];
		double []xn =new double[3];
		int k= ref.tempSa.length;
		double []depth=new double[k];
		for (int i=0;i<k;i++){
			xn[0]=ref.tempSa[i][0];xn[1]=ref.tempSa[i][1];xn[2]=ref.tempSa[i][2];
			rn[0]=ref.tempRn[i][0];rn[1]=ref.tempRn[i][1];rn[2]=ref.tempRn[i][2];
			depth[i]=getNeedleDepth(xn,rn);
		}
		return depth;
	}
	private int getTimeLength(Object result) {
		Object[] result1 = (Object[]) result;
		double[] a = (double[]) result1[3];
		int t = (int) (a[0] * 2000);
		return t;
	}

}
