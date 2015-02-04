/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.cwru.eecs.ros.api;

import matlabcontrol.MatlabProxy;
import java.text.DecimalFormat;
import java.util.ArrayList;

/**
 * 
 * @author Zhuofu
 */
public class SoftwareState {

	private String actionname;
	private double speed;
	private Point targetPosition;
	private DirVector targetDirection;
	private boolean isInside;
	private double insertDist;
	private Point homePosition;
	private Point currentPosition;
	private Point readyPosition;
	private DirVector homeVector;
	private DirVector currentVector;
	private DirVector readyVector;
	private int estimateTimeLength;
	private double startTime;
	private double endTime;
	private int accelerationLen;
	private double[][] hardware;
	private double[] refNeedleDepth;
	private int hardwarelength;
	private int endSignal = 0;
	private ArrayList<AdditionalRef> additionalRefDataList;
	private DecimalFormat df = new DecimalFormat("#.####");
	private double pausetime=0;
	private double ts=0.0005;

	SoftwareState() {
		targetPosition = new Point();
		targetDirection = new DirVector();
		homePosition = new Point();
		homeVector = new DirVector();
		currentPosition = new Point();
		currentVector = new DirVector();
		readyPosition = new Point();
		readyVector = new DirVector();
		additionalRefDataList = new ArrayList<AdditionalRef>();
	}

	public void set(String actionname, double speed, double[] tp, double[] td,
			boolean isinside) {
		this.actionname = actionname;
		this.speed = speed;
		this.targetPosition.set(tp[0], tp[1], tp[2]);
		this.targetDirection.set(td[0], td[1], td[2]);
		this.isInside = isinside;

	}

	public void fieldInf(Point hp, DirVector hv, Point cp, DirVector cv,
			Point rp, DirVector rv) {
		homePosition.set(hp.x, hp.y, hp.z);
		homeVector.set(hv.Rx, hv.Ry, hv.Rz);
		currentPosition.set(cp.x, cp.y, cp.z);
		currentVector.set(cv.Rx, cv.Ry, cv.Rz);
		readyPosition.set(rp.x, rp.y, rp.z);
		readyVector.set(rv.Rx, rv.Ry, rv.Rz);
	}

	public void setHardwareData(double[][] result) {
		hardwarelength = result.length;
		hardware = result;
	}

	public void addAddtionalRefInfo(double[] begin, double[] end, Object result) {
		AdditionalRef relativeMoveDir = new AdditionalRef("relativeToBeginPos",
				begin, end);
		// AdditionalRef relativeToEndPos = new
		// AdditionalRef("relativeToEndPos", begin, end);
		relativeMoveDir.getMoveCosin(result);
		// relativeToEndPos.getRelativeMoveDirOnBeginPoint(result);

		additionalRefDataList.add(relativeMoveDir);
		// additionalRefDataList.add(relativeToEndPos);
	}

	public String writeAdditionalRefInfo(int m) {

		String line = "";
		String temp = "";
		if (additionalRefDataList.isEmpty()) {
			line = "NoAdditioanlData";
			return line;
		}
		for (int i = 0; i < additionalRefDataList.size(); i++) {

			temp = df.format(additionalRefDataList.get(i).data[m][0]) + " ";
			line = line + temp;
		}
		return line;

	}

	public String writeActionParameter() {
		String actionPara = "null";
		actionPara = actionname + " " + "beginTime" + " "
				+ df.format(startTime * 100) + " " + "endTime" + " "
				+ df.format(endTime * 100) + " " + "AccelerationTimeLength"
				+ " " + df.format(accelerationLen);
		return actionPara;

	}

	public String toString(int index, int timelength) {

		String a0 = null;
		if (insertDist != 0) {
			a0 = df.format(insertDist);
		}
		int j = index / 20;
		// System.out.println(j);
		int size=(int) (timelength-pausetime/ts);		
		if (index>size)
		{actionname="pause";}
		
		String a1 = actionname + " " + df.format(speed) + " " + a0 + " "
				+ df.format(refNeedleDepth[j]);
		String a2 = "0";
		if (refNeedleDepth[j] > 0) {
			a2 = "1";
		}
		String a3 = "null" + " " + "null" + " " + "null";
		String a4 = "null" + " " + "null" + " " + "null";
		String a5 = "null" + " " + "null" + " " + "null";
		String a6 = "null" + " " + "null" + " " + "null";
		String a7 = "null" + " " + "null" + " " + "null";
		String a8 = "null" + " " + "null" + " " + "null";
		if (targetDirection != null && targetPosition != null) {
			double[] td = targetDirection.toMatrix();
			double[] tp = targetPosition.toMatrix();
			a3 = df.format(tp[0]) + " " + df.format(tp[1]) + " "
					+ df.format(tp[2]);
			a4 = df.format(td[0]) + " " + df.format(td[1]) + " "
					+ df.format(td[2]);
		}
		if (currentPosition != null && currentVector != null) {
			double[] cd = currentVector.toMatrix();
			double[] cp = currentPosition.toMatrix();
			a5 = df.format(cp[0]) + " " + df.format(cp[1]) + " "
					+ df.format(cp[2]);
			a6 = df.format(cd[0]) + " " + df.format(cd[1]) + " "
					+ df.format(cd[2]);
		}
		if (readyVector != null && readyPosition != null) {
			double[] rd = readyVector.toMatrix();
			double[] rp = readyPosition.toMatrix();
			a7 = df.format(rp[0]) + " " + df.format(rp[1]) + " "
					+ df.format(rp[2]);
			a8 = df.format(rd[0]) + " " + df.format(rd[1]) + " "
					+ df.format(rd[2]);
		}
		String a = a1 + " " + a2 + " " + a3 + " " + a4 + " " + a5 + " " + a6
				+ " " + a7 + " " + a8 + " ";
		return a;
	}

	public double[][] getHardware() {
		return hardware;
	}

	public int getHardwareLength() {
		return hardwarelength;
	}

	public int getEstimateTimeLength() {
		return estimateTimeLength;
	}
	public void setPauseTime(double pt){
		pausetime=pt;
	}
	public void setDist(double d) {
		insertDist = d;
	}

	public void setEstimateTimeLength(int t) {
		estimateTimeLength = t;
	}

	public void setEndSignal() {
		endSignal = 1;
	}

	public boolean isEnd() {
		if (endSignal == 1) {
			return true;
		} else {
			return false;
		}
	}

	public void setStartAndEnd(Object result) {
		Object[] result2 = (Object[]) result;
		double[] time = (double[]) result2[0];
		startTime = time[0];
		endTime = time[time.length - 1];
		accelerationLen = time.length / 3;
	}

	public void setRefNeedleDepth(double[] nd) {
		refNeedleDepth = nd;
	}

	public static void main(String[] args) {
		SoftwareState a = new SoftwareState();
		double[] tp = { 1, 2, 3 };
		double[] td = { 1, 2, 3 };
		boolean isinside = true;
		double speed = 1;
		String actionname = "m";
		a.set(actionname, speed, tp, td, isinside);

		MatlabProxy proxy = null;
		RobotProxy r = new RobotProxy(proxy);
		r.moveNeedle(tp, td, speed);
		System.out.println("END");
	}
}
