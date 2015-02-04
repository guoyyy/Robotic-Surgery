package edu.cwru.eecs.ros.api;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
/**
 * 
 * @author Zhuofu
 */
public class RefTrajectory {

	double[][] Rneedle;
	double[][] SABiRx;
	double[] time;
	double TotalTime;
	private boolean isempty;
	boolean dosequence;
	double[] accdecc;
	double ts;
	double[][] tempRn;
	double[][] tempSa;
	public RefTrajectory(double sampletime) {
		isempty = true;
		dosequence = false;
		ts=sampletime;
	}

	public void update(double[][] rneedle, double[][] sabirx, double[] t,
			double total) {
		// update rneedle
		int a1 = Rneedle.length;
		int a2 = rneedle.length;
		int a = a2 + a1;
		double[][] temprneedle = new double[a][3];
		double[][] tempsabirx = new double[a][3];
		double[] tempt = new double[a];
		for (int i = 0; i < a1; i++) {
			for (int j = 0; j < 3; j++) {
				temprneedle[i][j] = Rneedle[i][j];
				tempsabirx[i][j] = SABiRx[i][j];
			}
			tempt[i] = time[i];
		}

		for (int i = a1; i < a; i++) {
			for (int j = 0; j < 3; j++) {
				temprneedle[i][j] = rneedle[i - a1][j];
				tempsabirx[i][j] = sabirx[i - a1][j];
			}
			tempt[i] = t[i - a1];
		}
		Rneedle = temprneedle;
		SABiRx = tempsabirx;
		time = tempt;
		// total
		TotalTime = (double) time.length / 100.0;
	}

	public void set(double[][] rneedle, double[][] sabirx, double[] t,
			double total) {
		Rneedle = rneedle;
		SABiRx = sabirx;
		time = t;
		TotalTime = total;
		isempty = false;
	}

	public void parseandupdate(Object result) {
		Object[] result2 = (Object[]) result;
		// accdecc=(double[]) result2[3];
		double[][] rneedle = ParseResult.formatResult(result2[2], 3);
		double[][] sabirx = ParseResult.formatResult(result2[1], 3);
		tempRn=rneedle;
		tempSa=sabirx;
		double[] t = (double[]) result2[0];
		double[] total = (double[]) result2[3];
		this.update(rneedle, sabirx, t, total[0]);
	}

	public void parseandset(Object result) {
		Object[] result2 = (Object[]) result;
		// accdecc=(double[]) result2[3];
		Rneedle = ParseResult.formatResult(result2[2], 3);
		SABiRx = ParseResult.formatResult(result2[1], 3);
		tempRn=Rneedle.clone();
		tempSa=SABiRx.clone();
		time = (double[]) result2[0];
		double[] total = (double[]) result2[3];
		TotalTime = (double) time.length / 1000.0;
		isempty = false;
	}
	
	public void gettrajectory(Object result) {
		if (dosequence == false) {
			parseandset(result);
		} else if (isempty == true) {
			parseandset(result);
		} else {
			parseandupdate(result);
		}
	}

	public void turnonsequcemove() {
		dosequence = true;
	}

	public void setdefault() {
		isempty = true;
		dosequence = false;
		Rneedle = null;
		SABiRx = null;
		time = null;
		TotalTime = 0;
	}

	public double[] getAccelarationDccelarationTimePeriod() {
		return accdecc;
	}

	public static void main(String[] args) {
		double[][] rneedle = new double[5][3];
		double[][] sabirx = new double[5][3];
		double[] t = new double[5];
		double ts = 0.001;

		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 3; j++) {
				rneedle[i][j] = 1.0;
				sabirx[i][j] = 0.66;
			}
			t[i] = 2.1;
		}

		RefTrajectory r = new RefTrajectory(ts);
		r.set(rneedle, sabirx, t, ts);
		r.update(rneedle, sabirx, t, ts);

		System.out.println(rneedle.length);
	}
}
