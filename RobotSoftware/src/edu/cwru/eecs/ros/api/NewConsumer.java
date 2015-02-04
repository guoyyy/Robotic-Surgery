/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.cwru.eecs.ros.api;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * 
 * @author Zhuofu
 */
public class NewConsumer implements Runnable {

	private final NewQueue<SoftwareState> queue;
	//int matlabReturenDataDimenstion;
	SoftwareState sw;
	String filename;
	BufferedWriter fwriter;
	BufferedWriter fwriter2;
	int busy = 1;
	private final BlockingQueue<SoftwareState> tempqueue = new ArrayBlockingQueue(
			10);
	private final Object setDirLock2 = new Object();
	Thread consumer;

	NewConsumer(NewQueue q) {
		queue = q;
	//matlabReturenDataDimenstion=dataDimension;
	}

	public void start() {
		Thread consumer = new Thread(this);
		consumer.start();
	}

	public void stop() {
		consumer = null;
	}

	public void run() {
		try {
			while (true) {
				sw = queue.remove();
				if (sw.isEnd() == true) {
					System.out.println("Consumer end");
					break;
				}
				if (sw.getHardware() != null) {
					writeToFile(tempqueue, sw.getHardware(),
							sw.getHardwareLength(), fwriter, fwriter2);
				} else {
					tempqueue.add(sw);
					System.out.println("add one");
				}
			}
		} catch (Exception ex) {
		}
	}

	private void writeToFile(BlockingQueue<SoftwareState> q,
			double[][] hardware, int hardwireLength, BufferedWriter fw1,
			BufferedWriter fw2) throws InterruptedException {
		if (hardware.length > 1) {
			String output_sw = "";
			DecimalFormat df = new DecimalFormat("0.#####E0");
			int sum = 0;
			try {
				fw1.write("-----------------------------------------------------------------------------------"
						+ "--------------------------------------------------------------------------------Reserved Line\n");
			} catch (IOException ex) {
				Logger.getLogger(NewConsumer.class.getName()).log(Level.SEVERE,
						null, ex);
			}
			// String moveparas="";
			while (q.isEmpty() != true) {
				sw = q.take();

				int timelength = sw.getEstimateTimeLength();
				String moveinfo = sw.writeActionParameter();
				// moveparas=moveparas+" "+moveinfo;
				// try {
				// fw.write(moveinfo);
				// fw.write("\n");
				// fw.flush();
				// } catch (IOException e) {
				// // TODO Auto-generated catch block
				// e.printStackTrace();
				// }
				try {
					fw2.write(moveinfo);
					fw2.write("\n");
					fw2.flush();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				for (int i = sum; i < timelength + sum; i++) {
					try {
						if (i > hardwireLength) {
							break;
						}
						output_sw = sw.toString(i - sum, timelength);
						fw1.write(output_sw);
						// fw.write(sw.writeAdditionalRefInfo(i-sum));//i out of
						// bounds
						for (int j = 0; j < hardware[0].length; j++) {

							fw1.write(df.format(hardware[i][j]) + " ");
						}
						fw1.write("\n");
						fw1.flush();
					} catch (IOException ex) {
						System.out
								.println("Problem writing results out to line: "
										+ i + " :: " + ex);
					}
				}
				sum = sum + timelength;
			}

		} else {
			while (q.isEmpty() != true) {
				sw = q.take();
			}
			System.out.println("Write a blank file");
			try {
				fw1.write("-----------------------------------------------------------------------------------"
						+ "--------------------------------------------------------------------------------Reserved Line\n");
				fw1.write("Fail-Trajectory\n");
				fw1.flush();
			} catch (IOException ex) {
				Logger.getLogger(NewConsumer.class.getName()).log(Level.SEVERE,
						null, ex);
			}
		}

		synchronized (setDirLock2) {
			busy = 0;
			System.out.println("write one");
			setDirLock2.notify();
		}

	}

	public void setDir(String dir) {
		if (filename == null) {
			filename = dir;
			try {
				fwriter = new BufferedWriter(new FileWriter(filename));

				int t = filename.lastIndexOf(".");
				String filename2 = filename.substring(0, t);
				filename2 = filename2 + ".info";
				fwriter2 = new BufferedWriter(new FileWriter(filename2));
			} catch (IOException ex) {

				System.out.println("no file found at " + filename);
			}
		} else {
			synchronized (setDirLock2) {
				while (busy == 1) {
					try {
						setDirLock2.wait();
					} catch (InterruptedException ex) {
						Logger.getLogger(NewConsumer.class.getName()).log(
								Level.SEVERE, null, ex);
					}
				}
				filename = dir;
				try {

					fwriter = new BufferedWriter(new FileWriter(filename));
					int t = filename.lastIndexOf(".");
					String filename2 = filename.substring(0, t);
					filename2 = filename2 + ".info";
					fwriter2 = new BufferedWriter(new FileWriter(filename2));
				} catch (IOException ex) {

					System.out.println("no file found at " + filename);
				}
				busy = 1;
			}
		}
	}

	public static void main(String[] args) {
		String dir = ".\\outdata.txt";
		NewQueue<SoftwareState> queue = new NewQueue();
		NewConsumer c = new NewConsumer(queue);
		c.setDir(dir);
		c.start();

		double[] tp = { -10, -200, 335 };
		double[] td = { 0, 0, 1 };
		boolean isinside = true;
		double speed = 1;
		String actionname = "m";
		SoftwareState sw = new SoftwareState();
		sw.set(actionname, speed, tp, td, isinside);

		SoftwareState sw2 = new SoftwareState();
		sw2.set(actionname, speed, tp, td, isinside);
		try {
			queue.add(sw);
		} catch (InterruptedException ex) {
			// Logger.getLogger(StateConsumer.class.getName()).log(Level.SEVERE,
			// null, ex);
		}
		try {
			queue.add(sw2);
		} catch (InterruptedException ex) {
			// Logger.getLogger(StateConsumer.class.getName()).log(Level.SEVERE,
			// null, ex);
		}

	}
}
