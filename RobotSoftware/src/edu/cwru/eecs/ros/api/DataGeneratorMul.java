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
public class DataGeneratorMul {

    private RobotProxy m_robot;
    private MatlabProxy m_proxy;
    private String m_cwd;

    public DataGeneratorMul() {
        //Set options for matlab proxy factory
        MatlabProxyFactoryOptions options = new MatlabProxyFactoryOptions.Builder().setUsePreviouslyControlledSession(true).build();

        //Create a proxy, which we will use to control MATLAB
        MatlabProxyFactory proxyFactory = new MatlabProxyFactory(options);
        try {
            m_proxy = proxyFactory.getProxy();
            m_robot = new RobotProxy(m_proxy);
            m_cwd = new File("./simulatorV2/").getAbsolutePath();
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

    public double[][] genData(String filename, int index) throws MatlabInvocationException {

    	double [][]pTargets={{-9.4233, -215.5671, 338.8466},{-9, -220, 338}};
        double [][]rTargets={{0.0845, -0.1168,0.9896},{0.3, 0.0020, 0.9995}};
        double [] insertDists={10.2239,10.2239};
        //check valid pose 1       
        Point pEnd = new Point();
        pEnd.set(pTargets[0][0], pTargets[0][1],pTargets[0][2]);
        edu.cwru.eecs.ros.api.DirVector rEnd = new edu.cwru.eecs.ros.api.DirVector();
        rEnd.set(rTargets[0][0], rTargets[0][1],rTargets[0][2]);
        rEnd.normalize();
        //chek the validity of needle pose.
        pEnd.move2(insertDists[0], rEnd.toMatrix());
        if (!m_robot.checkValidPose(pEnd, rEnd)) {
            System.out.println("This needle pose is invalid at end position.");
            return null;
        }
        /**Get insertdist */
        double insertDist = m_robot.getNeedleDepth(pEnd.toMatrix(), rEnd.toMatrix());
        if (insertDist == 0) {
            System.out.println("The end point is outside the gel");
            return null;
        }
        insertDist = insertDists[0];
        // Get Ready Position & Check the validity of Ready Position
        Point pReady = new Point();
        pReady.set(pEnd.x - insertDist * rEnd.Rx, pEnd.y - insertDist * rEnd.Ry, pEnd.z - insertDist * rEnd.Rz);

        if (!m_robot.checkValidPose(pReady, rEnd)) {
            System.out.println("This needle pose of is invalid at ready position.");
            return null;
        }   
        
      //check valid pose 2       
        Point pEnd2 = new Point();
        pEnd2.set(pTargets[1][0], pTargets[1][1],pTargets[1][2]);

        edu.cwru.eecs.ros.api.DirVector rEnd2 = new edu.cwru.eecs.ros.api.DirVector();
        rEnd2.set(rTargets[1][0], rTargets[1][1],rTargets[1][2]);
        rEnd2.normalize();
        //chek the validity of needle pose.
        pEnd2.move2(insertDists[1], rEnd2.toMatrix());
        if (!m_robot.checkValidPose(pEnd2, rEnd2)) {
            System.out.println("This needle pose is invalid at end position.");
            return null;
        }
        /**Get insertdist */
        double insertDist2 = m_robot.getNeedleDepth(pEnd2.toMatrix(), rEnd2.toMatrix());
        if (insertDist2 == 0) {
            System.out.println("The end point is outside the gel");
            return null;
        }
        insertDist2 = insertDists[1];
        // Get Ready Position & Check the validity of Ready Position
        Point pReady2 = new Point();
        pReady2.set(pEnd2.x - insertDist2 * rEnd2.Rx, pEnd2.y - insertDist2 * rEnd2.Ry, pEnd2.z - insertDist2 * rEnd2.Rz);

        if (!m_robot.checkValidPose(pReady2, rEnd2)) {
            System.out.println("This needle pose of is invalid at ready position.");
            return null;
        }   
        
       double[][] 	rTargets2={{rEnd.Rx, rEnd.Ry,rEnd.Rz},{rEnd2.Rx, rEnd2.Ry,rEnd2.Rz}};
       
        
     return genData(pTargets, rTargets2, insertDists, filename, index);
    }

    private double[][] genData(double[][] pTargets, double[][] rTargets, double[] insertDists, String filename, int index) throws MatlabInvocationException {
        m_robot.nameDateFile(filename, index);
        cmdCD(m_cwd);
        double speed = 100;
        int debuglevel = 1; 
        boolean checkflag = false;//check trajectory flag.
        //double[][] result = m_robot.singleInsertion(pTarget, rTarget, insertDist, speed, debuglevel, checkflag);
        double[][] result = m_robot.MultipleInsertion(pTargets, rTargets, insertDists,speed,debuglevel, checkflag);
//        File file = new File(filename);
//        DecimalFormat df = new DecimalFormat("#.####");
//        try {
//            RandomAccessFile rf = new RandomAccessFile(filename, "rw");
//            try {
//                //System.out.println(rf.getFilePointer());
//                rf.writeBytes("Ready Point: " + df.format(pTarget[0]) + " " + df.format(pTarget[1]) + " " + df.format(pTarget[2]) + "  "
//                        + "Direction: " + df.format(rTarget[0]) + " " + df.format(rTarget[1]) + " " + df.format(rTarget[2]) + " "
//                        + "Insert-Distance: " + df.format(insertDist));
//                rf.close();// 
//
//            } catch (IOException ex) {
//                Logger.getLogger(DataGenerator.class.getName()).log(Level.SEVERE, null, ex);
//            }
//        } catch (FileNotFoundException ex) {
//            Logger.getLogger(DataGenerator.class.getName()).log(Level.SEVERE, null, ex);
//        }
        
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

    public static void main(String[] args) throws MatlabConnectionException, MatlabInvocationException {
        DataGeneratorMul dataGenerator = new DataGeneratorMul();

        dataGenerator.open();
        for (int i = 0; i < 1; i++) { //String filename=".\\outdata.txt";
            String dir = "RawTrajectory" + (i);
            String fileName = "raw_trajectory/" + dir + ".raw";
            double[][] result = dataGenerator.genData(fileName, i); //Generate data of single insertion (move needle, insert, extract needle, move)

            if (result == null) {
                System.out.println("Fail to generate trajectory");
            } else {
                System.out.println("Data:" + result.length + "*" + result[0].length);
                System.out.println("Head: " + result[0]);
            }
        }
        dataGenerator.close();

    }
}