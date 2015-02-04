/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.cwru.eecs.ros.api;

/**
 *
 * @author Zhuofu
 */
public class AdditionalRef {

    String name;
    double[][] data;
    double[] actionBeginPosition = new double[3];
    double[] actionEndPosition= new double[3];
//    double[][] rneedle;
//    double[][] sabirx;
//    double[] time;
//    double[] total;

    public AdditionalRef(String dataname, double[] current, double[] target) {
        actionBeginPosition = current.clone();
        actionEndPosition = target.clone();
        name = dataname;
    }

    public void getMoveCosin(Object result) {
        Object[] result2 = (Object[]) result;
        double dirx = 0, diry = 0, dirz = 0;
        double dirxPrime = 0, diryPrime = 0, dirzPrime = 0;
        double[][] sabirx = ParseResult.formatResult(result2[1], 3);
        double[] time = (double[]) result2[0];

        data = new double[time.length][3];
        for (int i = 0; i < time.length; i++) {
            dirx = sabirx[i][0] - actionBeginPosition[0];
            diry = sabirx[i][1] - actionBeginPosition[1];
            dirz = sabirx[i][2] - actionBeginPosition[2];

            dirxPrime = actionEndPosition[0] - actionBeginPosition[0];
            diryPrime = actionEndPosition[1] - actionBeginPosition[1];
            dirzPrime = actionEndPosition[2] - actionBeginPosition[2];

            double dirabs = Math.sqrt(dirx * dirx + diry * diry + dirz * dirz);
            double dirabsPrime = Math.sqrt(dirxPrime * dirxPrime + diryPrime * diryPrime + dirzPrime * dirzPrime);
//        double sum=Math.sqrt(xdir*xdir+ydir*ydir+zdir*zdir);
//        sum
            if (dirabs * dirabsPrime == 0) {
                data[i][0] = 1;
            } else {
                data[i][0] = (dirx * dirxPrime + diry * diryPrime + dirz * dirzPrime) / (dirabs * dirabsPrime);
            }

        }

    }
//    public void getRelativeMoveDirOnEndPoint(Object result){
//        Object[] result2 = (Object[]) result;
//        double xdir=0,ydir=0,zdir=0;
//        double[][] sabirx = ParseResult.formatResult(result2[1], 3);
//        double[] time= (double[]) result2[0];
//        
//        data=new double[time.length][3];
//        for(int i=0;i<time.length;i++){
//        xdir=sabirx[i][0]-actionEnd[0];
//        ydir=sabirx[i][1]-actionEnd[1];
//        zdir=sabirx[i][2]-actionEnd[2];
//        double sum=Math.sqrt(xdir*xdir+ydir*ydir+zdir*zdir);
//        data[i][0]=xdir/sum;
//        data[i][1]=ydir/sum;
//        data[i][2]=zdir/sum;              
//        }
//        
//    }
}
