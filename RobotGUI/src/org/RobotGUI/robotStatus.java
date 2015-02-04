/**
 * 
 */
package org.RobotGUI;

/**
 * @author Zhuofu
 *
 */
public class robotStatus {
	Point actualNeedleTipPosition;
	Vector actualR;
	double[] actualFront;
	double[] actualBack;
	Point refNeedleTipPosition;
	Vector refR;
	double[] refFront;
	double[] refBack;
	double[] pos,torq,err;
	double needleDepth1, needleDepth2, needleDepth;
	double needleForce1, needleForce2, needleForce;
	
	/**
	 * 
	 */
	public robotStatus(double[][] outdata) {
		// TODO Auto-generated constructor stub
		int resedule=outdata.length%46;
		int k=outdata.length/46;
		if (resedule==0)
		{System.out.println("out data is not 46 dimension");}
		double []currentstate=new double[46];
		for(int i=0;i<46;i++)
		{currentstate[i]=outdata[k-1][i];}
		//
		double time=currentstate[0];
		
		actualNeedleTipPosition=new Point();
		actualNeedleTipPosition.x=currentstate[1];
		actualNeedleTipPosition.y=currentstate[2];
		actualNeedleTipPosition.z=currentstate[3];
		
		actualR=new Vector();
		actualR.Rx=currentstate[4];
		actualR.Ry=currentstate[5];
		actualR.Rz=currentstate[6];
		
		actualFront=new double[3];
		actualFront[0]=currentstate[7];
		actualFront[1]=currentstate[8];
		actualFront[2]=currentstate[9];
		
		actualBack=new double[3];
		actualBack[0]=currentstate[10];
		actualBack[1]=currentstate[11];
		actualBack[2]=currentstate[12];
		
		refNeedleTipPosition=new Point();
		refNeedleTipPosition.x=currentstate[13];
		refNeedleTipPosition.y=currentstate[14];
		refNeedleTipPosition.z=currentstate[15];
		
		refR=new Vector();
		refR.Rx=currentstate[16];
		refR.Ry=currentstate[17];
		refR.Rz=currentstate[18];
		
		refFront=new double[3];
		refFront[0]=currentstate[19];
		refFront[1]=currentstate[20];
		refFront[2]=currentstate[21];
		
		refBack=new double[3];
		refBack[0]=currentstate[22];
		refBack[1]=currentstate[23];
		refBack[2]=currentstate[24];
		
		pos=new double[5];
		pos[0]=currentstate[25];
		pos[1]=currentstate[26];
		pos[2]=currentstate[27];
		pos[3]=currentstate[28];
		pos[4]=currentstate[29];
		
		torq=new double[5];
		torq[0]=currentstate[30];
		torq[1]=currentstate[31];
		torq[2]=currentstate[32];
		torq[3]=currentstate[33];
		torq[4]=currentstate[34];
		
		err=new double[5];
		err[0]=currentstate[35];
		err[1]=currentstate[36];
		err[2]=currentstate[37];
		err[3]=currentstate[38];
		err[4]=currentstate[39];
		
		needleDepth1=currentstate[40];
		needleDepth2=currentstate[41];
		needleDepth=currentstate[42];
		
		needleForce1=currentstate[43];
		needleForce2=currentstate[44];
		needleForce=currentstate[45];
	}

        public Point getNeedlePos() {
            return actualNeedleTipPosition;
        }

        public double[] getFront() {
            return actualFront;
        }

        public double[] getBack() {
            return actualBack;
        }

        public Vector getR() {
            return actualR;
        }

        public double[] getAngles() {
            return pos;
        }

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub

	}

}
