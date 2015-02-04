/**
 * 
 */
package edu.cwru.eecs.ros.api;

/**
 * @author Zhuofu
 *
 */
public class RobotStatus {
        double time;
	Point actualNeedleTipPosition;
	DirVector actualR;
	double[] actualFront;
	double[] actualBack;
	Point refNeedleTipPosition;
	DirVector refR;
	double[] refFront;
	double[] refBack;
	double[] pos,torq,err;
	double needleDepth1, needleDepth2, needleDepth;
	double needleForce1, needleForce2, needleForce;
	
	/**
	 * 
	 */
	public RobotStatus() {
            actualNeedleTipPosition=new Point();
            actualR=new DirVector();
            actualFront=new double[3];
            actualBack=new double[3];
            refNeedleTipPosition=new Point();
            refR=new DirVector();
            refFront=new double[3];
            refBack=new double[3];
            pos=new double[5];
            torq=new double[5];
            err=new double[5];
        }
		// TODO Auto-generated constructor stub
		//int resedule=outdata.length%46;
		//int k=outdata.length/46;
		//if (resedule==0)
		//{System.out.println("out data is not 46 dimension");}
        public void update(double[][] outdata){
		double []currentstate=new double[46];
		
                for(int i=0;i<46;i++)
		{currentstate[i]=outdata[outdata.length-1][i];}
		//
		time=currentstate[0];
		
		
		actualNeedleTipPosition.x=currentstate[1];
		actualNeedleTipPosition.y=currentstate[2];
		actualNeedleTipPosition.z=currentstate[3];
		
		
		actualR.Rx=currentstate[4];
		actualR.Ry=currentstate[5];
		actualR.Rz=currentstate[6];
		
		
		actualFront[0]=currentstate[7];
		actualFront[1]=currentstate[8];
		actualFront[2]=currentstate[9];
		
		
		actualBack[0]=currentstate[10];
		actualBack[1]=currentstate[11];
		actualBack[2]=currentstate[12];
		
		
		refNeedleTipPosition.x=currentstate[13];
		refNeedleTipPosition.y=currentstate[14];
		refNeedleTipPosition.z=currentstate[15];
		
		
		refR.Rx=currentstate[16];
		refR.Ry=currentstate[17];
		refR.Rz=currentstate[18];
		
		
		refFront[0]=currentstate[19];
		refFront[1]=currentstate[20];
		refFront[2]=currentstate[21];
		
		
		refBack[0]=currentstate[22];
		refBack[1]=currentstate[23];
		refBack[2]=currentstate[24];
		
		
		pos[0]=currentstate[25];
		pos[1]=currentstate[26];
		pos[2]=currentstate[27];
		pos[1]=currentstate[28];
		pos[2]=currentstate[29];
		
		
		torq[0]=currentstate[30];
		torq[1]=currentstate[31];
		torq[2]=currentstate[32];
		torq[1]=currentstate[33];
		torq[2]=currentstate[34];
		
		
		err[0]=currentstate[35];
		err[1]=currentstate[36];
		err[2]=currentstate[37];
		err[1]=currentstate[38];
		err[2]=currentstate[39];
		
		needleDepth1=currentstate[40];
		needleDepth2=currentstate[41];
		needleDepth=currentstate[42];
		
		needleForce1=currentstate[43];
		needleForce2=currentstate[44];
		needleForce=currentstate[45];
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub

	}

}
