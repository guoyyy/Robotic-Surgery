/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package org.RobotGUI;

import javax.media.opengl.GL;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.glu.GLU;
import javax.media.opengl.glu.GLUquadric;

import java.awt.event.*;

/**
 *
 * @author sean
 */
public class Renderer implements GLEventListener, MouseListener, MouseMotionListener, KeyListener {
    private double[] target;
    private double[] needlePos;
    private double[] r;
    private double[] front;
    private double[] back;
    private double[] angles;
    private double deltaAngle;
    private float camX, camY, camZ;
    int xpos, ypos;
    int xnew, ynew;
    double deltaX, deltaY, deltaZ;
    float alpha, beta, radius;
   

    public Renderer() {
        target = new double[3];
        needlePos = new double[3];
        front = new double[3];
        back = new double[3];
        angles = new double[6];
        r = new double[3];
        target[0] = 0;
        target[1] = -90;
        target[2] = 400;
        r[0] = -0.0327f;//-6.54
        r[1] = 0.0020f;//.4
        r[2] = 0.9995f;//199.9
        
        /*needlePos[0] = -10.7505f;
        needlePos[1] = -206.2838f;
        needlePos[2] = 330.8692f;
        front[0] = -2.6f;
        front[1] = -45.2f;
        front[2] = 197.5f;
        back[0] = 1f;
        back[1] = 0f;
        back[2] = 0f;*/
        angles[0] = -0.9;
        angles[1] = 3.9;
        angles[2] = 4.9;
        angles[3] = 7;
        angles[4] = 2.4;
        angles[5] = 7.7;
        
        //the distance between the front and back is = 202.638f
        //therefore the middle is = 101.319
        //these parameters, set the image center at (0,0,0)
        
        front[0] = 0;
        front[1] = 110;
        front[2] = 101.319;
        back[0] = 0;
        back[1] = 110;
        back[2] = -101.319;
        needlePos[0] = -10.7505f;
        needlePos[1] = -206.2838f;
        needlePos[2] = 330.8692f;
        alpha = 0;
        radius = 650; 
        
        camX = 0;
        camY = 0;
        camZ = 650;
        

        
    }

    public void init(GLAutoDrawable drawable) {
        GL gl = drawable.getGL();
       //System.err.println("INIT GL IS: " + gl.getClass().getName());

        gl.setSwapInterval(1);

        gl.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        gl.glShadeModel(GL.GL_SMOOTH);
       
        
        
    }
    
   /* public void drawLine(GLAutoDrawable drawable, float x1, float y1, float x2, float y2) {
        GL gl = drawable.getGL();
    }*/

    public void display(GLAutoDrawable drawable) {
        GL gl = drawable.getGL();
        GLU glu = new GLU();

        // Clear the drawing area
        gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);
        // Reset the current matrix to the "identity"
        gl.glLoadIdentity();

        //glu.gluLookAt(-200, -50, -50, 0, -150, 100, 0, 1, 0);
        //glu.gluLookAt(camX, camY, camZ, 0, -150, 100, 0, 1, 0);
//        glu.gluLookAt(camX, camY, camZ, 0, 0, 0, 0, 1, 0);
        angles[2] = -angles[5];
        
        glu.gluLookAt(0, 0, 650, 0, 0, 0, 0, 1, 0);
        //alpha and beta are camera angles,
        gl.glRotatef(beta, 1.0f, 0.0f, 0.0f);
        gl.glRotatef(alpha, 0.0f, 1.0f, 0.0f);
        
        gl.glColor3f(1.0f, 1.0f, 1.0f);//white
      
        //Graphics: Line frame for robot 
        
        //front set of arms
        
        gl.glBegin(GL.GL_LINES);       
        gl.glColor3f(0, 0, 1);
        gl.glVertex3d(front[0], front[1], front[2]);
        gl.glVertex3d(front[0] + Math.cos(angles[0]) * 100, front[1] + Math.sin(angles[0]) * Math.sin(angles[2]) * -100, front[2] + Math.cos(angles[2]));
        gl.glEnd();
        
        gl.glBegin(GL.GL_LINES);
        gl.glVertex3d(front[0] + Math.cos(angles[1]) * 100, front[1] + Math.sin(angles[1]) * Math.sin(angles[2]) * -100, front[2] + Math.cos(angles[2]));
        gl.glVertex3d(front[0], front[1], front[2]);
        gl.glEnd();
        
        gl.glBegin(GL.GL_LINES);
        gl.glColor3f(1, 0, 0);
        gl.glVertex3d(front[0] + Math.cos(angles[0]) * 100, front[1] + Math.sin(angles[0]) * Math.sin(angles[2]) * -100, front[2] + Math.cos(angles[2]));
        gl.glVertex3d(front[0] + Math.cos((angles[0] + angles[1]) / 2) * 200, front[1] + Math.sin((angles[0] + angles[1]) / 2) * Math.sin(angles[2]) * 200, front[2] + Math.cos(angles[2]) * 200);
        gl.glEnd();
        
        gl.glBegin(GL.GL_LINES);
        gl.glVertex3d(front[0] + Math.cos((angles[0] + angles[1]) / 2) * 200, front[1] + Math.sin((angles[0] + angles[1]) / 2) * Math.sin(angles[2]) * 200, front[2] + Math.cos(angles[2]) * 200);
        gl.glVertex3d(front[0] + Math.cos(angles[1]) * 100, front[1] + Math.sin(angles[1]) * Math.sin(angles[2]) * -100, front[2] + Math.cos(angles[2]));
        gl.glEnd();
        
        
        //Back set of arms
        gl.glBegin(GL.GL_LINES);       
        gl.glColor3f(0, 0, 1);
        gl.glVertex3d(back[0], back[1], back[2]);
        gl.glVertex3d(back[0] + Math.cos(angles[3]) * 100, back[1] + Math.sin(angles[3]) * Math.sin(angles[5]) * -100, back[2] + Math.cos(angles[5]));
        gl.glEnd();
        
        gl.glBegin(GL.GL_LINES);
        gl.glVertex3d(back[0] + Math.cos(angles[4]) * 100, back[1] + Math.sin(angles[4]) * Math.sin(angles[5]) * -100, back[2] + Math.cos(angles[5]));
        gl.glVertex3d(back[0], back[1], back[2]);
        gl.glEnd();
        
        gl.glBegin(GL.GL_LINES);
        gl.glColor3f(1, 0, 0);
        gl.glVertex3d(back[0] + Math.cos(angles[3]) * 100, back[1] + Math.sin(angles[3]) * Math.sin(angles[5]) * -100, back[2] + Math.cos(angles[5]));
        gl.glVertex3d(back[0] + Math.cos((angles[3] + angles[4]) / 2) * 200, back[1] + Math.sin((angles[3] + angles[4]) / 2) * Math.sin(angles[5]) * 200, back[2] + Math.cos(angles[5]) * 200);
        gl.glEnd();
        
        gl.glBegin(GL.GL_LINES);
        gl.glVertex3d(back[0] + Math.cos((angles[3] + angles[4]) / 2) * 200, back[1] + Math.sin((angles[3] + angles[4]) / 2) * Math.sin(angles[5]) * 200, back[2] + Math.cos(angles[5]) * 200);
        gl.glVertex3d(back[0] + Math.cos(angles[4]) * 100, back[1] + Math.sin(angles[4]) * Math.sin(angles[5]) * -100, back[2] + Math.cos(angles[5]));
        gl.glEnd();
        

        
        //Needle of Robot -NEED TO FIND OUT LENGTH OF NEEDLE
        gl.glBegin(GL.GL_LINES);
        gl.glColor3f(0, 1, 1);
        gl.glVertex3d(front[0] + Math.cos((angles[0] + angles[1]) / 2) * 200 + r[0]*200, front[1] + Math.sin((angles[0] + angles[1]) / 2) * Math.sin(angles[2]) * 200 + r[1]*200, front[2] + Math.cos(angles[2]) * 200 + r[2]*200);
        gl.glVertex3d(front[0] + Math.cos((angles[0] + angles[1]) / 2) * 200, front[1] + Math.sin((angles[0] + angles[1]) / 2) * Math.sin(angles[2]) * 200, front[2] + Math.cos(angles[2]) * 200);
        //gl.glVertex3d(needlePos[0] - (r[0] * 200), needlePos[1] - (r[1] * 200), needlePos[2] - (r[2] * 200));
        gl.glEnd();
        
        
        //Target? currently, is a sphere 
        
        gl.glPushMatrix();
        gl.glTranslated(target[0], target[1], target[2]);
        GLUquadric earth = glu.gluNewQuadric();
        glu.gluQuadricDrawStyle(earth, GLU.GLU_FILL);
        glu.gluQuadricNormals(earth, GLU.GLU_FLAT);
        glu.gluQuadricOrientation(earth, GLU.GLU_OUTSIDE);
        final float radius = 20f;
        final int slices = 16;
        final int stacks = 16;
        glu.gluSphere(earth, radius, slices, stacks);
        glu.gluDeleteQuadric(earth);
        gl.glPopMatrix();
        
        

        //line connecting front and back ON TOP
        gl.glLineWidth(4);
        gl.glBegin(GL.GL_LINES);        
        gl.glColor3f(1, 1, 1);
        gl.glVertex3d(front[0], front[1], front[2]);
        gl.glVertex3d(back[0], back[1], back[2]);
        gl.glEnd();
        
       //LINE CONNECTING  FRONT AND BACK ON BOTTOM
        gl.glBegin(GL.GL_LINES);
        gl.glColor3f(1, 1, 1);
        //gl.glVertex3d(back[0] + Math.cos((angles[2] + angles[3]) / 2) * 200, back[1] + Math.sin((angles[2] + angles[3]) / 2) * Math.sin(angles[4]) * 200, back[2] + Math.cos(angles[4]) * 200);
        //gl.glVertex3d(front[0] + Math.cos((angles[0] + angles[1])/2) * 200, front[1] + Math.sin((angles[0] + angles[1])/2) * 200, front[2]);
        gl.glVertex3d(back[0] + Math.cos((angles[3] + angles[4]) / 2) * 200, back[1] + Math.sin((angles[3] + angles[4]) / 2) * Math.sin(angles[5]) * 200, back[2] + Math.cos(angles[5]) * 200);
        gl.glVertex3d(front[0] + Math.cos((angles[0] + angles[1]) / 2) * 200, front[1] + Math.sin((angles[0] + angles[1]) / 2) * Math.sin(angles[2]) * 200, front[2] + Math.cos(angles[2]) * 200);
        gl.glEnd();
        
        
        //FRAME
        gl.glBegin(GL.GL_LINES);
        gl.glColor3f(1, 1, 1);
        gl.glVertex3d(front[0]+110, front[1], front[2]);
        gl.glVertex3d(front[0]-110, front[1], front[2]);
        gl.glEnd();
        
        gl.glBegin(GL.GL_LINES);
        gl.glVertex3d(front[0]+110, front[1], front[2]);
        gl.glVertex3d(front[0]+110, front[1]-220, front[2]);
        gl.glEnd();
        
        gl.glBegin(GL.GL_LINES);
        gl.glVertex3d(front[0]-110, front[1], front[2]);
        gl.glVertex3d(front[0]-110, front[1]-220, front[2]);
        gl.glEnd();
        
        //FRAME - back end
        
        gl.glBegin(GL.GL_LINES);
        gl.glColor3f(1, 1, 1);
        gl.glVertex3d(back[0]+110, back[1], back[2]);
        gl.glVertex3d(back[0]-110, back[1], back[2]);
        gl.glEnd();
        
        gl.glBegin(GL.GL_LINES);
        gl.glVertex3d(back[0]+110, back[1], back[2]);
        gl.glVertex3d(back[0]+110, back[1]-220, back[2]);
        gl.glEnd();
        
        gl.glBegin(GL.GL_LINES);
        gl.glVertex3d(back[0]-110, back[1], back[2]);
        gl.glVertex3d(back[0]-110, back[1]-220, back[2]);
        gl.glEnd();
        
       
        gl.glFlush();
    }

    public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {
        GL gl = drawable.getGL();
        GLU glu = new GLU();

        if (height <= 0) { // avoid a divide by zero error!
            height = 1;
        }
        
        final double h = (double) width / (double) height;
        gl.glViewport(0, 0, width, height);
        gl.glMatrixMode(GL.GL_PROJECTION);
        gl.glLoadIdentity();
        glu.gluPerspective(45.0f, h, 0, 1000);
        gl.glMatrixMode(GL.GL_MODELVIEW);
        //glu.gluLookAt(-200, -50, -50, 0, -150, 100, 0, 1, 0);


        //glu.gluLookAt(camX, camY, camZ, 0, -150, 100, 0, 1, 0);
        glu.gluLookAt(0, 0, 0, 0, 0, 0, 0, 1, 0);
        

        gl.glLoadIdentity();
    }

    public void displayChanged(GLAutoDrawable drawable, boolean modeChanged, boolean deviceChanged) {
    }

    public void drawRobot(GLAutoDrawable drawable, Point needlePos, Vector r, Point target, double[] angles) {
        this.needlePos = needlePos.toMatrix();
        this.r = r.toMatrix();
        this.target = target.toMatrix();
        this.angles = angles;
    }

	@Override
	public void keyPressed(KeyEvent key) {
		//key commands
		int keyCode = key.getKeyCode();
	    switch( keyCode ) { 
	        case KeyEvent.VK_UP://changes beta, the camera angle to rotate around x-axis
	            // handle up 
	        	beta -= 10;
	        	if((beta) < -360) 
	        		beta += 360;
	        	System.out.println("beta = " + beta + "camY = " + camY);
	        	
	            break;
	        case KeyEvent.VK_DOWN:
	            // handle down
	        	System.out.println("beta = " + beta + "camY = " + camY);
	        	beta += 10;
	        	if((beta) > 360) 
	        		beta -= 360;
	            break;
	        case KeyEvent.VK_LEFT://changes alpha, the camera angle to rotate around y-axis
	            // handle left 
	        	alpha -= 10;
	        	camX = (float) (radius*Math.sin(alpha));
	        	camZ = (float) (radius*Math.cos(alpha));
	        	System.out.println("camX = " + alpha);
	            break;
	        case KeyEvent.VK_RIGHT:
	            // handle right
	        	alpha += 10;
	        	camX = (float) (radius*Math.sin(alpha));
	        	camZ = (float) (radius*Math.cos(alpha));
	        	System.out.println("camX = " + alpha);
	            break;
	        case KeyEvent.VK_A://various angle control: will be changed
	        	angles[0] +=.1;     
	        	System.out.println(angles[0]);
	        break;
	        case KeyEvent.VK_Z:
	        	angles[0] -=.1;    
	        	System.out.println(angles[0]); 
	        break;
	        case KeyEvent.VK_S:
	        	angles[1] +=.1;     
	        	System.out.println(angles[1]);
	        break;
	        case KeyEvent.VK_X:
	        	angles[1] -=.1;     
	        	System.out.println(angles[1]);
	        break;
/*	        case KeyEvent.VK_D:
	        	angles[2] +=.1;
	        	System.out.println(angles[2]);
	        break;
	        case KeyEvent.VK_C:
	        	angles[2] -=.1;     
	        	System.out.println(angles[2]);
	        break;*/
	        case KeyEvent.VK_F:
	        	angles[3] +=.1;     
	        	System.out.println(angles[3]);
	        break;
	        case KeyEvent.VK_V:
	        	angles[3] -=.1;     
	        	System.out.println(angles[3]);
	        break;
	        case KeyEvent.VK_G:
	        	angles[4] +=.1;
	        	System.out.println(angles[4]);
	        break;
	        case KeyEvent.VK_B:
	        	angles[4] -=.1; 
	        	System.out.println(angles[4]);
	        break;
	        case KeyEvent.VK_D:
	        	angles[5] +=.1;
	        	System.out.println(angles[5]);
	        break;
	        case KeyEvent.VK_C:
	        	angles[5] -=.1; 
	        	System.out.println(angles[5]);
	        break;
	        case KeyEvent.VK_ESCAPE:
	        	System.exit(0); 
	       	break;
	        case KeyEvent.VK_L:
	        	radius -= 10; 
	        	camX = (float) (radius*Math.sin(alpha));
	        	camZ = (float) (radius*Math.cos(alpha));
	        	System.out.println(radius);
	        break;
	        case KeyEvent.VK_P:
	        	radius +=10;
	        	camX = (float) (radius*Math.sin(alpha));
	        	camZ = (float) (radius*Math.cos(alpha));
	        	System.out.println(radius);
	        	
	       	break;
	    }
		

		
	}

	@Override
	public void keyReleased(KeyEvent key) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void keyTyped(KeyEvent key) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseDragged(MouseEvent mouse) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseMoved(MouseEvent mouse) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseClicked(MouseEvent mouse) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseEntered(MouseEvent mouse) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseExited(MouseEvent mouse) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mousePressed(MouseEvent mouse) {
		xpos = mouse.getX();
		ypos = mouse.getY();
		System.out.println("xpos = " + xpos);
		System.out.println("ypos =" + ypos);
	}

	@Override
	public void mouseReleased(MouseEvent mouse) {
		xnew = mouse.getX();
		ynew = mouse.getY();
		
		camX = camX + (xpos - xnew);
		camY = camY + (ypos - ynew);
		System.out.println("xnew = " + xnew);
		System.out.println("ynew =" + ynew);
		System.out.println("camX = " + camX);
		System.out.println("camY = " + camY);		
		
	}
	
	public double getAngles(int i) {
		return angles[i];
		}
	
	public void setAngles(int i, double value) {
		angles[i] = value;
	}
	
	
}
