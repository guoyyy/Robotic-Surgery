package org.RobotGUI;
import matlabcontrol.MatlabInvocationException;

public interface GUIListener {

  void handleInitialization() throws MatlabInvocationException;        
   // more method signatures 
 // void handleAngleIncrement();
  //void handleAngleDecrement();
  void cancleChangeNeedleDirection();
  void handleMoveAction();
  void changeNeedleDirection(double alpha,double beta);
}