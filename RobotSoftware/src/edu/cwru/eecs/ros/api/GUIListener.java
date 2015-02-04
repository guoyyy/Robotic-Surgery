package edu.cwru.eecs.ros.api;
import matlabcontrol.MatlabInvocationException;

public interface GUIListener {

  void handleInitialization() throws MatlabInvocationException;        
   // more method signatures 
  void handleMoveAction();
  void handleExtraction();
  void changeNeedleDirection(double alpha,double beta);
}