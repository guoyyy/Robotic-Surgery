/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.cwru.eecs.ros.api;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

/**
 *
 * @author Zhuofu
 */
public class SoftwareStateProfile {
    Queue<SoftwareState> profile;
    SoftwareStateProfile(){
         profile=new LinkedList<SoftwareState>();
    }
}
