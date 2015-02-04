/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.cwru.eecs.ros.api;

import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Zhuofu
 */
public class NewProducer implements Runnable{

    private final NewQueue<SoftwareState> queue;
    SoftwareState sw;
    int busy = 0;
     private final Object setStateLock2 = new Object(); 
     private final Object getStateLock=new Object();
     private boolean isend=false;
     Thread producer;
    NewProducer(NewQueue q) {
        queue = q;
    }
    public void start(){
        Thread producer=new Thread(this);
        producer.start();
    }
    public void stop(){
        producer=null;
    }
    public void run() {     
        while (true){
       SoftwareState toReturn;  
       synchronized(getStateLock) {  
            while(busy == 0) {  
                try {
                    getStateLock.wait();
                } catch (InterruptedException ex) {
                    Logger.getLogger(NewProducer.class.getName()).log(Level.SEVERE, null, ex);
                }
            }  
            try {
                queue.add(sw);
                busy=0;
                if (sw.isEnd()==true)
                isend=true;
            } catch (InterruptedException ex) {
                Logger.getLogger(NewProducer.class.getName()).log(Level.SEVERE, null, ex);
            }
        }  
   
       if(isend==true)
        {   System.out.println("producer end");
            break;}
       
        synchronized(setStateLock2) {  
            setStateLock2.notify();  
        } 
        
    }
    }

    public void ProduceSoftwareState(SoftwareState s) throws InterruptedException {
           synchronized(setStateLock2) {  
            while(busy == 1) {  
                setStateLock2.wait();  
            }  
            //sw=new SoftwareState();
            sw=s;
            busy=1;
        }  
   
        synchronized(getStateLock) {  
            getStateLock.notify();  
        }  
//        synchronized(monitor){
//        while (true) {
//            if (busy == 0) {
//                busy = 1;
//                sw = s;
//                monitor.notify();
//                busy=0;
//                break;
//            } else {
//                try {
//                    Thread.currentThread().sleep(1000);
//                } catch (InterruptedException ex) {
//                    Logger.getLogger(NewProducer.class.getName()).log(Level.SEVERE, null, ex);
//                    System.out.println("error at ProduceSoftwareState");
//                }
//            }
//        }}
    }
}
