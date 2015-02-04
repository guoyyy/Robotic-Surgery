/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.cwru.eecs.ros.api;

/**
 *
 * @author Zhuofu
 */
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
   
public class NewQueue<SoftwareState> {  
   
    private final int SIZE = 15;  
   
    private BlockingQueue<SoftwareState> queue = new  ArrayBlockingQueue(SIZE);  
   
    private final Object consumerLock = new Object();  
    private final Object producerLock = new Object();  
   
    public void add(SoftwareState data) throws InterruptedException {  
        synchronized(producerLock) {  
            while(queue.size() == SIZE) {  
                producerLock.wait();  
            }  
            queue.add(data);  
        }  
   
        synchronized(consumerLock) {  
            consumerLock.notify();  
        }  
    }  
   
    public SoftwareState remove() throws InterruptedException {  
        SoftwareState toReturn;  
        synchronized(consumerLock) {  
            while(queue.size() == 0) {  
                consumerLock.wait();  
            }  
            toReturn = queue.take();  
        }  
   
        synchronized(producerLock) {  
            producerLock.notify();  
        }  
        return toReturn;  
    }  
    
    public static void main(String[] args){
    
        
        
       
       // swstate.set("move needle", speed, targetposition, targetdirection, isinside);
        //state.profile.add(swstate);
        //producer.ProduceSoftwareState(swstate);
    }
} 