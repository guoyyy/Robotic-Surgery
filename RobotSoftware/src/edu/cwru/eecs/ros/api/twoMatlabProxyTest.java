/**
 * 
 */
package edu.cwru.eecs.ros.api;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DecimalFormat;

import matlabcontrol.MatlabConnectionException;
import matlabcontrol.MatlabInvocationException;
import matlabcontrol.MatlabProxy;
import matlabcontrol.MatlabProxyFactory;
import matlabcontrol.MatlabProxyFactoryOptions;

/**
 * @author Zhuofu
 *
 */
public class twoMatlabProxyTest {
	private MatlabProxy m_proxy1;
    //private MatlabProxy m_proxy2;
	/**
	 * 
	 */
	public twoMatlabProxyTest() {
		{
	        //Set options for matlab proxy factory
	        MatlabProxyFactoryOptions options = new MatlabProxyFactoryOptions.Builder().setUsePreviouslyControlledSession(true).setHidden(true).build();
	        //MatlabProxyFactoryOptions options2 = new MatlabProxyFactoryOptions.Builder().setUsePreviouslyControlledSession(true).build();
	      
	        //Create a proxy, which we will use to control MATLAB
	        MatlabProxyFactory proxyFactory = new MatlabProxyFactory(options);
	       // MatlabProxyFactory proxyFactory2 = new MatlabProxyFactory(options2);
	        try {
	            m_proxy1 = proxyFactory.getProxy();
	            //m_proxy2 = proxyFactory2.getProxy();

	        } catch (MatlabConnectionException e) {
	            System.err.println("Create Matlab Proxy failed!");
	            e.printStackTrace();
	        } 
	    }// TODO Auto-generated constructor stub
	}
	public void display(){
		try {
			m_proxy1.eval("setupConstants");
			//m_proxy2.eval("disp('hello world2')");
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	/**
	 * @param args
	 * @throws IOException 
	 */
	public static void main(String[] args) throws IOException {

	}

}
