

sysStr = computer;

if(~strcmp(sysStr,'PCWIN64')) 

    path(path,'~/robosurgery-2/Matlab_Needle_Tissue_Models/Needle_Plotting');
    path(path,'~/robosurgery-2/Matlab_Robot_Models/Common');
    path(path,'~/robosurgery-2/Matlab_Robot_Models/Wrist_3DOF');
    path(path,'~/robosurgery-2/Matlab_Robot_Models/ABB_IRB140');
    path(path,'~/robosurgery-2/Matlab_Needle_Tissue_Models/Needle_Geometry');
    path(path,'~/robosurgery-2/Matlab_Needle_Tissue_Models/Needle_Forces');

else
   %Not sure what to do here. 
    
end