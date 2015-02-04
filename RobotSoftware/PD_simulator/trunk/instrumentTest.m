instrument_CalcTDir_variable=instrument_CalcTDir_T.signals.values;
[m n p]=size(instrument_SABRFK.signals.values);
instrument_SABRFK_variable=reshape(instrument_SABRFK.signals.values,n,p)';
[m n p]=size(instrument_getJointVelocities.signals.values);
instrument_getJointVelocities_variable=reshape(instrument_getJointVelocities.signals.values,n,p)';
[m n p]=size(instrument_torquesToForces.signals.values);
instrument_torquesToForces_variable=reshape(instrument_torquesToForces.signals.values,n,p)';
k=1;
instrument_Cal_set=zeros(length(actual_Needle_Tip_Position),size(instrument_CalcTDir_variable,2));
instrument_SABRFK_set=zeros(length(actual_Needle_Tip_Position),size(instrument_SABRFK_variable,2));
instrument_geJointVelocities_set=zeros(length(actual_Needle_Tip_Position),size(instrument_getJointVelocities_variable,2));
instrument_torquesToForces_set=zeros(length(actual_Needle_Tip_Position),size(instrument_torquesToForces_variable,2));
for i=1:length(instrument_CalcTDir_variable)
    if(k+19>length(instrument_CalcTDir_variable))
        m=length(actual_Needle_Tip_Position)-k+1;
        instrument_Cal_set(k:end,:)=repmat(instrument_CalcTDir_variable(i,:),m,1);
        instrument_SABRFK_set(k:end,:)=repmat(instrument_SABRFK_variable(i,:),m,1);
        instrument_geJointVelocities_set(k:end,:)=repmat(instrument_getJointVelocities_variable(i,:),m,1);
        
    else
        instrument_Cal_set(k:k+19,:)=repmat(instrument_CalcTDir_variable(i,:),20,1);
        instrument_SABRFK_set(k:k+19,:)=repmat(instrument_SABRFK_variable(i,:),20,1);
        instrument_geJointVelocities_set(k:k+19,:)=repmat(instrument_getJointVelocities_variable(i,:),20,1);
        
    end
    k=k+20;
end
if(length(instrument_torquesToForces_set)>length(instrument_torquesToForces_variable))
    instrument_torquesToForces_set(1:length(instrument_torquesToForces_variable),:)=instrument_torquesToForces_variable;
    instrument_torquesToForces_set(length(instrument_torquesToForces_variable)+1:end,:)=...
    repmat(instrument_torquesToForces_variable(end,:),length(instrument_torquesToForces_set)-length(instrument_torquesToForces_variable),1);
else
    instrument_torquesToForces_set=instrument_torquesToForces_variable(1:length(instrument_torquesToForces_set),:);
end
outdata2=[instrument_Cal_set instrument_SABRFK_set instrument_geJointVelocities_set instrument_torquesToForces_set];
t=toc;