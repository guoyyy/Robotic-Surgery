function [fgel, fskin, ftotal] = needleforcetest(d,time)
dmax = 20;
Ain = [0.01    0.005   0.02;0.0   -0.005   -0.02];
Bin = [0.02    0.0025 -0.01];

turnaroundtime = 4.7720;
skinthickness = 0.5;

%skin parameters
b0 = Bin(1);
b1 = Bin(2);
b2 = Bin(3);

if d <= 0 
    fgel=0;
    fskin=0;
    ftotal=0;
    return;
end

fskin=0;

if time > turnaroundtime
    fskin=0;
end
if time <= turnaroundtime
    fskin=0;
end

%if we haven't started retracting, and we haven't gone through the block,
%use the parameters that have the cutting force
if d < dmax && time < turnaroundtime
    
    A = Ain(1,:);
    
    % calculate force due to skin
    if d < skinthickness
        
        dskin = d;
        fskin = b0 + b1*dskin + b2*dskin^2;      
        
    elseif d > dmax - skinthickness
        
        dskin = skinthickness-(dmax-d);
        fskin = b0 + b1*dskin + b2*dskin^2;
        
    end
    
else
    %otherwise, there's only the friction force, and there can be no force
    %due to piercing the skin
    A = Ain(2,:);
    
end


a0 = A(1);
a1 = A(2);
a2 = A(3);

fgel = a0 + a1*d + a2*d^2;

ftotal = fgel + fskin;
