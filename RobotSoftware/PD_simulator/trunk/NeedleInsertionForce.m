function f = NeedleInsertionForce(d,dmax,Ain,skinthickness,B)
%Given the depth of needle insertion and a parameter vector describing the
%material, this function returns the force felt by the needle.
%Note that the force is entirely along the axis of insertion.


% % % %sample parameter values
% % % %from Maurin et al., "In Vivo Study of Forces During Needle Insertions"
% % % %parameters for before the capsule puncture
% % % %[a,b,d0,F0] = [0.121,-0.098,11.45,0.2];
% % % %parameters for after the puncture
% % % %[a b d0 F0] = [-0.031,1.7,19.61,-3.39];
% % % 
% % % a = paramvec(1);
% % % b = paramvec(2);
% % % d0 = paramvec(3);
% % % F0 = paramvec(4);
% % % 
% % % f = (F0 + b) * exp(a*(d-d0)) + b;


%force due to gel
if d < dmax 
    A = Ain(:,1);
else
    A = Ain(:,2);
end


a0 = A(1);
a1 = A(2);
a2 = A(3);

f = a0 + a1*d + a2*d^2;

%force due to "skin"
if d < skinthickness 
    dskin = d;
    
    b0 = B(1);
    b1 = B(2);
    b2 = B(3);

    f = f + b0 + b1*dskin + b2*dskin^2;
elseif d > dmax - skinthickness
    dskin = skinthickness-(dmax-d);
    
    b0 = B(1);
    b1 = B(2);
    b2 = B(3);
    
    f = f + b0 + b1*dskin + b2*dskin^2;
end