%Calculate loop gains from kinematics
theta=[0.35 0.35 -0.35 0.35 0.35];
theta=[0 0 0 0 0];
[NeedleTip,NeedleRot,Front,Back,Jf,Jb]=JTest(theta(1), theta(2), theta(3), theta(4), theta(5));

for (ix=1:5),
    Wrench=[0 0 0 0 0 0]';
    Wrench(ix)=1;
    T=gT_Test(Wrench,Front,Back,Jf,Jb);
    V=gV_Test(T, Front, Back, Jf, Jb);
    JTmatrix(:,ix)=T;
    GainMatrix(:,ix)=V(1:5,1);
end

GainMatrix

xts=0.001;
BW=50;
CGains=(1-exp(-2*pi*BW*xts))./(xts.*diag(GainMatrix));
PGains=CGains(1:3)
AGains=CGains(4:5)

BWa=-log(1-([Kpos;Kdir].*xts.*diag(GainMatrix)))/xts/2/pi


for (ix=1:5),
    EefVel=[0 0 0 0 0 0]';
    EefVel(ix)=1;
    jV=gjV_Test(EefVel,Front,Back,Jf,Jb);
    V=gV_Test(jV, Front, Back, Jf, Jb);
    Jinvmatrix(:,ix)=jV;
    GainMatrix2(:,ix)=V(1:5,1);
end

GainMatrix2

CGains2=(1-exp(-2*pi*BW*xts))./(xts.*diag(GainMatrix2));
PGains2=CGains2(1:3)
AGains2=CGains2(4:5)

BWa2=-log(1-([Kpos;Kdir].*xts.*diag(GainMatrix2)))/xts/2/pi

for (ix=1:5),
    jV=[0 0 0 0 0 0]';
    jV(ix)=1;
    V=gV_Test(jV, Front, Back, Jf, Jb);
    Jmatrix(:,ix)=V;
end
