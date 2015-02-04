function [Vb_0N gTarg_0N] = fncNeedleStepwiseMotion(g_0N,needleState,n_r,p0,p1,p2,p3,d)
%fncNeedleStepwiseMotion Generates a motion plan based on the state number
%   Generates the Needle velocity based on the motion of the needle and the
%   state of the needle. Each state corresponds to a needle type.
%   State 1:  prepare needle for entrance.
%   State 2:  Perform needle Bite.
%   State 3:  Reorient needle for rest of suture
%   State 4:  Complete suture.

%IO list
%--------------------------------------------------------------------------
%inputs:
%        g_0N: Needle Transform.
%        needleState: state of the needle Drive.
%        p0: depends on needleState.
%        p1: depends on needleState.
%        p2: depends on needleState.
%        p3: depends on needleState.
%        d : depends on needleState.

Vb_0N = zeros(6,1);
gTarg_0N = eye(4);

switch needleState
    
    
    case 1
        %Use given entry point and direction to move the needle appropriately.
        %p0 is the entry point (in frame 0)
        %p1 is the normal vector. (in frame 0)
        %p2 is the vector of the gash
        %p3 is the location of the gash
        %d  is the distance from the tissue that the needle will start the
        %approach
        
        %Approximate the tissue as a plan that is locally normal to p0 along p1.
        %reorient the Needle so that it is also pointing correctly.
        
        %The math here is in lab notebook:
        RTarg =zeros(3,3);
        
        RTarg(:,1) = -p1/norm(p1);
        RTarg(:,3) =  p2/norm(p2);
        
        RTarg(:,2) = -cross(RTarg(:,1),RTarg(:,3));
        
        PTarg = p0+p1*d/norm(p1)-2*RTarg*[0;n_r;0];
        
        
        gTarg_0N = [RTarg PTarg; zeros(1,3) 1];
        
        
        POld = g_0N(1:3,4);
        
        ROld = g_0N(1:3,1:3);
        
        
        QTarg = R2Q(RTarg);
        QOld  = R2Q(ROld);
        
        Qrot = QaXQb(QuatInv(QOld),QTarg);
        
        
        
        
        Vs_0N = [PTarg-POld; Qrot(1)*Qrot(2:4)'];
        
        
        Vb_0N = Adjoint(Ginv(g_0N))*Vs_0N;
        
    case 2
        %Begin Approach to and bite of Tissue;
        %Use given entry point and direction to move the needle appropriately.
        %p0 is unused
        %p1 is unused
        %p2 is unused
        %p3 is unused
        %d  is unused
        
        %Approximate the tissue as a plan that is locally normal to p0 along p1.
        %reorient the Needle so that it is also pointing correctly.
        Vb_0N = [1;0;0;0;0;0];
        gTarg_0N = eye(4);
        
        
        
        
    case 3
        %After a successful Tissue_Bite,
        %move the needle such that the
        %Use given entry point and direction to move the needle appropriately.
        %p0 is Entry  point
        %p1 is tissue normal point
        %p2 is the Gash point.
        %p3 is the exit point
        %d  is unused
        alpha_0 = p0;
        beta_0 = p1;
        gamma_0 = p2;
        delta_0 = p3;
        c_n = [0;n_r;0];
        
        
        
        %Simple Version:
        %Magic math goes here!!!
        %Vb_0N = [-2*n_r;0;0;0;0;-1]*g;
        
        
        %p1 = [0;1;0];
        %p2 = [-5;5;0];
        
        pl1 = alpha_0'*beta_0;
        
        c_0 = g_0N(1:3,1:3)*c_n+g_0N(1:3,4);
        
        pl2 = c_0'*beta_0;
        
        dp = pl2-pl1;
        
        gammaS_0 = c_0-dp*beta_0;
        
        yn_0 = g_0N(1:3,1:3)*[0;1;0];

        ypn_0 = yn_0- (yn_0'*beta_0)*beta_0;
        
        tn_0 = alpha_0-gammaS_0;
        
        tpn_0 = tn_0- (tn_0'*beta_0)*beta_0;
        
        tpn_0=tpn_0/norm(tpn_0);
        
        %This is problematic because the term ypn_0 can't be negative. 
        alphaS_0 = gammaS_0+sqrt(n_r^2-dp^2)*tpn_0;
        
        zn_0 = g_0N(1:3,1:3)*[0;0;1];
        
        deltaS_0 = delta_0 - (zn_0'*delta_0)*(zn_0);
        
        pm_0 = (deltaS_0+alphaS_0)/2;
        
        v1a_0 = (deltaS_0-alphaS_0);
       
        
        d1 = norm(v1a_0)/2;
        
        v1_0 = v1a_0/norm(v1a_0);
        
        nfc_0 = beta_0(1:3)*sqrt(n_r^2-(d1)^2)+pm_0(1:3);
        
        
        v0_0 = c_0(1:3)-alphaS_0(1:3);
        vf_0 = nfc_0(1:3)-alphaS_0(1:3);
        
        alphaS_n = Ginv(g_0N)*[alphaS_0;1];
        
        cosA = (v0_0'*vf_0)/(norm(vf_0)*norm(v0_0));
        
        
        angle = acos(cosA)/5;
        
        Vb_0N(4:6)  = [0;0;-angle];
        Vb_0N(1:3)  = -VectHat(Vb_0N(4:6))*alphaS_n(1:3);
        
        %define a small forward momentum
        
        Vb_0N_fm = [-n_r;0;0;0;0;-1]*d;
        
        Vb_0N = Vb_0N+Vb_0N_fm;
        
        Vs_0N = Adjoint(g_0N)*Vb_0N;
        %Find a circle of radius r_n such that the

    case 4
        
        %This is easy
        %p0 is Entry  point
        %p1 is tissue normal point
        %p2 is the exit point.
        %p3 is unused
        %d  is the gain
        
        Vb_0N = [-n_r;0;0;0;0;-1]*d;
        gTarg_0N = eye(4);
        
        
        
    otherwise
        
        
        
        
end







end

