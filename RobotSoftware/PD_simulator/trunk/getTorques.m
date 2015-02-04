function torques = getTorques(f,R,Jb,Jf)

J = [Jf;Jb];

F = f * -R;

torques = J'*F;