 
vals =trajectoryparams(0,5, 0, 100, 0, 50, 0, 150, 100, 0, 0)
t=vals(1,:);

subplot(3,3,1);
plot(t, (vals(2,:)));
% axis([ 0 5 0 10]);
xlabel("time(s)");
ylabel("distance(mm)");
title("X position");

subplot(3,3,2);
plot(t, ((vals(3,:))));
title("Y position");
% axis([ 0 5 0 5]);
xlabel("time(s)");
ylabel("distance(mm)");

subplot(3,3,3);
plot(t, ((vals(4,:))));
title("Z position");
% axis([ 0 5 0 5]);
xlabel("time(s)");
ylabel("distance(mm)");


subplot(3,3,4);
plot(t,((vals(5,:))));
title("X Velocity position");
% axis([ 0 5 0 5]);
xlabel("time(s)");
ylabel("velocity(mm/s)");


subplot(3,3,5);
plot(t, ((vals(6,:))));
title("Y velocity position");
% axis([ 0 5 0 5]);
xlabel("time(s)");
ylabel("velocity(mm/s)");

subplot(3,3,6);
plot(t, ((vals(7,:))));
title("Z velocity position");
%axis([ 0 5 0 5]);
xlabel("time(s)");
ylabel("velocity(mm/s)");

subplot(3,3,7);
plot(t, ((vals(8,:))));
title("X acceleration ");
%axis([ 0 5 0 5]);
xlabel("time(s)");
ylabel("acceleration(mm/s*s)");

subplot(3,3,8);
plot(t, ((vals(9,:))));
title("X acceleration ");
%axis([ 0 5 0 5]);
xlabel("time(s)");
ylabel("acceleration(mm/s*s)");

subplot(3,3,9);
plot(t, ((vals(10,:))));
title("X acceleration ");
%axis([ 0 5 0 5]);
xlabel("time(s)");
ylabel("acceleration(mm/s*s)");

function u=trajectoryparams(to,tf, xo, xf, yo, yf, zo, zf, timestep, vo, vf)

ao=0;
af=0;
%q = a0 + a1t+a2tt+a3ttt
%qdot = a1+2a2t+3a3tt
A=[1 to to*to to^3;...
   0 1 2*to 3*to^2;...
   1 tf tf*tf tf^3;...
   0 1 2*tf 3*tf*tf];
%ax=[a1; a2; a3; a4];
%ay=[a1; a2; a3; a4];


qx=[xo; vo; xf; vf]
qy = [yo; vo; yf; vf]
qz = [zo; vo; zf; vf]

ax=inv(A)*qx
ay=inv(A)*qy
az=inv(A)*qz
t=linspace(0,tf, timestep);

u=[t; 
    [ax(1)+ax(2)*t+ax(3)*t.^2+ax(4)*t.^3;
    ay(1)+ay(2)*t+ay(3)*t.^2+ay(4)*t.^3;
    az(1)+az(2)*t+az(3)*t.^2+az(4)*t.^3];
    
    [ax(2)+ax(3)*2*t+ax(4)*3*t.^2;
    ay(2)+ay(3)*2*t+ay(4)*3*t.^2;
    az(2)+az(3)*2*t+az(4)*3*t.^2];
    
    [ax(3)*2+ax(4)*6*t;
    ay(3)*2+ay(4)*6*t;
    az(3)*2+az(4)*6*t]
    ]
end
