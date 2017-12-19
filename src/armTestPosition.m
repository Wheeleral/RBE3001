javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

% move the arm in a triangle shape and visualize

% links = [8, 6.5, 8.125];
links = [202, 167, 202]; %measured....
calibration = [-224, 1855, -1185];
pp = PacketProcessor(7);

arm = Arm(links, pp);
index = 1;
historyX = [];
historyY = [];
historyZ = [];
u=[];
hisx=[];
hisy=[];
hisz=[];
%cam = webcam;
vertex= [100 100 50;
         200 0 200;
         50 -100 100];

index=1;
spacing=20;
arm.setPID([.0033 .0033 .0033], [0 0 0], [0.01 0.01 0.01]);


arm.setPositionInverse(vertex(1,1),vertex(1,2),vertex(1,3));

while 1
    %set the position
    %arm.setPositionInverse(0, -300, 200); %work pos1
    %arm.setPositionInverse(200, 0, 200); %work
    %arm.setPositionInverse( 100, 100, 0); %work pos2
    for i = 1:3
        %arm.setPositionInverse(vertex(i,1),vertex(i,2),vertex(i,3));        
        %while(not(arm.atSetpoint(vertex(i,1),vertex(i,2),vertex(i,3))))
            t=linspace(0,1, spacing);
            points = transpose([(1-t)*vertex(i,1)+t*vertex(mod(i, 3)+1, 1);...
                (1-t)*vertex(i,2)+t*vertex(mod(i, 3)+1, 2);...
                (1-t)*vertex(i,3)+t*vertex(mod(i, 3)+1, 3)]);

            for j = 1:spacing
                arm.setPositionInverse(points(j,1),points(j,2),points(j,3));
                isNot = 1;
                while(isNot)
                     angles=arm.getd();
%                     currXYZ =arm.link3Draw(angles(1), angles(2), angles(3));
%                     disp(currXYZ);
%                     disp(points(j,:));
                 angles = arm.getd();
                 curXYZ = arm.link3Draw(angles(1), angles(2), angles(3));
                 historyX = [historyX curXYZ(1)];
                 historyY = [historyY curXYZ(2)];
                 historyZ = [historyZ curXYZ(3)];
                    pause(0.05);
                    isNot = not(arm.atSetpoint(points(j,1),points(j,2),points(j,3)));
                end
%                      angles = arm.getd();
        %      curXYZ = arm.link3Draw(angles(1), angles(2), angles(3));
        %      historyX = [historyX curXYZ(1)];
        %      historyY = [historyY curXYZ(2)];
        %      historyZ = [historyZ curXYZ(3)];
                pause(.05);
            end
%      angles = arm.getd();
    %      curXYZ = arm.link3Draw(angles(1), angles(2), angles(3));
    %      historyX = [historyX curXYZ(1)];
    %      historyY = [historyY curXYZ(2)];
    %      historyZ = [historyZ curXYZ(3)];
            pause(1);
    end
    
%     %data collection
%      angles = arm.getd();
%      curXYZ = arm.link3Draw(angles(1), angles(2), angles(3));
%      historyX = [historyX curXYZ(1)];
%      historyY = [historyY curXYZ(2)];
%      historyZ = [historyZ curXYZ(3)];
%      plotDataPos(historyX,historyY,historyZ); 

    %armDrawHist(arm, angles, links, historyY, historyX, historyZ) 
    pause(0.1);
    tic
end

arm.close();

function plotData(historyX, historyY, historyZ, hisx, hisy, hisz)

    subplot(2, 1,1);
    plot(historyX);
    hold on
    plot(historyY);
    plot(historyZ);
    
    title("XYZ Position");
    legend("X", "Y", "Z");
    xlabel('Time');
    ylabel('Position(mm)');
    hold off
    
    subplot(2,1,2);
    plot(hisx);
    hold on
    plot(hisy);
    plot(hisz);
    
    legend("Angle 1", "Angle 2", "Angle 3");
    title("Angles over Time");
    xlabel('Time');
    ylabel('Degrees');
    hold off
end
function plotDataPos(historyX, historyY, historyZ)

    plot(historyX);
    hold on
    plot(historyY);
    plot(historyZ);
    
    title("XYZ Position");
    legend("X", "Y", "Z");
    xlabel('Time');
    ylabel('Position(mm)');
    hold off
    

end

function armDraw(arm, angles, links)
    angles(1) = angles(1)+90;
    p2 = arm.link2(angles(1), angles(2));
    p3 = arm.link3(angles(1), angles(2), angles(3));

    plot3([0 0 p2(1) p3(1)],[0 0 p2(2) p3(2)],[0 links(1) p2(3) p3(3)],'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);grid on;%axis([-31,31,-31,31,0,31]);

    text(p3(1),p3(2),p3(3),['  (', num2str(p3(1),3), ', ', num2str(p3(2),3),', ', num2str(p3(3),3), ')']);
    title('3DOF Arm')
    xlabel('X Axis');
    ylabel('Y Axis');
    zlabel('Z Axis');
    axis([-links(1)*3 links(1)*3 0 links(1)*3 0 links(1)*3]);
    h = rotate3d;
    h.Enable = 'off';
    
end

function armDrawHist(arm, angles, links, historyX, historyY, historyZ) %draw the arm in 3D space
%    angles(1) = angles(1)+90;
    p2 = arm.link2(angles(1), angles(2)); %
    p3 = arm.link3Draw(angles(1), angles(2), angles(3));
    hold off;
    plot3([0 0 p2(2) p3(2)],[0 0 p2(1) p3(1)],[0 links(1) p2(3) p3(3)],'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);grid on;%axis([-31,31,-31,31,0,31]);
    hold on;
    plot3(historyX, historyY, historyZ,'-','LineWidth',1);
    
    text(p3(1),p3(2),p3(3),['  (', num2str(p3(1),3), ', ', num2str(p3(2),3),', ', num2str(p3(3),3), ')']);
    title('3DOF Arm with History')
    xlabel('X Axis');
    ylabel('Y Axis');
    zlabel('Z Axis');
    axis([-links(1)*3 links(1)*3 0 links(1)*3 0 links(1)*3]);
    h = rotate3d;
    h.Enable = 'off';
    
end

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