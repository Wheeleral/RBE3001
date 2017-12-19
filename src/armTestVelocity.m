javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

% move the arm in a triangle shape and visualize

% links = [8, 6.5, 8.125];
links = [200, 166, 200];
calibration = [-224, 1855, -1185];
pp = PacketProcessor(7);

arm = Arm(links, pp);
index = 1;
historyX = [];%store in single matix?
historyY = [];
historyZ = [];
u=[];
hisVeltip=[];
hisvel1=[];
hisvel2=[];
hisvel3=[];
%cam = webcam;

arm.setPID([.0033 .0033 .0033], [0 0 0], [0.01 0.01 0.01]);
while 1
    
%     if index >= 0 && index <= 3
%         u = arm.set([250, 500, 500]);
%     end
%     if(index > 3 && index <= 6)
%         u = arm.set([0, 500, 500]);
%     end
%     if(index > 6 && index <= 9)
%         u = arm.set([0, 250, 500]);
%     end
%     if(index >= 9)
%         index = 0;
%     end
%     for i=1:50
%         curVals = arm.set([0,0,0]);
%         setVals = [curVals(1)-calibration(1), curVals(4)-calibration(2), curVals(7)-calibration(3)];
%         u = [u; setVals];
%             
%         pause(0.1);
%     end
%     
%     csvwrite('armSetPoints.csv', u);
%     fileValues = csvread('armSetPoints.csv');
%     u=arm.set([fileValues(index,1), fileValues(index, 2), fileValues(index, 3)]);
%     
%     angles = [-90*(u(1)-calibration(1))/1000, 90*(u(4)-calibration(2))/1000, -90*(u(7)-calibration(3))/1000];


%     
%     pause(0.1);
%     if(index < 50)
%         index = index+1;
%     else
%         index = 1;
%     end
    desiredVel=[50 0 0];
vels = arm.getV();
  angles = arm.getd();
  %velocity=arm.getv();
  jointVel = arm.invVelocityKin(angles(1), angles(2), angles(3), desiredVel(1), desiredVel(2), desiredVel(3));

  veltip =sqrt(desiredVel(1)*desiredVel(1)+desiredVel(2)*desiredVel(2)+desiredVel(3)*desiredVel(3));

  %   disp(jointVel);
   % pos = arm.positionTransform(0,90,-90)
   setAng = jointVel*toc + angles;
   
 %  armDraw(arm, angles, [10 10 10]);
  arm.setd(setAng); 
     curXYZ = arm.link3Draw(angles(1), angles(2), angles(3));
    historyX = [historyX curXYZ(1)];
    historyY = [historyY curXYZ(2)];
    historyZ = [historyZ curXYZ(3)];
 %   armDrawHist(arm, angles, links, historyY, historyX, historyZ);
 
 hisVeltip=[hisVeltip veltip];
    hisvel1=[hisvel1 vels(1)];
    hisvel2=[hisvel2 vels(2)];
    hisvel3=[hisvel3 vels(3)];

     armDrawHist(arm, angles, links, historyY, historyX, historyZ);
    
 
 
%   ret =arm.set([0 500 500]);
%   disp(ret);
  pause(0.1);
  tic
end

arm.close();

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
%     camproj('perspective')
    h = rotate3d;
    h.Enable = 'off';
    
%     pause(0.1);
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
    title('3DOF Arm')
    xlabel('X Axis');
    ylabel('Y Axis');
    zlabel('Z Axis');
    axis([-links(1)*3 links(1)*3 0 links(1)*3 0 links(1)*3]);
%     camproj('perspective')
    h = rotate3d;
    h.Enable = 'off';
    
%     pause(0.1);
end