javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;

links = [202, 167, 202]; %measured....
calibration = [-224, 1855, -1185];
pp = PacketProcessor(7);

arm = Arm(links, pp);
values= zeros(15, 1, 'single');
 a = 1;
 r = 1;
 z =1;

%Create an array of 32 bit floaing point zeros to load an pass to the
%packet processor

returnValues= zeros(15, 1, 'single');
sinWaveInc = 1.0;
range = 5.0;

vertex =[100, 0, 0; %position for arm for 5 setpoints
 100, 75, 198; 
120, 53, 131;
100,75 ,131;
150, 154, 188;
200, 200, 100; %position for arm for 5 setpoints
 150, 175, 100; 
 200, 053, 231;
120, 175 ,131;
 110, 154, 188];
curr=pos1;

PID=zeros(15, 1, 'single'); %set inital pid matrix

p = [0.01, 0.0045, 0.0045]; %.Good pid numbers
i = [0.0, 0.000, 0.00];
d = [1.0, 0.001, 0.001];
for j=0:2
    PID((j * 3) + 1) = p(j+1);
    PID((j * 3) + 2) = i(j+1);
    PID((j * 3) + 3) = d(j+1);
end
   historyX = [];
historyY = [];
historyZ = [];
hisx=[];
hisy=[];
hisz=[];
hism=[];
port=55;
arm.setPID([.0033 .0033 .0033], [0 0 0], [0.01 0.01 0.01]);


       port = 42;
       returnValues = pp.command(port,values)
       
       angles = arm.getd();
spacing=50;

       arm.closeGripper();
       pause(1);
       %arm.movepoint(arm.links(2)+arm.links(3) , 0, arm.links(1),50);
       while 1
           for i = 1:9
               t=linspace(0,1, spacing);
               points = transpose([(1-t)*vertex(i,1)+t*vertex(mod(i, 3)+1, 1);...
                   (1-t)*vertex(i,2)+t*vertex(mod(i, 3)+1, 2);...
                   (1-t)*vertex(i,3)+t*vertex(mod(i, 3)+1, 3)]);
               disp(vertex(i,1));
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
                       
                       
                       
                       force = arm.calcForce(angles);
                       hisx=[hisx force(1)];
                       hisy=[hisy force(2)];
                       hisz=[hisz force(3)];
                       hism=[hism sqrt(force(1)^2 +force(2)^2 +force(3)^2)];
                       pause(0.05);
                       isNot = not(arm.atSetpoint(points(j,1),points(j,2),points(j,3)));
                   end
                                          angles = arm.getd();
                       curXYZ = arm.link3Draw(angles(1), angles(2), angles(3));
                       historyX = [historyX curXYZ(1)];
                       historyY = [historyY curXYZ(2)];
                       historyZ = [historyZ curXYZ(3)];
                       
                       
                       force = arm.calcForce(angles);
                       hisx=[hisx force(1)];
                       hisy=[hisy force(2)];
                       hisz=[hisz force(3)];
                       hism=[hism sqrt(force(1)^2 +force(2)^2 +force(3)^2)];
               end
               port = 42;
               returnValues = pp.command(port,values)
               
               angles = arm.getd();
               
               curXYZ = arm.link3Draw(angles(1), angles(2), angles(3));
               historyX = [historyX curXYZ(1)];
               historyY = [historyY curXYZ(2)];
               historyZ = [historyZ curXYZ(3)];
               
               
               disp(returnValues(3));
               disp(returnValues(6));
               disp(returnValues(9));
               force = arm.calcForce(angles)
               hisx=[hisx force(1)];
               hisy=[hisy force(2)];
               hisz=[hisz force(3)];
               hism=[hism sqrt(force(1)^2 +force(2)^2 +force(3)^2)];
               
               % plot the forces in 3D
               
               pause(5)           
                plotData(historyX, historyY, historyZ, hisx, hisy, hisz, hism)
           end
           
        plotData(historyX, historyY, historyZ, hisx, hisy, hisz, hism)
           
       end
% pp.shutdown();
%     clear java;



function armDrawForce(arm, angles, links, force)
%    angles(1) = angles(1)+90;
    p2 = arm.link2(angles(1), angles(2)); %
    p3 = arm.link3Draw(angles(1), angles(2), angles(3));

%     xf= [[p3(1)+10*force(1); p3(3); p3(2)] [p3(1); p3(3); p3(2)]]
%     yf= [[p3(1); p3(3)+10*force(2); p3(2)] [p3(1); p3(3); p3(2)]]
%     zf= [[p3(1); p3(3); p3(3)+10*force(2)] [p3(1); p3(3); p3(2)]]
    hold off;
    plot3([0 0 p2(2) p3(2)],[0 0 p2(1) p3(1)],[0 links(1) p2(3) p3(3)],'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);grid on;%axis([-31,31,-31,31,0,31]);
    hold on;
    %force
%     plot3(yf,xf, zf,'X-','LineWidth',3,'MarkerSize',4,'MarkerFaceColor',[0,0.5,0.5]);
    %plot3([p3(2) p3(2)], [p3(1)  p3(1)],[p3(3) p3(3)],'X-','LineWidth',3,'MarkerSize',4);
    plot3( [p3(2) p3(2)+100*force(1)],[p3(1) p3(1)],[p3(3) p3(3)],'LineWidth',1.5);
    plot3( [p3(2) p3(2)],[p3(1)+100*force(2) p3(1)],[p3(3) p3(3)] ,'LineWidth',1.5);
    plot3( [p3(2) p3(2)],[p3(1) p3(1)],[ p3(3) p3(3)-1000*force(3)] ,'LineWidth',1.5);

    
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


function plotData(historyX, historyY, historyZ, hisx, hisy, hisz, hism)

    subplot(3, 1,1);
    plot(historyX);
    hold on
    plot(historyY);
    plot(historyZ);
    
    title("XYZ Position");
    legend("X", "Y", "Z");
    xlabel('Time');
    ylabel('Position(mm)');
    hold off
    
    subplot(3,1,2);
    plot(hisx);
    hold on
    plot(hisy);
    plot(hisz);
    
    legend("Force x", "Force y", "Force z");
    title("Force over Time");
    xlabel('Time');
    ylabel('Newtons(n)?');
    hold off
    
    subplot(3,1,3);
    plot(hism);
    
    legend("Force");
    title("Force Magnitude over Time");
    xlabel('Time');
    ylabel('Force');
    hold off
end