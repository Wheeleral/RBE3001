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

pos1 =[0, 0, 0]; %position for arm for 5 setpoints
pos2 =[ 0, 75, 398]; 
pos3 = [0, 253, 431];
pos4 = [0, 475 ,431];
pos5 = [0, 654, 588];

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
   
port=55;
arm.setPID([.0033 .0033 .0033], [0 0 0], [0.01 0.01 0.01]);

while 1
arm.getPos()

end
% 
% while 1
% 
%        port = 42;
%        returnValues = pp.command(port,values);
%        
%        angles = arm.getd();
%      % armDraw(arm, angles, links);
% 
%         disp(returnValues(3)); 
%         disp(returnValues(6)); 
%         disp(returnValues(9)); 
% 
%                 
%         pause(0.2)
%         
%        
%        
%      
% end
% pp.shutdown();
%     clear java;



function armDraw(arm, angles, links)
    angles(1) = angles(1)+90;
    p2 = arm.link2(angles(1), angles(2));
    p3 = arm.link3Draw(angles(1), angles(2), angles(3));

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