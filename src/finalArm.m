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

% light=  1.2475; %from second strain guage
%    heavy= 6.0582;
thresholdWeight= 7; %4.3;
PID=zeros(15, 1, 'single'); %set inital pid matrix

p = [0.01, 0.0045, 0.0045]; %.Good pid numbers
i = [0.0, 0.000, 0.00];
d = [1.0, 0.001, 0.001];
for j=0:2
    PID((j * 3) + 1) = p(j+1);
    PID((j * 3) + 2) = i(j+1);
    PID((j * 3) + 3) = d(j+1);
end
   
lightY=[230, -170, 30];
lightB=[30, -170, 30];
lightG=[130, -170, 30];

heavyY=[230, 170, 30];
heavyB=[30, 170, 30];
heavyG=[130, 170, 30];

port=55;
arm.setPID([.0033 .0033 .0033], [0 0 0], [0.01 0.01 0.01]);

if ~exist('cam', 'var')
    cam = webcam;
end


while 1
    arm.movepoint(167,0,200, 50);
    arm.openGripper();
    
    %find object
    object=closestObject(arm, cam);
    disp("Object found!");
    disp(object);
    if(object(1)<275)
    %move to object
    arm.movepickup(object(1), object(2), object(3));
    arm.closeGripper();
    pause(.5);
        
    %weigh object
    arm.movepoint(250, 0, 167, 150);
    pause(4);
    weight=arm.getWeight()
    if(weight>thresholdWeight)
        objectweight=1;
    else 
        objectweight=0;
    end
    pause(1);

    disp(object(4));
    disp(objectweight);  
    disp(determineSort(object(4), objectweight));
    sortpos=determineSort(object(4), objectweight)
    arm.movepoint(sortpos(1), sortpos(2), sortpos(3), 300);
    arm.openGripper();
    else
        disp("Too Many Objects on Field");
    end
    
end

function u = determineSort(obj, weight)
lightY=[230, 180, 30];
lightB=[30, 180, 30];
lightG=[130, 180, 30];

heavyY=[230, -180, 30];
heavyB=[30, -180, 30];
heavyG=[130, -180, 30];
    if (obj==1) && (weight==1)
        u=heavyB;
        
    elseif(obj==1) && (weight==0)
        u=lightB;
        
    elseif(obj==2) && (weight==1)
        u=heavyG;
        
    elseif(obj==2) && (weight==0)
        u=lightG;
        
    elseif(obj==3) && (weight==1)
        u=heavyY;
        
    else
        u=lightY;
    end    

end

%go in cam class
function u = closestObject(arm, cam)
    vals=[];
    visionVals= visionTest(cam);
    currPos=arm.getPos();

    while(isempty(visionVals) || strcmp(visionVals, 'null'))
         visionVals= visionTest(cam);
         disp("Waiting for object");
         pause(.5);
    end
   % disp(visionVals)
   size(visionVals,1);
    for i = 1:size(visionVals,1)

        obj=visionVals(i,1:3);
         vals=[vals [magnitude(currPos, obj); obj(1) ;obj(2); obj(3); visionVals(i, 4)]];
   
    end
    
    j =min(vals(1,:));
    for t = 1:size(visionVals,1)
        if(vals(1,t)==j)
            u=vals(2:5, t );
        else
            u=vals(2:5, 1 );
        end
            
    end
  
                


end

function u = magnitude(vec1, vec2)
    dist = vec2-vec1;
    sv = dist.*dist;
    dp = sum(sv);
    u =abs(sqrt(dp));

end