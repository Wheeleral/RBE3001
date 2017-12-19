classdef Arm 
    properties
        links;
        pp; %packet processor
        randSet;
    end
    methods
        
        function arm = Arm(links, pp) %create the arm
            arm.links = links;
            
            javaaddpath('../lib/hid4java-0.5.1.jar');

            import org.hid4java.*;
            import org.hid4java.event.*;
            import java.nio.ByteBuffer;
            import java.nio.ByteOrder;
            import java.lang.*;
            
            arm.pp = pp;
            arm.randSet = zeros(15, 1, 'single');
        end
           
        %set the pid of the arm
        function setPID(arm, arm1, arm2, arm3)
            PIDarray = zeros(15, 1, 'single');
            PIDarray(1:3) = [arm1(1), arm2(1), arm3(1)];
            PIDarray(4:6) = [arm1(2), arm2(2), arm3(2)];
            PIDarray(7:9) = [arm1(3), arm2(3), arm3(3)];
            arm.pp.command(55, PIDarray);
        end
        
        function close(arm) %close the communication
            arm.pp.shutdown();
            clear java;
        end
        
        function u = getRaw(arm) %get the encoder values
            u = arm.pp.command(42, arm.randSet);
        end
        
        
        function u = getRawAng(arm)
            raw = arm.getRaw();
            u = [raw(1) raw(4) raw(7)];
        end
        
        function u = getd(arm) %get the encoder values in regards to degrees
            raw = arm.getRaw();
            
            u = [90*raw(1)/1000; 90*raw(4)/1000; -90*raw(7)/1000];
        end
        
         function u = getV(arm) %get the encoder values in regards to degrees
            raw = arm.getRaw();
            
            u = [raw(2) 90*raw(5) -90*raw(8)/1000];
        end
        
        function u = set(arm, angles)% set hte arm to move to a certain angle 
            values = zeros(15, 1, 'single');
            
             for j=0:4
                 values((j * 3) + 1) = angles(1, mod(j, 3)+1);
                 %Send junk data for velocity and force targets
                 values((j * 3) + 2) = 0;
                 values((j * 3) + 3) = 3;
             end
             
            u = arm.pp.command(37, values);
        end
        
        function u = setd(arm, angles)% set hte arm to move to a certain angle 
            values = zeros(15, 1, 'single');
            angles = [1000*(angles(1)/90) 1000*(angles(2)/90) -1000*(angles(3)/90)];
             for j=0:4
                 values((j * 3) + 1) = angles(1, mod(j, 3)+1);
                 %Send junk data for velocity and force targets
                 values((j * 3) + 2) = 0;
                 values((j * 3) + 3) = 3;
             end
             
            u = arm.pp.command(37, values);
            
            
        end
        
        function setVelocity(arm, vel, time, curPos)%set the "velocity" with time steps
            values = zeros(15, 1, 'single');
            
            set = vel*time+curPos;
            
            for j=0:4
                 values((j * 3) + 1) = (set(mod(j, 3) +1));
                 values((j * 3) + 2) = 0;
                 values((j * 3) + 3) = 0;
            end
            disp(values)
            arm.pp.command(37, values)
        end
        
        function setPoint(arm, pos)% send an angle to the arm (triangle)
            setAng = arm.getSetAngles(pos);
            arm.pp.command(37, setAng);
        end
        
        function u = getSetAngles(arm, pos)  %triangle math
            a10 = [0;0;arm.links(1)];
            t0 = [pos(1); pos(2); pos(3)];
            psir = atan(pos(2)/pos(1));
            
            if(pos(2)==0)
                psir = 0;
            end
            
            psi = psir*180/pi;

            % move from base coord system to system 1
            Rz = [cos(psir) sin(psir) 0; -sin(psir) cos(psir) 0; 0 0 1];
            t1 = Rz*(t0-a10);

            % remaining links form triangle with known side lengths using
            % l2, l2, and position vector in coord system 1.
            % solve for angles
            c = norm(t1);
            phir = acos((arm.links(2)^2+arm.links(3)^2-c^2)/(2*arm.links(2)*arm.links(3)))-pi/2;
            phi = phir*180/pi;
            alpha = acos((arm.links(2)^2-arm.links(3)^2+c^2)/(2*arm.links(2)*c));
            beta = atan(t1(3,1)/t1(1,1));
            thetar = alpha+beta;
            theta = thetar*180/pi;

            u = [psi, theta, phi];
        end
               
        % forward kinematics of the tip of link2
        function u = link2(arm, q1, q2)
            x = cosd(q1)*arm.links(2)*cosd(q2);
            y = cosd(q2)*arm.links(2)*sind(q1);
            z = sind(q2)*arm.links(2)+arm.links(1);

            u = [x y z];
        end
        
        % forward kinematics of the tip of link3
        function u = link3(arm, q1, q2, q3)
            y = (arm.links(2)*cosd(q2) + arm.links(3)*sind(q2-q3))*cosd(q1); % calculate x-pos of the endefector
            x = (arm.links(2)*cosd(q2) + arm.links(3)*sind(q2-q3))*sind(q1) - arm.links(2); % calculate y-pos of the endefector
            z = arm.links(1)+arm.links(2)*sind(q2) - arm.links(3)*cosd(q2-q3); % calculate z-pos of the endefector

            u= [x y z];
        end
        
        %modified link3 for drawing
        function u = link3Draw(arm, q1, q2, q3)
            x = (arm.links(2)*cosd(q2) + arm.links(3)*sind(q2-q3))*cosd(q1); % calculate x-pos of the endefector
            y = (arm.links(2)*cosd(q2) + arm.links(3)*sind(q2-q3))*sind(q1) ; % calculate y-pos of the endefector
            z = arm.links(1)+arm.links(2)*sind(q2) - arm.links(3)*cosd(q2-q3); % calculate z-pos of the endefector

            u= [x y z];
        end
        
        function u = getPos(arm)
            deg=arm.getd();
            u= arm.link3Draw(deg(1), deg(2), deg(3));
        end
        
        %postion transformation dh params
        function u = positionTransform(arm, q1, q2, q3)
            a = [0 1.6 2.0];
            alpha = [-90 0 90];
            d = [.9 0 0]; %m
            
            x = arm.transformDH(a(1), alpha(1),d(1), q1) ;
            y = arm.transformDH(a(2), alpha(2), d(2), q2-90) ;
            z = arm.transformDH(a(3), alpha(3), d(3), -q3+90);
            u=x*y*z;
        end
        
        %get the dh param matrix
        function u = transformDH(arm, a, alpha, d, theta)
            alpha=(alpha/180)*pi;
            theta=(theta/180)*pi;
            
            u = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha)  a*cos(theta); ...
                sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); ...
                0            sin(alpha)             cos(alpha)              d; ...
                0            0                       0                        1];
        end
        
        %calulate the jacobian
        function u = jacob(arm, q1, q2, q3)
            syms q1 q2 q3
            x = (arm.links(2)*cosd(q2) + arm.links(3)*sind(q2-q3))*cosd(q1);
            y = (arm.links(2)*cosd(q2) + arm.links(3)*sind(q2-q3))*sind(q1); % calculate y-pos of the endefector
            z = arm.links(1)+arm.links(2)*sind(q2) - arm.links(3)*cosd(q2-q3); % calculate z-pos of the endefector
           % x1= diff(x, q1);
            
            u =[jacobian(x, [sq1 sq2 sq3]);jacobian(y, [sq1 sq2 sq3]);jacobian(z, [sq1 sq2 sq3])];

        end 
        
        %velocity kinimatics
        function u = velocityKin(arm, q1, q2, q3, angV1, angV2 , angV3) 
            %convert angles to radians
            syms q1 q2 q3
            pos = positionTransform( q1, q2, q3);
            Jacob = jacob(pos);
            
            u = Jacob*[angV1; angV2; angV3; 1];            
            
        end 
        
        %inverse velocity kinimatics
        function u =invVelocityKin(arm, q1, q2, q3, v1, v2 , v3)
           q1=deg2rad(q1);
           q2=deg2rad(q2);
           q3=deg2rad(q3);
            
           syms sq1 sq2 sq3 %solve symbolically
          
            x = (arm.links(2)*cos(sq2) + arm.links(3)*sin(sq2-sq3))*cos(sq1);
            y = (arm.links(2)*cos(sq2) + arm.links(3)*sin(sq2-sq3))*sin(sq1); % calculate y-pos of the endefector
            z = arm.links(1)+arm.links(2)*sin(sq2) - arm.links(3)*cos(sq2-sq3); % calculate z-pos of the endefector
            
            Jacob = [jacobian(x, [sq1 sq2 sq3]);jacobian(y, [sq1 sq2 sq3]);jacobian(z, [sq1 sq2 sq3])];
            invf =inv(Jacob);
            invs=subs(invf,[sq1 sq2 sq3], [q1 q2 q3]);
            u = rad2deg(double(invs*[v1; v2; v3]));
        end
        
        %inverse position kinimatics
        function u = inversePosition(arm, px, py, pz)
            
           pa=sqrt(px^2+py^2);
           pz=pz-arm.links(1);
           
            theta1 = atand(py/px);
            theta2=atand(pz/pa)+acosd((pa^2+pz^2+arm.links(2)^2-arm.links(3)^2)/(2*arm.links(2)*sqrt(pa^2+pz^2)));
            theta3=-acosd(((pa^2+pz^2)-(arm.links(2)^2+arm.links(3)^2))/(2*(arm.links(2)*arm.links(3))));
            
            u=[theta1 theta2 -(theta3+90)];
        end
        
        function u = checkBounds(arm, px, py, pz)
            if(pz>(arm.links(2)+arm.links(3)*2))
                u=0;
            elseif(pz<0)
                u=0;
            elseif(sqrt(px^2+py^2+pz^2)>(arm.links(2)+arm.links(3)))
                u=0;
            elseif(sqrt(px^2 +py^2+(pz+200)^2)<175)%just above the min
                u=0;
            else
                u=1;
            end 
        end
        
        %set the angles of the arm with regards to the inverse position
        %kinimatics
        function setPositionInverse(arm, px, py, pz)
            angles= arm.inversePosition(px, py, pz);
             arm.setd(angles);
           
            
        end
        
        function u = getWeight(arm)
            values= zeros(15, 1, 'single');
            weight=[];
           for i= 1:5
               port = 42;
               returnValues = arm.pp.command(port,values) ;           
               weight=[weight returnValues(6)];
               
           end 
            u=mean(weight);
            
        end
        
        function u = atSetpoint(arm, px, py, pz)
            angles=arm.getd();
            currXYZ =arm.link3Draw(angles(1), angles(2), angles(3));
                
            u = islogical((abs(currXYZ(1)-px)<8) & (abs(currXYZ(2)-py)<8) & (abs(currXYZ()-pz)<5));
        end
        
        function u = forceTorque(arm, q1, q2, q3, t1, t2, t3)
           q1=deg2rad(q1);
           q2=deg2rad(q2);
           q3=deg2rad(q3);
            
           syms sq1 sq2 sq3 %solve symbolically
%             nx=0;
%             ny=0'
%             nz=0;            
            ft=[t1; t2; t3];

            x = (arm.links(2)*cos(sq2) + arm.links(3)*sin(sq2-sq3))*cos(sq1);
            y = (arm.links(2)*cos(sq2) + arm.links(3)*sin(sq2-sq3))*sin(sq1); % calculate y-pos of the endefector
            z = arm.links(1)+arm.links(2)*sin(sq2) - arm.links(3)*cos(sq2-sq3); % calculate z-pos of the endefector
            
            Jacob = [jacobian(x, [sq1 sq2 sq3]);jacobian(y, [sq1 sq2 sq3]);jacobian(z, [sq1 sq2 sq3])];
            JT =transpose(Jacob);
            invJT= pinv(JT);
            
            invJTS=subs(invJT,[sq1 sq2 sq3], [q1 q2 q3]);
            u = rad2deg(double(invJTS*ft));
        end
        
        function u = getTorque(arm)
            vals = arm.getRaw();
            u=[vals(3); vals(6); vals(9)];
        end
        
        function u = calcForce(arm, angles)
            q1=angles(1);
            q2=angles(2);
            q3=angles(3);
            torque= arm.getTorque();
             q1=deg2rad(q1);
           q2=deg2rad(q2);
           q3=deg2rad(q3);
            
           syms sq1 sq2 sq3 %solve symbolically
%             nx=0;
%             ny=0'
%             nz=0;            
            ft=[torque(1); torque(2); torque(3)];

            x = (arm.links(2)*cos(sq2) + arm.links(3)*sin(sq2-sq3))*cos(sq1);
            y = (arm.links(2)*cos(sq2) + arm.links(3)*sin(sq2-sq3))*sin(sq1); 
            z = arm.links(1)+arm.links(2)*sin(sq2) - arm.links(3)*cos(sq2-sq3); 
            
            Jacob = [jacobian(x, [sq1 sq2 sq3]);jacobian(y, [sq1 sq2 sq3]);jacobian(z, [sq1 sq2 sq3])];
            JT =transpose(Jacob);
            invJT= pinv(JT);
            
            invJTS=subs(invJT,[sq1 sq2 sq3], [q1 q2 q3]);
            u = rad2deg(double(invJTS*ft));
        end
        
        function u = openGripper(arm)%open the gripper by setting the first number to 1 (needs to be greater than zero)
            values = zeros(15, 1, 'single');            
            values(1) = 1;
             
            u = arm.pp.command(5, values);
        end
        
        function u = closeGripper(arm) %close the gripper by setting first number to 0
            values = zeros(15, 1, 'single');         
            values(1) = 0;
             
            u = arm.pp.command(5, values);
        end
        

        
        
        function move(arm, px, py, pz)
            currpos=arm.getPos();
            arm.vectorMove(currpos(1), currpos(2), 100, 120);
            pause(1);
            arm.vectorMove(px, py, 100,50);
            pause(1);
            arm.vectorMove(px, py, pz,40);
            pause(1);
        end  
        
        function movepoint(arm, px, py, pz, timestep)
            currpos=arm.getPos();
            arm.vectorMove(px, py, pz, timestep);
            

        end  
        function movepickup(arm, px, py, pz)
            currpos=arm.getPos();
            arm.vectorMove(px, py, 100,50);
            pause(1);
            arm.vectorMove(px, py, pz,40);
            pause(1);
            
        end
        
        function vectorMove(arm,  px, py, pz, spacing)
            currpos=arm.getPos();
            pcx=currpos(1);
            pcy=currpos(2);
            pcz=currpos(3);
         vals=arm.trajectoryparams(0,1, pcx, px, pcy, py, pcz, pz, spacing, 0, 0);

           for j = 1:spacing
                arm.setPositionInverse(vals(2,j),vals(3,j),vals(4,j));
                isNot = not(arm.atSetpoint(vals(2,j),vals(3,j),vals(4,j)));
                while(isNot)
                    pause(0.02);
                    isNot = not(arm.atSetpoint(vals(2,j),vals(3,j),vals(4,j)));
                end
                pause(.02);
            end 
            
        end
        
        % 0, tf = total time, xyzo = starting pos, xyzf = final pos, timestep =
        % num of generated points, vo = initial vel, vf = final vel
        function u=trajectoryparams(arm, to,tf, xo, xf, yo, yf, zo, zf, timestep, vo, vf)
        %make quintic to get rid  of some of the jerkyness
            ao=0;
            af=0;

            A=[1 to to^2 to^3 to^4 to^5;...
               0 1 2*to 3*to^2 4*to^3 5*to^4;...
               0 0 2 6*to 12*to^2 20*to^3; ...
               1 tf tf^2 tf^3 tf^4 tf^5;...
               0 1 2*tf 3*tf*tf to^4 to^5; ...
               0 0 2 6*tf 12*tf^2 20*tf^3];

            qx=[xo; vo; 0; xf; vf; 0];
            qy = [yo; vo; 0; yf; vf; 0];
            qz = [zo; vo; 0; zf; vf; 0];

            ax=inv(A)*qx;
            ay=inv(A)*qy;
            az=inv(A)*qz;
            t=linspace(0,tf, timestep);

            u=[t; 
                [ax(1)+ax(2)*t+ax(3)*t.^2+ax(4)*t.^3 + ax(5)*t.^4+ax(6)*t.^5;
                ay(1)+ay(2)*t+ay(3)*t.^2+ay(4)*t.^3 + ay(5)*t.^4+ay(6)*t.^5;
                az(1)+az(2)*t+az(3)*t.^2+az(4)*t.^3+ az(5)*t.^4+az(6)*t.^5];

                [ax(2)+ax(3)*2*t+ax(4)*3*t.^2;
                ay(2)+ay(3)*2*t+ay(4)*3*t.^2;
                az(2)+az(3)*2*t+az(4)*3*t.^2];

                [ax(3)*2+ax(4)*6*t;
                ay(3)*2+ay(4)*6*t;
                az(3)*2+az(4)*6*t]
                ];
        end
        
    end
end