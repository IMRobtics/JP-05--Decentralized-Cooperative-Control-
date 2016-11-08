clear all;
close all;
clf;
clc;

%Dynamic characteristic of the robot
M=0.1;
B_model=0.1;
kd=2;
ksk=1;
kr=0.15;

number_of_robots=1;

xdot = zeros(number_of_robots,1);
ydot = zeros(number_of_robots,1);

x_pos_new=0;
y_pos_new=0;

%Vector to hold the position history
robot_Trajectory_Vec=[];
OBSTACLE=0;
Chi=0;
Psi=0;
%--------------------------------------------------------------------------




%--------------------------------------------------------------------------
%Staring point of the robot
x1=1;
y1=1;
robot{1}=[-10,0];

%Target point of the robot
x2=18;
y2=18;


% plot([x1 x2],[y1 y2],'r--'),hold on,...
%     plot(x1,y1,'o'),plot(x2,y2,'o'),grid on;

%--------------------------------------------------------------------------
%    Developing the Obstacle
%--------------------------------------------------------------------------
%Drawing ellipses arouond the obstacle
%xa,ya is the origin of the rectangle (bottom left corner as according to matlab rectangle command)
%(x0,y0) is the centre of the rectangle/obstacle
%the ellipse envelops a rectangle with (x0+-v1,y0+-v2) as its vertices

%defining some rectangular obstacles
origin_of_rectangle=[3,10];
width=6;
height=4;

figure(1);
rectangle('Position',[origin_of_rectangle width height],'Curvature',[0.05 0.05]),...
    axis equal,axis([-10 20 -10 20]),hold on

xa=origin_of_rectangle(1);
ya=origin_of_rectangle(2);

v1=width/2;
v2=height/2;
%Center of the obstacle
x0=xa+width/2;   
y0=ya+height/2;

%Plotting the ellipse encircling the obstacle
%defining range for x
x=-20:0.1:40;

A=sqrt(1/(2*(v1^2)));
B=sqrt(1/(2*(v2^2)));

y1=sqrt((1-A^2*(x-x0).^2)/B^2)+y0;
y2=-sqrt((1-A^2*(x-x0).^2)/B^2)+y0;
plot(x,y1,'g',x,y2,'g'),axis tight,grid on,hold on

% B1=0.1:0.1:0.5
% A1=(A/B)*B1

%Defining ellipses around the obstacles
v1_expansion=v1:v1/10:v1*1.25;
A1=sqrt(1./(2*(v1_expansion.^2)));
B1=(B/A)*A1;

for i=1:length(B1)
%     if(B1(i))>B
%         disp('I am here')
%     else
        y1=sqrt((1-A1(i)^2*(x-x0).^2)/B1(i)^2)+y0;
        y2=-sqrt((1-A1(i)^2*(x-x0).^2)/B1(i)^2)+y0;
        plot(x,y1,'--k',x,y2,'--k'),axis tight,grid on,hold on       
%     end
    
end
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------


%Moving the robot with time
% for i=1:number_of_robots

for t=1:600
    t
    dummy=robot(1);
    xy_pos=cell2mat(dummy);
    x_pos=xy_pos(1);
    y_pos=xy_pos(2);
    
    %Calculation of the angle with which robot is approaching the obstacle
    if(t>3)
    robot_Trajectory_Vec{t-2};
    robot_Trajectory_Vec{t-1};
    xy_coordinate=robot_Trajectory_Vec{t-1}-robot_Trajectory_Vec{t-2};
   
    Chi=atan2(y0-y_pos,x0-x_pos);    %Angle between the obstacle centre-robot  and the horizontal
    Psi=atan2(xy_coordinate(2),xy_coordinate(1));   %Robot Heading angle with the horizontal
    end
    
    
    %Contsant force applied on the robots
    fxkdes=0.1;
    fykdes=0.1;
%     fkdes=sqrt(fxkdes^2+fykdes^2);
    %     fyv=0.1+*sin(2*pi*(1/1000)*t);
    
    %---------------------------------------------------------------------    
    %Testing whether the robot is approaching the obstacle
    test=A1(end)^2*(x_pos-x0)^2+B1(end)^2*(y_pos-y0)^2-1;
    
    rk=sqrt(A^2*(x_pos-x0)^2+B^2*(y_pos-y0)^2-1);
    ra=1;
    
% %     %My old conditions with which the code was working
% %     if test<1e-3 
% %         OBSTACLE=1;
% %         %ra=sqrt((x_pos-x0)^2+(y_pos-y0)^2);
% %     else
% %         OBSTACLE=0;
% %     end

%New conditions as defined in the paper 
    if rk<=ra 
        OBSTACLE=1;
        %ra=sqrt((x_pos-x0)^2+(y_pos-y0)^2);
    else
        OBSTACLE=0;
    end


    %---------------------------------------------------------------------    
    %   Defining forces to avoid obstacle
    %---------------------------------------------------------------------    
%     if (OBSTACLE==0)
    if(rk>ra)
        fxkOA=fxkdes;
        fykOA=fykdes;
       
    
%     elseif (OBSTACLE==1) 
    else
%         fxkOA=fxkdes;
%         fykOA=fykdes;
%         
        fxkrc=(B/A)*(y_pos-y0);
        fykrc=-(A/B)*(x_pos-x0);
        fxkrcc=-(B/A)*(y_pos-y0);
        fykrcc=(A/B)*(x_pos-x0);
        
        fkdes=sqrt(fxkdes^2+fykdes^2);
        
        if (mod(Psi-Chi,2*pi)<=pi)   %Psi>=Chi
            fxkr=fxkrc;
            fykr=fykrc;
        elseif (mod(Psi-Chi,2*pi)>pi) %Psi<Chi
            fxkr=fxkrcc;
            fykr=fykrcc;
        end
        
         
            
            
        mod_fkr=sqrt(fxkr^2+fykr^2);
        fxkrn=fxkr/mod_fkr;
        fykrn=fykr/mod_fkr;
        
%         rk=sqrt(A^2*(x_pos-x0)^2+B^2*(y_pos-y0)^2-1);
        fxkOA=fxkdes+((abs(fkdes)*fxkrn)/(rk^2))*(1/rk-1/ra);
        fykOA=fykdes+((abs(fkdes)*fykrn)/(rk^2))*(1/rk-1/ra);
        
    end    
    
    % if(abs(FxkVS(i))>FORCE_THRESHOLD)
    fx = @(t,x) [x(2); (fxkOA-(B_model+kd)*x(2))/M];
    [T,X]=ode45(fx,[0,0.5],[x_pos;xdot]);
    [m,z] = size(X);
    x_pos_new=real(X(m,1));  %we used 'real' to avoid the complex values we get sometime when the solution of the differential equation necomes complex
    xdot=real(X(m,2));
    % end
    
    % if(abs(FykVS(i))>FORCE_THRESHOLD)
    fy = @(t,y) [y(2); (fykOA-(B_model+kd)*y(2))/M];
    [T,Y]=ode45(fy,[0,0.5],[y_pos;ydot]);
    [m,z] = size(Y);
    y_pos_new=real(Y(m,1));
    ydot=real(Y(m,2));
    
    robot{1}=[x_pos_new y_pos_new];
    robot_Trajectory_Vec=[robot_Trajectory_Vec;robot];
    % end
end

robots_trajectory=cell2mat(robot_Trajectory_Vec);
figure(1);
plot(robots_trajectory(:,1),robots_trajectory(:,2),'r-'),hold on, grid on,



