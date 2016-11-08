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
robot{1}=[-5,1];

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

v1=width/2
v2=height/2
%Center of the obstacle
x0=xa+width/2;   
y0=ya+height/2;

%Defining ellipses around the obstacles
%defining range for x
x=-20:0.1:40;

A=sqrt(1/(2*(v1^2)));
B=sqrt(1/(2*(v2^2)));

y1=sqrt((1-A^2*(x-x0).^2)/B^2)+y0;
y2=-sqrt((1-A^2*(x-x0).^2)/B^2)+y0;
plot(x,y1,'g',x,y2,'g'),axis tight,grid on,hold on


v1_expansion=v1:v1/10:v1*2;
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