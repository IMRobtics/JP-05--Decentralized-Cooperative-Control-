figure(1);
%---------------------------------------------------------------------
%   Plotting the obstacle
%---------------------------------------------------------------------
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


%---------------------------------------------------------------------
%         Drawing lines between each robot
%---------------------------------------------------------------------

axis equal
% Calculating distance between all the robots
    %Computing distance of individual robot from all other robots
    r=zeros(number_of_robots,number_of_robots);
    for i=1:number_of_robots
        for j=1:number_of_robots
            a=robot{i}-robot{j};
            r(i,j)=sqrt(a(1)^2+a(2)^2);
        end
    end
    
    r
    
    smallest_distance_between_robots=min(r(r>0))  %returns the minimum no-zero value
    smallest_distance_between_robots=floor(smallest_distance_between_robots);
     
    r=zeros(number_of_robots,number_of_robots);
    for i=1:number_of_robots
        for j=1:number_of_robots
            a=robot{i}-robot{j};
            r(i,j)=sqrt(a(1)^2+a(2)^2);
            r(i,j)=floor(r(i,j));
            
            if number_of_robots<4
              first_robot=robot(i);
                xy_pos=cell2mat(first_robot);
                x1=xy_pos(1);
                y1=xy_pos(2);
    
                second_robot=robot(j);
                xy_pos=cell2mat(second_robot);
                x2=xy_pos(1);
            
                y2=xy_pos(2);
                plot(hyp_robot(1),hyp_robot(2),'o'),hold on
                plot([x1 x2],[y1 y2],'m--'),hold on,grid on
            
            
            elseif r(i,j)==smallest_distance_between_robots
                first_robot=robot(i);
                xy_pos=cell2mat(first_robot);
                x1=xy_pos(1);
                y1=xy_pos(2);
    
                second_robot=robot(j);
                xy_pos=cell2mat(second_robot);
                x2=xy_pos(1);
            
                y2=xy_pos(2);
                plot(hyp_robot(1),hyp_robot(2),'o'),hold on
                plot([x1 x2],[y1 y2],'m--'),hold on,grid on;
            
            end
            
        end
    end


robots_trajectory=cell2mat(robot_Trajectory_Vec);

%plotting staring point of the robots
plot(robots_trajectory(1,1),robots_trajectory(1,2),'ko'),hold on, grid on,
plot(robots_trajectory(1,3),robots_trajectory(1,4),'ro'),hold on, grid on,
plot(robots_trajectory(1,5),robots_trajectory(1,6),'bo'),hold on, grid on,
plot(robots_trajectory(1,7),robots_trajectory(1,8),'co'),hold on, grid on,
plot(robots_trajectory(1,9),robots_trajectory(1,10),'go'),hold on, grid on,


figure(1);
plot(robots_trajectory(:,1),robots_trajectory(:,2),'k-'),hold on, grid on,
plot(robots_trajectory(:,3),robots_trajectory(:,4),'r-'),hold on, grid on,
plot(robots_trajectory(:,5),robots_trajectory(:,6),'b-'),hold on, grid on,
plot(robots_trajectory(:,7),robots_trajectory(:,8),'c-'),hold on, grid on,
plot(robots_trajectory(:,9),robots_trajectory(:,10),'g-'),hold on, grid on,

% plot(robots_trajectory(:,11),robots_trajectory(:,12),'o-<'),hold on, grid on,

plot(virtual_robot_Trajectory_Vec(:,1),virtual_robot_Trajectory_Vec(:,2),'o-'),hold on, grid on,