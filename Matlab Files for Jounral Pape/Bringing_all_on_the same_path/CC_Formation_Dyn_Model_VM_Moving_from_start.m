clc;
clear all;
close all;

%attraction and repulsion between charges

%A negative charge is places at the centre and four chrages are placed at
%different location

%-----------------------------------------------------------------------
%Initialization of parameters
%-----------------------------------------------------------------------
number_of_robots=6;

%Dynamic characteristic of the robot
% M=0.1;
% B=0.1;
% kd=2;
% ksk=1;
% kr=0.15;
% alpha=3; %radius of the circle



M=1;
B=1;
kd=9;
ksk=1;
kr=0.15;
alpha=3; %radius of the circle

%Charges on each robot
q(1:number_of_robots)=10;

FORCE_THRESHOLD = 1e-4;


%flag for testing whether the robots have developed a Swarm on not
SWARM_FLAG=0;


%Initialization of robots
robot=cell(1,number_of_robots);

%Initial Position of the robots
robot{1}=[-3,3];
robot{2}=[1,1];
robot{3}=[5,2];
robot{4}=[5,5];
robot{5}=[1,-1];
robot{6}=[10,6];
hyp_robot=[0,0];  %position of the hypothetical (Virtual Leader) Robot


xdot = zeros(number_of_robots,1);
ydot = zeros(number_of_robots,1);

% xdot_virtual=zeros(number_of_robots,1);
% ydot_virtual=zeros(number_of_robots,1);

xdot_virtual=0;
ydot_virtual=0;

%Vector to hold the position history
robot_Trajectory_Vec=[];
virtual_robot_Trajectory_Vec=[];



for t=1:800  %total simulation time
    t
    %Compute the attractive forces between the center (hypothetical robot)
    %and the actual robots
    %-----------------------------------------------------------
    %-----------------------------------------------------------
    %Electrostatic force by Coloumb's Law
    %F=K(q1q2)/r^2;
    %-----------------------------------------------------------
    %-----------------------------------------------------------
    
    %Computing the new position of Virtual Robot
    
%     if (SWARM_FLAG==0)
%         hyp_robot(1)=hyp_robot(1);
%         hyp_robot(2)=hyp_robot(2);
%         hyp_robot=hyp_robot;
%     elseif (SWARM_FLAG==1)
%        break;
        %Second method is to apply a force on the virtual robot which will
        %cause movement in the virtual robot
        
            fxv=0.5;
%             fyv=10;
            fyv=0.5*sin(2*pi*(1/1000)*t);
        
            fx_virtual = @(t,x) [x(2); (fxv-(B+kd)*x(2))/M];
            [T,X_virtual]=ode45(fx_virtual,[0,0.1],[hyp_robot(1);xdot_virtual]);
            [m,z] = size(X_virtual);
            x_pos_new=X_virtual(m,1);
            xdot_virtual=X_virtual(m,2);
        
              
            fy_virtual = @(t,y) [y(2); (fyv-(B+kd)*y(2))/M];
            [T,Y_virtual]=ode45(fy_virtual,[0,0.1],[hyp_robot(2);ydot_virtual]);
            [m,z] = size(Y_virtual);
            y_pos_new=Y_virtual(m,1);
            ydot_virtual=Y_virtual(m,2);
            
%             hyp_robot(1)=x_pos_new;
%             hyp_robot(2)=y_pos_new;
            hyp_robot=[x_pos_new y_pos_new];
            
%            SWARM_FLAG=0;
        
%     end
    
    
    
    %Computing the force for the first robot
    %Equation (6) of the paper
    
    %Computing distance of individual robot from all other robots
    r=zeros(number_of_robots,number_of_robots);
    for i=1:number_of_robots
        for j=1:number_of_robots
            a=robot{i}-robot{j};
            r(i,j)=sqrt(a(1)^2+a(2)^2);
        end
    end
    
%     r
    
    %Computing orientation of individual from all other robots
    theta=zeros(number_of_robots,number_of_robots);
    for i=1:number_of_robots
        for j=1:number_of_robots
            if i==j
                theta(i,j)=0;
            elseif i~=j
                a=robot{i}-robot{j};
                theta(i,j)=atan2(a(2),a(1));
            end
        end
    end
    
%     theta*180/pi
    
    
    %Computing Electrostatic forces (Repulsive) on individual robot
    %from all other robots
    
    Electrostatic_Forces=zeros(number_of_robots,number_of_robots);
    for i=1:number_of_robots
        for j=1:number_of_robots
            if i==j
                Electrostatic_Forces(i,j)=0;
            elseif i~=j
                Electrostatic_Forces(i,j)=(kr*q(i)*q(j))/(r(i,j)^2);
            end
        end
    end
%     Electrostatic_Forces
    
    
    
    %Decomposition of Electrostatic forces in x and y components
    for i=1:number_of_robots
        for j=1:number_of_robots
            EF_x(i,j)=Electrostatic_Forces(i,j)*cos(theta(i,j));
            EF_y(i,j)=Electrostatic_Forces(i,j)*sin(theta(i,j));
        end
    end
    
    %Computing x and y components of Attractive Force (Equation 8 in JP)

    
    for i=1:number_of_robots
        a=robot{i}-hyp_robot;
        Attractive_Force_x(i)=ksk*(a(1)*(a(1)^2+a(2)^2-alpha^2));
        Attractive_Force_y(i)=ksk*(a(2)*(a(1)^2+a(2)^2-alpha^2));
    end
    
    
    
    %Computing resultant forces on each robot
    for i=1:number_of_robots
        FxkVS(i)=sum(EF_x(i,1:number_of_robots))-Attractive_Force_x(i);
        FykVS(i)=sum(EF_y(i,1:number_of_robots))-Attractive_Force_y(i);
    end
    
%     for i=1:number_of_robots
%         F_res(i)=sqrt(F_x_res(i)^2+F_y_res(i)^2);
%         theta_res(i)=atan2(F_y_res(i),F_x_res(i));
%     end
%     F_res
%     
    for i=1:number_of_robots
        
        dummy=robot(i);
        xy_pos=cell2mat(dummy);
        x_pos=xy_pos(1);
        y_pos=xy_pos(2);
        
                
        %initializing x_pos_new and y_pos_new
        x_pos_new=x_pos;
        y_pos_new=y_pos;
        
        distance_from_hyp_robot=sqrt((x_pos-hyp_robot(1))^2+(y_pos-hyp_robot(2))^2);
        
        if(abs(FxkVS(i))>FORCE_THRESHOLD)
            fx = @(t,x) [x(2); (FxkVS(i)-(B+kd)*x(2))/M];
            [T,X]=ode45(fx,[0,0.1],[x_pos;xdot(i)]);
            [m,z] = size(X);
            x_pos_new=X(m,1);
            xdot(i)=X(m,2);
        end
        
        if(abs(FykVS(i))>FORCE_THRESHOLD)
            fy = @(t,y) [y(2); (FykVS(i)-(B+kd)*y(2))/M];
            [T,Y]=ode45(fy,[0,0.1],[y_pos;ydot(i)]);
            [m,z] = size(Y);
            y_pos_new=Y(m,1);
            ydot(i)=Y(m,2);
        end
% 
%         if abs(FxkVS(i))<=FORCE_THRESHOLD &&  abs(FykVS(i))<=FORCE_THRESHOLD
%             x_pos_new=x_pos;
%             y_pos_new=y_pos;
%             SWARM_FLAG=1
% 
%         
%         end
        
        robot{i}=[x_pos_new y_pos_new];
        robot_Trajectory_Vec=[robot_Trajectory_Vec;robot];
        
    end
     virtual_robot_Trajectory_Vec=[virtual_robot_Trajectory_Vec;hyp_robot];
    
    
%     %Drawing lines connecting all the robots
%     if (SWARM_FLAG==1)
%         

    
        
    
end %end of the bigger loop

%---------------------------------------------------------------------
%         Drawing lines between each robot
%---------------------------------------------------------------------

% % % % figure(1);
% % % % axis equal
% % % % % Calculating distance between all the robots
% % % %     %Computing distance of individual robot from all other robots
% % % %     r=zeros(number_of_robots,number_of_robots);
% % % %     for i=1:number_of_robots
% % % %         for j=1:number_of_robots
% % % %             a=robot{i}-robot{j};
% % % %             r(i,j)=sqrt(a(1)^2+a(2)^2);
% % % %         end
% % % %     end
% % % %     
% % % %     r
% % % %     
% % % %     smallest_distance_between_robots=min(r(r>0))  %returns the minimum no-zero value
% % % %     smallest_distance_between_robots=floor(smallest_distance_between_robots);
% % % %      
% % % %     r=zeros(number_of_robots,number_of_robots);
% % % %     for i=1:number_of_robots
% % % %         for j=1:number_of_robots
% % % %             a=robot{i}-robot{j};
% % % %             r(i,j)=sqrt(a(1)^2+a(2)^2);
% % % %             r(i,j)=floor(r(i,j));
% % % %             
% % % %             if number_of_robots<4
% % % %               first_robot=robot(i);
% % % %                 xy_pos=cell2mat(first_robot);
% % % %                 x1=xy_pos(1);
% % % %                 y1=xy_pos(2);
% % % %     
% % % %                 second_robot=robot(j);
% % % %                 xy_pos=cell2mat(second_robot);
% % % %                 x2=xy_pos(1);
% % % %             
% % % %                 y2=xy_pos(2);
% % % %                 plot(hyp_robot(1),hyp_robot(2),'o'),hold on
% % % %                 plot([x1 x2],[y1 y2],'r'),hold on,grid on
% % % %             
% % % %             
% % % %             elseif r(i,j)==smallest_distance_between_robots
% % % %                 first_robot=robot(i);
% % % %                 xy_pos=cell2mat(first_robot);
% % % %                 x1=xy_pos(1);
% % % %                 y1=xy_pos(2);
% % % %     
% % % %                 second_robot=robot(j);
% % % %                 xy_pos=cell2mat(second_robot);
% % % %                 x2=xy_pos(1);
% % % %             
% % % %                 y2=xy_pos(2);
% % % %                 plot(hyp_robot(1),hyp_robot(2),'o'),hold on
% % % %                 plot([x1 x2],[y1 y2],'r'),hold on,grid on;
% % % %             
% % % %             end
% % % %             
% % % %         end
% % % %     end
% % % % 


robots_trajectory=cell2mat(robot_Trajectory_Vec);
k=1;
for i=1:number_of_robots
    
    X_for_polygon(i)= robots_trajectory(end,k);
    Y_for_polygon(i)=robots_trajectory(end,k+1);
    k=k+2;
end
k=convhull(X_for_polygon,Y_for_polygon);
figure(1);
plot(X_for_polygon(k),Y_for_polygon(k)),hold on;


plot(robots_trajectory(:,1),robots_trajectory(:,2),'k-'),hold on, grid on,
plot(robots_trajectory(:,3),robots_trajectory(:,4),'r-'),hold on, grid on,
plot(robots_trajectory(:,5),robots_trajectory(:,6),'b-'),hold on, grid on,
plot(robots_trajectory(:,7),robots_trajectory(:,8),'c-'),hold on, grid on,
plot(robots_trajectory(:,9),robots_trajectory(:,10),'g-'),hold on, grid on,
plot(robots_trajectory(:,11),robots_trajectory(:,12),'m-'),hold on, grid on,

plot(virtual_robot_Trajectory_Vec(:,1),virtual_robot_Trajectory_Vec(:,2),'o-'),hold on, grid on,

 


% %drawing a circle of radius=alpha
% count=1;
% for theta=0:0.1:2*pi
% x_circle(count)=(alpha+0.5)*cos(theta);
% y_circle(count)=(alpha+0.5)*sin(theta);
% count=count+1;
% end

% x_circle=x_circle+hyp_robot(1);
% y_circle=y_circle+hyp_robot(2);
plot(hyp_robot(1),hyp_robot(2),'o'),hold on
% plot(x_circle,y_circle,'r-'),hold on,

