clc;
clear all;
close all;

%attraction and repulsion between charges

%A negative charge is places at the centre and four chrages are placed at
%different location

%-----------------------------------------------------------------------
%Initialization of parameters
%-----------------------------------------------------------------------
number_of_robots=3;

%Dynamic characteristic of the robot
%My values
M=0.1;
B_model=0.1;
kd=2;
ksk=1;
kr=0.15;
alpha=3; %radius of the circle

% % M=1;
% % B_model=1;
% % kd=9;
% % ksk=100;
% % kr=10;
% % alpha=3; %radius of the circle


kp=10;
tau=1;
tspan_ode45=[0.0 0.1];
%Charges on each robot
q(1:number_of_robots)=10;
test=zeros(1,number_of_robots);
OBSTACLE=zeros(1,number_of_robots);



%flag for testing whether the robots have developed a Swarm on not
%Initially the SWARM_FLAG is set to ZERO because the robots are not in a
%swarm formation

SWARM_FLAG=0;

%Defining and initializing angles for obstacle avoidance
Chi=zeros(1,number_of_robots);
Psi=zeros(1,number_of_robots);

%Initialization of robots
robot=cell(1,number_of_robots);

%Initial Position of the robots
robot{1}=[-3,3];
robot{2}=[1,1];
robot{3}=[5,2];
robot{4}=[5,5];
robot{5}=[1,-1];
% robot{6}=[10,6];
hyp_robot=[0,0];  %position of the hypothetical (Virtual Leader) Robot


xdot = zeros(number_of_robots,1);
ydot = zeros(number_of_robots,1);
xdot_virtual=zeros(number_of_robots,1);
ydot_virtual=zeros(number_of_robots,1);


%Vector to hold the position history
robot_Trajectory_Vec=[];
virtual_robot_Trajectory_Vec=[];

%-----------------------------------------------------------------------
%                   Code for Animated LInes
%------------------------------------------------------------------------

color = 'kbgrcmy'; colorVal=1;
robotTrajectory_ZP = [animatedline('Color',color(colorVal),'LineWidth',2)];
for i = 1: number_of_robots-1
    colorVal = colorVal+1;
    if(colorVal>7)
        colorVal=1;
    end
    robotTrajectory_ZP = [robotTrajectory_ZP;animatedline('Color',color(colorVal),'LineWidth',2)];
end


%--------------------------------------------------------------------------
%    Developing the Obstacle
%--------------------------------------------------------------------------
%Drawing ellipses arouond the obstacle
%xa,ya is the origin of the rectangle (bottom left corner as according to matlab rectangle command)
%(x0,y0) is the centre of the rectangle/obstacle
%the ellipse envelops a rectangle with (x0+-v1,y0+-v2) as its vertices

%defining some rectangular obstacles
origin_of_rectangle=[5,7];
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

y1=real(sqrt((1-A^2*(x-x0).^2)/B^2)+y0);
y2=real(-sqrt((1-A^2*(x-x0).^2)/B^2)+y0);
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
    y1=real(sqrt((1-A1(i)^2*(x-x0).^2)/B1(i)^2)+y0);
    y2=real(-sqrt((1-A1(i)^2*(x-x0).^2)/B1(i)^2)+y0);
    plot(x,y1,'--k',x,y2,'--k'),axis tight,grid on,hold on
    %     end
    
end
%------------------------------------------------------------------------


for t=1:190  %total simulation time [The main BIG Loop]
    %     robot{i}=[x_pos_new y_pos_new];
    robot_Trajectory_Vec=[robot_Trajectory_Vec;robot];
    t
    
    for i=1:number_of_robots
        dummy=robot(i);
        xy_pos=cell2mat(dummy);
        x_pos(i)=xy_pos(1);
        y_pos(i)=xy_pos(2);
    end
    
    
    %----------------------------------------------------------------------
    %Calculation of the angle with which robot is approaching the obstacle
    %We are using condition t>3 because the robot must have moved a little
    %distance in order to calculate the heading angle. Heading angle is
    %based on its current and previous position.
    if(t>3)
        for i=1:number_of_robots
            
            final_pos=cell2mat(robot_Trajectory_Vec(t-1,i));
            initial_pos=cell2mat(robot_Trajectory_Vec(t-2,i));
            %         robot_Trajectory_Vec{t-2}
            %         robot_Trajectory_Vec{t-1}
            %         xy_coordinate=robot_Trajectory_Vec{t-1}-robot_Trajectory_Vec{t-2}
            xy_coordinate=final_pos-initial_pos;
            Chi(i)=atan2(y0-y_pos(i),x0-x_pos(i));    %Angle between the obstacle centre-robot  and the horizontal
            Psi(i)=atan2(xy_coordinate(2),xy_coordinate(1));   %Robot Heading angle with the horizontal
        end
        %     Chi, Psi
    end
    %---------------------------------------------------------------------
    %Testing whether the robot is approaching the obstacle
    %If the 'test' value is approx 1e-3 it means that the robot uis very
    %near the ellipse. If the robot is exactly on the ellipse then 'test'
    %must be ZERO. [This was the first approach I was using for obstacle avoidance]
    
    %The second approach for testing whetner the robot is approaching the
    %obstacle or not is based on the journal paper, it compares ra with rk
    %and then takes a decision.
    
    for i=1:number_of_robots
        %     test(i)=A1(end)^2*(x_pos(i)-x0)^2+B1(end)^2*(y_pos(i)-y0)^2-1;
        
        rk(i)=sqrt(A^2*(x_pos(i)-x0)^2+B^2*(y_pos(i)-y0)^2-1);
        ra(i)=1;
        
        %          My old conditions with which the code was working
        %         if test(i)<1e-3
        %             OBSTACLE(i)=1
        %             ra(i)=sqrt((x_pos(i)-x0)^2+(y_pos(i)-y0)^2);
        %         else
        %             OBSTACLE(i)=0
        %         end
        
%         %New conditions as defined in the paper
%         if rk(i)<=ra(i)
%             OBSTACLE(i)=1;
%             %ra=sqrt((x_pos-x0)^2+(y_pos-y0)^2);
%         else
%             OBSTACLE(i)=0;
%         end
%         OBSTACLE;
%         
        %---------------------------------------------------------------------
        %   Defining forces to avoid obstacle
        %---------------------------------------------------------------------
        if (rk(i)>ra(i))
            %         fxkOA(i)=fxkdes(i);
            %         fykOA(i)=fykdes(i);
            %
            fxkOA(i)=0;
            fykOA(i)=0;
            
            %         elseif (sum(OBSTACLE)>1)
        elseif (rk(i)<=ra(i))
            %         fxkOA=fxkdes;
            %         fykOA=fykdes;
            %
            SWARM_FLAG=0;
            fxkrc(i)=(B/A)*(y_pos(i)-y0);
            fykrc(i)=-(A/B)*(x_pos(i)-x0);
            fxkrcc(i)=-(B/A)*(y_pos(i)-y0);
            fykrcc(i)=(A/B)*(x_pos(i)-x0);
            
            %             fkdes(i)=sqrt(fxkdes(i)^2+fykdes(i)^2);
            
            if (mod(Psi(i)-Chi(i),2*pi)<=pi)   %Psi>=Chi
                fxkr(i)=fxkrc(i);
                fykr(i)=fykrc(i);
            elseif (mod(Psi(i)-Chi(i),2*pi)>pi) %Psi<Chi
                fxkr(i)=fxkrcc(i);
                fykr(i)=fykrcc(i);
            end
            
            
            mod_fkr(i)=sqrt(fxkr(i)^2+fykr(i)^2);
            fxkrn(i)=fxkr(i)/mod_fkr(i);
            fykrn(i)=fykr(i)/mod_fkr(i);
            
%             rk(i)=sqrt(A^2*(x_pos(i)-x0)^2+B^2*(y_pos(i)-y0)^2-1);
            %         fxkOA(i)=fxkdes(i)+((abs(fkdes(i))*fxkrn(i))/(rk(i)^2))*(1/rk(i)-1/ra(i));
            %         fykOA(i)=fykdes(i)+((abs(fkdes(i))*fykrn(i))/(rk(i)^2))*(1/rk(i)-1/ra(i));
            
        end
    end
    
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %            Computing the new position of Virtual Robot
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    
    %I have to verify the following if condition, I think that the virtual
    %robot should not move when the SWARM has not been developed. The
    %movement of the virtual robot is prevented when SWARM_FLAG=0, once the
    %SWARM_FLAG=1 i.e. swarm has been established then the virtual robot
    %starts to move.
    
    % % %      %This code was used when I was not using obstacles
    % % %     if (SWARM_FLAG==0)
    % % %         hyp_robot=hyp_robot;
    % % %     elseif (SWARM_FLAG==1)
    % % %
    % % %             fxv=0.1;
    % % %             fyv=0.1;
    % % % %             fyv=0.1*sin(2*pi*(1/1000)*t);
    % % %
    % % %             fx_virtual = @(t,x) [x(2); (fxv-(B_model+kd)*x(2))/M];
    % % %             [T,X_virtual]=ode45(fx_virtual,tspan_ode45,[hyp_robot(1);xdot_virtual(i)]);
    % % %             [m,z] = size(X_virtual);
    % % %             x_pos_new=X_virtual(m,1);
    % % %             xdot_virtual(i)=X_virtual(m,2);
    % % %
    % % %
    % % %             fy_virtual = @(t,y) [y(2); (fyv-(B_model+kd)*y(2))/M];
    % % %             [T,Y_virtual]=ode45(fy_virtual,tspan_ode45,[hyp_robot(2);ydot_virtual(i)]);
    % % %             [m,z] = size(Y_virtual);
    % % %             y_pos_new=Y_virtual(m,1);
    % % %             ydot_virtual(i)=Y_virtual(m,2);
    % % %             hyp_robot=[x_pos_new y_pos_new];
    % % %
    % % % %            SWARM_FLAG=0;
    % % %
    % % %     end
    
    %Code for updating the Virtual robot position when I am using Obstacles
    rm=min(rk);
    fxvdes=1.5;  %force on the virtual robot
    fyvdes=1.5;
    
    
%     if (sum(OBSTACLE)==0)
    if (rm>ra)
        fxvBS_virtual=fxvdes;
        fyvBS_virtual=fyvdes;
        
        %     elseif (sum(OBSTACLE)>1)  %The journal paper has used different condition
%     elseif (sum(OBSTACLE)>1)   %(rm<=ra)
    elseif (rm<=ra)
        xm=mean(x_pos);
        ym=mean(y_pos);
        
        fxvBS_virtual=fxvdes+kp*(xm-hyp_robot(1))*(1-exp(-tau*rm));
        fyvBS_virtual=fyvdes+kp*(ym-hyp_robot(2))*(1-exp(-tau*rm));
    end
    
    
    fx_virtual = @(t,x) [x(2); (fxvBS_virtual-(B_model+kd)*x(2))/M];
    [T,X_virtual]=ode45(fx_virtual,tspan_ode45,[hyp_robot(1);xdot_virtual(i)]);
    [m,z] = size(X_virtual);
    x_pos_new=real(X_virtual(m,1));
    xdot_virtual(i)=real(X_virtual(m,2));
    
    
    fy_virtual = @(t,y) [y(2); (fyvBS_virtual-(B_model+kd)*y(2))/M];
    [T,Y_virtual]=ode45(fy_virtual,tspan_ode45,[hyp_robot(2);ydot_virtual(i)]);
    [m,z] = size(Y_virtual);
    y_pos_new=real(Y_virtual(m,1));
    ydot_virtual(i)=real(Y_virtual(m,2));
    hyp_robot=[x_pos_new y_pos_new];
    
    
    %---------------------------------------------------------------------
    %----------------------------------------------------------------------
    %            End of code Computing the new position of Virtual Robot
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    
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
    
    %Compute the attractive forces between the center (hypothetical robot)
    %and the actual robots
    %-----------------------------------------------------------
    %-----------------------------------------------------------
    %Electrostatic force by Coloumb's Law
    %        F=K(q1*q2)/r^2;
    %-----------------------------------------------------------
    %-----------------------------------------------------------
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
%     end
%     
%     
%     
%     %Computing resultant forces on each robot
%     for i=1:number_of_robots
        fxkVS(i)=sum(EF_x(i,1:number_of_robots))-Attractive_Force_x(i);
        fykVS(i)=sum(EF_y(i,1:number_of_robots))-Attractive_Force_y(i);
        fkVS(i)=sqrt(fxkVS(i)^2+fykVS(i)^2);
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
        x_pos(i)=xy_pos(1);
        y_pos(i)=xy_pos(2);
        
        
        %initializing x_pos_new and y_pos_new
        x_pos_new=x_pos(i);
        y_pos_new=y_pos(i);
        
        distance_from_hyp_robot=sqrt((x_pos(i)-hyp_robot(1))^2+(y_pos(i)-hyp_robot(2))^2);
        
        %---------------------------------------------------------------
        %    Forces with obstacles (Behavioral structure)
        %---------------------------------------------------------------
        
        %Force when the robot is approaching the obstacle
        if(rk(i)<ra(i))
%             disp('force with obstacle');
            fxkVS(i);
            fykVS(i);
            
              
            FxkBS(i)= fxkVS(i)+((fkVS(i)*fxkrn(i))/(rk(i)^2))*(1/rk(i)-1/ra(i));
            FykBS(i)= fykVS(i)+((fkVS(i)*fykrn(i))/(rk(i)^2))*(1/rk(i)-1/ra(i));
            
            
    
            %Force when the robot has passed the obstacle
        else
%             disp('force witout obstacle');
            FxkBS(i)= fxkVS(i)*exp(-tau*rk(i))+fxkVS(i)*(1-exp(-tau*rk(i)));
            FykBS(i)= fykVS(i)*exp(-tau*rk(i))+fykVS(i)*(1-exp(-tau*rk(i)));
            
%             fx = @(t,x) [x(2); (FxkBS(i)-(B_model+kd)*x(2))/M];
%             [T,X]=ode45(fx,tspan_ode45,[x_pos(i);xdot(i)]);
%             [m,z] = size(X);
%             x_pos_new=real(X(m,1));
%             xdot(i)=real(X(m,2));
%             
%             fy = @(t,y) [y(2); (FykBS(i)-(B_model+kd)*y(2))/M];
%             [T,Y]=ode45(fy,tspan_ode45,[y_pos(i);ydot(i)]);
%             [m,z] = size(Y);
%             y_pos_new=real(Y(m,1));
%             ydot(i)=real(Y(m,2));
            
            %             x_pos_new=x_pos(i);
            %             y_pos_new=y_pos(i);
            %
            %             SWARM_FLAG=1
            
        end
           fx = @(t,x) [x(2); (FxkBS(i)-(B_model+kd)*x(2))/M];
            [T,X]=ode45(fx,tspan_ode45,[x_pos(i);xdot(i)]);
            [m,z] = size(X);
            x_pos_new=real(X(m,1));
            xdot(i)=real(X(m,2));
            
            fy = @(t,y) [y(2); (FykBS(i)-(B_model+kd)*y(2))/M];
            [T,Y]=ode45(fy,tspan_ode45,[y_pos(i);ydot(i)]);
            [m,z] = size(Y);
            y_pos_new=real(Y(m,1));
            ydot(i)=real(Y(m,2));
        %---------------------------------------------------------------
        
        
        % % % % %         %---------------------------------------------------------------
        % % % % %         %    Forces without obstacles
        % % % % %         %---------------------------------------------------------------
        % % % % %         if(abs(fxkVS(i))>FORCE_THRESHOLD)
        % % % % %             fx = @(t,x) [x(2); (fxkVS(i)+fxkOA(i)-(B_model+kd)*x(2))/M];
        % % % % %             [T,X]=ode45(fx,tspan_ode45,[x_pos(i);xdot(i)]);
        % % % % %             [m,z] = size(X);
        % % % % %             x_pos_new=X(m,1);
        % % % % %             xdot(i)=X(m,2);
        % % % % %         end
        % % % % %
        % % % % %         if(abs(fykVS(i))>FORCE_THRESHOLD)
        % % % % %             fy = @(t,y) [y(2); (fykVS(i)+fykOA(i)-(B_model+kd)*y(2))/M];
        % % % % %             [T,Y]=ode45(fy,tspan_ode45,[y_pos(i);ydot(i)]);
        % % % % %             [m,z] = size(Y);
        % % % % %             y_pos_new=Y(m,1);
        % % % % %             ydot(i)=Y(m,2);
        % % % % %         end
        % % % % %
        % % % % %         if abs(fxkVS(i))<=FORCE_THRESHOLD &&  abs(fykVS(i))<=FORCE_THRESHOLD
        % % % % %             x_pos_new=x_pos(i);
        % % % % %             y_pos_new=y_pos(i);
        % % % % %             SWARM_FLAG=1
        % % % % %
        % % % % %         end
        % % % % %
        % % % % %         %---------------------------------------------------------------
        % % % % %
        robot{i}=[x_pos_new y_pos_new];
        %         robot_Trajectory_Vec=[robot_Trajectory_Vec;robot];
        addpoints(robotTrajectory_ZP(i),x_pos_new,y_pos_new);
        drawnow
        
    end
    virtual_robot_Trajectory_Vec=[virtual_robot_Trajectory_Vec;hyp_robot];
    
    
    %     %Drawing lines connecting all the robots
    %     if (SWARM_FLAG==1)
    %
    
    
    
    
end %end of the bigger loop

%---------------------------------------------------------------------
%         Drawing lines between each robot
%---------------------------------------------------------------------

figure(1);
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
            plot([x1 x2],[y1 y2],'r'),hold on,grid on
            
            
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
            plot([x1 x2],[y1 y2],'r'),hold on,grid on;
            
        end
        
    end
end

robots_trajectory=cell2mat(robot_Trajectory_Vec);

% figure(1);
% plot(robots_trajectory(:,1),robots_trajectory(:,2),'k-'),hold on, grid on,
% plot(robots_trajectory(:,3),robots_trajectory(:,4),'r-'),hold on, grid on,
% plot(robots_trajectory(:,5),robots_trajectory(:,6),'b-'),hold on, grid on,
% plot(robots_trajectory(:,7),robots_trajectory(:,8),'c-'),hold on, grid on,
% plot(robots_trajectory(:,9),robots_trajectory(:,10),'g-'),hold on, grid on,

% plot(robots_trajectory(:,11),robots_trajectory(:,12),'o-<'),hold on, grid on,

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



% % % % %plotting a line between the final points
% % % % for i=1:2:(number_of_robots*2)
% % % %
% % % %     x1=robots_trajectory(end,i);
% % % %     y1=robots_trajectory(end,i+1);
% % % %
% % % %     if i<(number_of_robots*2-2)
% % % %         x2=robots_trajectory(end,i+2);
% % % %         y2=robots_trajectory(end,i+3);
% % % %     else
% % % %         x2=robots_trajectory(end,mod(i+2,number_of_robots*2));
% % % %         y2=robots_trajectory(end,mod(i+3,number_of_robots*2));
% % % %     end
% % % %
% % % %     if (x1<=x2)
% % % %
% % % %         x_points=x1:0.1:x2;
% % % %     elseif (x1>x2)
% % % %         x_points=x2:0.1:x1;
% % % %         %     elseif x1==x2
% % % %         %         x_points=x1*ones(1,100);
% % % %     end
% % % %
% % % %     if (abs(x1-x2)<=1e-3)
% % % %         disp('I am here')
% % % %         if y1<y2
% % % %             y_points=y1:0.1:y2;
% % % %         elseif y1>y2
% % % %             y_points=y2:0.1:y1;
% % % %         end
% % % %
% % % %         x_points=x1*ones(1,length(y_points));
% % % %     else
% % % %
% % % %         slope=(y2-y1)/(x2-x1);
% % % %         y_points=slope*(x_points-x1)+y1;
% % % %     end
% % % %
% % % %
% % % %     figure(1)
% % % %     plot(x_points,y_points,'b--'),hold on,
% % % % end
