clc;
clear;

%close all;
clf

% Define maze boundaries
xmin = -10;
xmax = 10;
ymin = -10;
ymax = 10;

% Define obstacles as inequalities in the form x <= f(y)
obstacles = {
@(x,y) (x >= -8.5 && x <= 7.5) && (y >= -8 && y <= -7.5);
@(x,y) (x >= 1 && x <= 1.5) && (y >= -10 && y <= -8);
@(x,y) (x >= 7 && x <= 7.5) && (y >= -7.5 && y <= -5.5);
@(x,y) (x >= -3 && x <= -2.5) && (y >= -7.5 && y <= -5.5);
@(x,y) (x >= -3 && x <= 5) && (y >= -5.5 && y <= -5);
@(x,y) (x >= -10 && x <= -8) && (y >= -5.5 && y <= -5);
@(x,y) (x >= -10 && x <= -5) && (y >= -5.5 && y <= -5);
@(x,y) (x >= -5.5 && x <= -5) && (y >= -5 && y <= -2);
@(x,y) (x >= -10 && x <= -3) && (y >= 0.5 && y <= 1);
@(x,y) (x >= -3 && x <= -2.5) && (y >= -2 && y <= 1);
@(x,y) (x >= -3 && x <= 7.5) && (y >= -2.5 && y <= -2);
@(x,y) (x >= 0 && x <= 7.5) && (y >= 0.5 && y <= 1);
@(x,y) (x >= 0 && x <= 0.5) && (y >= 1 && y <= 4);
@(x,y) (x >= -3 && x <= 10) && (y >= 3.5 && y <= 4);
@(x,y) (x >= -10 && x <= -6) && (y >= 3.5 && y <= 4);
@(x,y) (x >= -7.5 && x <= 7.5) && (y >= 7.5 && y <= 8);
@(x,y) (x >= -1.5 && x <= -1) && (y >= 8 && y <= 10);
@(x,y) (x >= 7 && x <= 7.5) && (y >= 6 && y <= 7.5);
@(x,y) (x >= 4.5 && x <= 5) && (y >= 4 && y <= 6);
@(x,y) (x >= -3 && x <= 1) && (y >= 5.5 && y <= 6);
@(x,y) (x >= -8 && x <= -7.5) && (y >= 6 && y <= 8);
@(x,y) (x >= -8 && x <= -2) && (y >= 5.5 && y <= 6);
@(x,y) x <= -10;
@(x,y) x >= 10;
@(x,y) y <= -10;
@(x,y) y >= 10;
};

% Define start and end points
start_point = [0, -10];
end_point = [0, 10];

% Visualization parameters
resolution = 0.1; % Resolution for plotting obstacles
marker_size = 10;

% Plot maze boundaries
figure(1);
hold on;
plot([-1, xmin, xmin, -1], [ymax, ymax, ymin, ymin], 'k-', 'LineWidth', 2);
plot([1, xmax, xmax, 1], [ymin, ymin, ymax, ymax], 'k-', 'LineWidth', 2);

% Plot obstacles
x_range = xmin:resolution:xmax;
y_range = ymin:resolution:ymax;
for i = 1:length(obstacles)
    for j = 1:length(x_range)
        for k = 1:length(y_range)
            if obstacles{i}(x_range(j), y_range(k)) && x_range(j)>-10 &&x_range(j)<10 && y_range(k)>-10 && y_range(k)<10
                plot(x_range(j), y_range(k), 'r.', 'MarkerSize', 10);
            end
        end
    end
end

% Plot start and end points
plot(start_point(1), start_point(2), 'go', 'MarkerSize', marker_size, 'LineWidth',2); % Green circle for start
plot(end_point(1), end_point(2), 'ro', 'MarkerSize', marker_size, 'LineWidth', 2); % Red circle for end

% Adjust axes and grid
axis equal;
grid on;
xlim([xmin, xmax]);
ylim([ymin, ymax]);
title('COSMOS 24 Cluster 11 Maze');

% code to detect obstacles
function forwardObstacle = checkForwardObstacle(Pose, obstacles, heading, r)
  forwardObstacle = false;
  for i = 1:length(obstacles)
      % check if there is an obstacle distance ʻrʻ in front of robot
      if obstacles{i}(Pose(1) + r * cos(heading), Pose(2) + r * sin(heading))
          forwardObstacle = true;
          break;
      end
  end
end

function leftDiagObstacle = checkLeftDiagObstacle(Pose, obstacles, heading, r)
   leftDiagObstacle = false;
   for i = 1:length(obstacles)
       % check if there is an obstacle distance ʻrʻ to left of robot
       if obstacles{i}(Pose(1) + r * cos(heading + pi/3), Pose(2) + r * sin(heading + pi/3))
           leftDiagObstacle = true;
           break;
       end
   end
end

function rightDiagObstacle = checkRightDiagObstacle(Pose, obstacles, heading, r)
   rightDiagObstacle = false;
   % check if there is an obstacle distance ʻrʻ to right of robot
   for i = 1:length(obstacles)
       if obstacles{i}(Pose(1) + r * cos(heading - pi/3), Pose(2) + r * sin(heading - pi/3))
           rightDiagObstacle = true;
           break;
       end
   end
end

% initializing values
vel = []; % velocity
Pose = zeros(3,100); % x and y position and angle
Pose(1:3,1) = [[0];[-10];[pi/2]]; % defining start point and angle
endPoint = [end_point(1), end_point(2)]; % defining desired end point

% returns Pose, which is a matrix of all x, y, and theta values
function Pose = move(endPoint, Pose, vel, obstacles)
    h = 0.05; % step size
    p = .5; % proportional coefficient for x and y velocity
    pa = 4; % proportional coefficient for angular velocity
    
    b = .4; % bias for turning when robot hits obstacle at straight angle
    bx = b; % x bias
    by = b; % y bias

    R_obs = .75; % obstacle radius
    R_safe = 1; % safe radius
    target = 1; % set mode to target
    avoidance = 0; % set avoidance mode off
    obs = [0,0]; % initialize obstacle

    k = 1; % counter
    while true
        % turn on avoidance mode if robot senses an obstacle 
        if avoidance == 0 && checkForwardObstacle(Pose(1:2,k), obstacles, Pose(3,k), R_obs)
            
            % set obstacle to point in front of robot
            obs(1) = Pose(1,k) + h * cos(Pose(3,k));
            obs(2) = Pose(2,k) + h * sin(Pose(3,k));

            % if wall to right of robot, set x bias to turn left
            if checkRightDiagObstacle(Pose(1:2,k), obstacles, Pose(3,k), R_obs) && ~checkLeftDiagObstacle(Pose(1:2,k), obstacles, Pose(3,k), R_obs)
                bx = b;
                disp("right")
            % if wall to left of robot, set x bias to turn right
            elseif ~checkRightDiagObstacle(Pose(1:2,k), obstacles, Pose(3,k), R_obs) && checkLeftDiagObstacle(Pose(1:2,k), obstacles, Pose(3,k), R_obs)
                bx = -b;
                disp("left")
            end

            % if wall is vertical, set y bias to turn up
            for i = 1:length(obstacles)
                if obstacles{i}(obs(1), (obs(2)+1))
                    by = -b;
                else
                    by = b;
                end
            end
        end

        % set distance from obstacle
        distance = ((Pose(1,k) - obs(1))^2 + (Pose(2,k) - obs(2))^2)^(1/2);

        % set target or obstacle mode
        if target == 1 && distance < R_obs % if hits obstacle, switch to avoidance controller
            target = 0;
            avoidance = 1;
            disp("obstacle");
        elseif avoidance == 1 && distance > R_safe % if avoids obstacle, switch to target controller
            target = 1;
            avoidance = 0;
            disp("safe");
        end

        % stop running after certain number of steps      
        if k == 5000
            Pose(:,k)
            disp("give up");
            break
        end

        % stop running if within 1 of desired end point
        if ((Pose(1,k) - endPoint(1))^2 + (Pose(2,k) - endPoint(2))^2)^(1/2) < 1
            disp("goal reached");
            break
        end

        % avoidance controller
        if avoidance == 1
            x = Pose(1,k);
            y = Pose(2,k);

            % V = -((obs(1)-x)^2 + (obs(2)-y)^2) potential function
            dVdx = -2 * (obs(1) - x); % derivative with respect to x
            dVdy = -2 * (obs(2) - y); % derivative with respect to y

            vel(1,k) = 3 * p * dVdx - bx;
            vel(2,k) = 3 * p * dVdy - by;
            vel(3,k) = 3 * bx;

            Pose(3,(k+1)) = Pose(3,k) + h * vel(3,k);
            Pose(1,(k+1)) = Pose(1,k) + h * vel(1,k);%* cos(Pose(3,k));
            Pose(2,(k+1)) = Pose(2,k) + h * vel(2,k);%* sin(Pose(3,k));

        elseif target == 1
            % motion equations
            heading = atan2((endPoint(2) - Pose(2,k)),(endPoint(1) - Pose(1,k)));

            vel(1:2,k) = p * abs((endPoint(:) - Pose(1:2,k)));
            vel(3,k) = pa * (heading - Pose(3,k));

            Pose(3,(k+1)) = Pose(3,k) + h * vel(3,k);
            Pose(1,(k+1)) = Pose(1,k) + h * vel(1,k) * cos(Pose(3,k));
            Pose(2,(k+1)) = Pose(2,k) + h * vel(2,k) * sin(Pose(3,k));
        end
        k = k + 1;
    end
end

% calling the function
Pose1 = move([-3,-4], Pose, vel, obstacles);
Pose2 = move([6,-1], Pose1(:,end), vel, obstacles);
Pose3 = move([-4,5], Pose2(:,end), vel, obstacles);
Pose4 = move([6,7], Pose3(:,end), vel, obstacles);
Pose5 = move([9,9], Pose4(:,end), vel, obstacles);
Pose6 = move(endPoint, Pose5(:,end), vel, obstacles);
Pose = [[Pose1],[Pose2],[Pose3],[Pose4],[Pose5],[Pose6]];
% number of steps
k = length(Pose);

% plot(Pose(1,:),Pose(2,:),LineWidth=2,Color="blue")

% % create robot
p = plot(Pose(1,1:k),Pose(2,1:k),"o", "MarkerFaceColor", "blue",'MarkerSize',15.0);
% create marker to indicate forward direction
f = plot(Pose(1,1:k),Pose(2,1:k),".", "Color", "blue",'MarkerSize',15.0);

d = .75; % directional marker appears length ʻdʻ in front of robot
for j = 2:k
  % position of robot
  p.XData = Pose(1,j);
  p.YData = Pose(2,j);
  % marker for forward direction - changes according to angle vector
  f.XData = Pose(1,j) + cos(Pose(3,j)) * d;
  f.YData = Pose(2,j) + sin(Pose(3,j)) * d;
  drawnow
end