%% STUDENT TODO: Need to fill this function that measures distances 
%% between states. This function should be able to take in a vector of states
%% and return their distance to a single state.
%% (a,b) -> can both be a single state  ==> d is a scalar
%% (a,b) -> "a" can be a vector of states, "b" a single state  ==> d is a vector of distances 
% For angle differences, use the function "angdiff" which returns an angle
% from -180 to +180 degrees.
%% COMMENTS:
%  a) You can choose any distance metric that would make sense. For example, the 
%simplest one would just be the euclidean distance between the states 
% (accounting for angle differences correctly), but this does not reflect the 
%actual motion needed.
%  b) You can try to come up with a heuristic (admissible, hopefully) that works 
%well in practice - A nice simple one might be to have a lower cost for the points 
%within a cone of (-maxsteerangl, +maxsteerangl) both in front of and behind the 
% car and high cost otherwise. Of course, these costs also have to depend on 
%the x,y distances as well. There could also be more complex ones where you 
%can fit a function based on the cost of actual forward propagations you do. 
%  c) You could also use the length of the car (= 7.5 cells in the current setup, 
% check simulate_carBot.m) to better inform your distance metric. 
function d = dist3d(a, b)
% INPUT:
% a - set of input poses [x,y,theta] => (N x 3), N can be >= 1
% b - single pose [x,y,theta] => (1 x 3)
% OUTPUT:
% d - distance between states (scalar or (Nx1) vector)

sizeT = size(a);
distList = [];

%[distance(qn, q)]^2 = (qn.x-q.x)^2 + (qn.y-q.y)^2 + A*min[(qn.theta-q.theta), (qn.theta-q.theta+2*PI), (qn.theta-q.theta-2*PI)]^2   --------------------- (1)

%minAngle = 7.5*(min((a(3)-b(3)), min((a(3)-b(3)+2*pi), (a( 3)-b(3)-2*pi))))^2;
%minAngle = abs(angdiff(a(3), b(3)));
thetaThres = abs(angdiff(a(3), b(3)));
minAngle = 0;
if thetaThres > 100
    minAngle = 1000;
else
    minAngle = (a(3)-b(3))^2;
end

d = sqrt((a(1)-b(1))^2 + (a(2)-b(2))^2 + minAngle);


