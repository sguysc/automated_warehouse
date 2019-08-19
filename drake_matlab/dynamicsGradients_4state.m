function [df, d2f, d3f] = dynamicsGradients_4state(a1, a2, a3, a4, order)
% This is an auto-generated file.
%
% See <a href="matlab: help generateGradients">generateGradients</a>. 

% Check inputs:
typecheck(a1,'DubinsPlant');
if (nargin<4) order=1; end
if (order<1) error(' order must be >= 1'); end
sizecheck(a1,[1  1]);
sizecheck(a2,[1  1]);
sizecheck(a3,[4  1]);
sizecheck(a4,[2  1]);

% Symbol table:
a3_4=a3(4);
a4_1=a4(1);
a4_2=a4(2);


% Compute Gradients:
df = sparse(4,7);
df(1,5) = -a4_1*sin(a3_4);
df(1,6) = cos(a3_4);
df(2,5) = a4_1*cos(a3_4);
df(2,6) = sin(a3_4);
df(3,6) = 1;
df(4,7) = 1;

% d2f
if (order>=2)
  d2f = sparse(4,49);
  d2f(1,33) = -a4_1*cos(a3_4);
  d2f(1,34) = -sin(a3_4);
  d2f(1,40) = -sin(a3_4);
  d2f(2,33) = -a4_1*sin(a3_4);
  d2f(2,34) = cos(a3_4);
  d2f(2,40) = cos(a3_4);
else
  d2f=[];
end  % if (order>=2)

% d3f
if (order>=3)
  d3f = sparse(4,343);
  d3f(1,229) = a4_1*sin(a3_4);
  d3f(1,230) = -cos(a3_4);
  d3f(1,236) = -cos(a3_4);
  d3f(1,278) = -cos(a3_4);
  d3f(2,229) = -a4_1*cos(a3_4);
  d3f(2,230) = -sin(a3_4);
  d3f(2,236) = -sin(a3_4);
  d3f(2,278) = -sin(a3_4);
else
  d3f=[];
end  % if (order>=3)


 % NOTEST
