% MAE 538 Design of Smart Material Systems - Course Project
% Active vibration control of a beam using smart materials
% Vinod Kumar Singla 4/22/2017
clear all;clc;close all; 
% %% Optimal placement for one patch
lb = zeros(1,2); % lower bounds on individual design variables
ub = 0.5.*ones(1,2); % upper bounds on individual design variables
opts = gaoptimset('PlotFcn',{@gaplotbestf,@gaplotstopping},'Generations',200,'TolFun',1e-6,'TolCon',1e-6);
fitnessfcn = @objF1;
nvars = 2; Lp = 0.1; % length of a piezopatch (50mm) = 1/10th beam length
nonlcon = []; 
Aeq = [-1,1];
beq = Lp;
A = [1 -1];
b = [0];
[x,fval,exitflag,output] = ga(fitnessfcn,nvars,A,b,Aeq,beq,lb,ub,nonlcon,opts)
[x,fval]'

% %% Optimal placement for two patches
% lb = zeros(1,4); % lower bounds on individual design variables
% ub = 0.5.*ones(1,4); % upper bounds on individual design variables
% opts = gaoptimset('PlotFcn',{@gaplotbestf,@gaplotstopping},'Generations',200,'TolFun',1e-6,'TolCon',1e-6);
% fitnessfcn = @objF2;
% nvars = 4; Lp = 0.1; % length of a piezopatch (50mm) = 1/10th beam length
% nonlcon = []; 
% Aeq = [-1,1,0,0;0,0,-1,1];
% beq = [Lp;Lp];
% A = [1 -1 0 0;0 0 1 -1;0 1 -1 0];
% b = [0;0;0];
% [x,fval,exitflag,output] = ga(fitnessfcn,nvars,A,b,Aeq,beq,lb,ub,nonlcon,opts)
% [x,fval]'

% Optimal placement for three patches
% lb = zeros(1,6); % lower bounds on individual design variables
% ub = 0.5.*ones(1,6); % upper bounds on individual design variables
% opts = gaoptimset('PlotFcn',{@gaplotbestf,@gaplotstopping},'Generations',200,'TolFun',1e-6,'TolCon',1e-6);
% fitnessfcn = @objF3;
% nvars = 6; Lp = 0.1; % length of a piezopatch (50mm) = 1/10th beam length
% nonlcon = []; 
% Aeq = [-1,1,0,0,0,0;0,0,-1,1,0,0;0,0,0,0,-1,1];
% beq = [Lp;Lp;Lp];
% A = [1 -1 0 0 0 0;0 0 1 -1 0 0;0 0 0 0 1 -1;...
%     0 1 -1 0 0 0;0 0 0 1 -1 0];
% b = [0;0;0;0;0];
% [x,fval,exitflag,output] = ga(fitnessfcn,nvars,A,b,Aeq,beq,lb,ub,nonlcon,opts)
% [x,fval]'