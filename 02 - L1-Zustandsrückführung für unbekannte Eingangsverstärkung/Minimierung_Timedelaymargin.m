function [x,fval,exitflag,output,population,score] = Minimierung_Timedelaymargin(nvars,lb,ub)
% This is an auto generated MATLAB file from Optimization Tool.

% Start with the default options
options = gaoptimset;
% Modify options setting
options = gaoptimset(options,'Display', 'off');
options = gaoptimset(options,'PlotFcns', { @gaplotbestf });
[x,fval,exitflag,output,population,score] = ...
ga(@Timedelaymargin,nvars,[],[],[],[],lb,ub,[],[],options);
