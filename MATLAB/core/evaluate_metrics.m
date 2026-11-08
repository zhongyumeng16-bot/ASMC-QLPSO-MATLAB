function metrics=evaluate_metrics(kind,theta,cfg,rho,phi)
% Portable default: native MATLAB. Optional locally rebuilt MEX accelerator.
if exist('sr_balanced_metrics_mex','file')==3
    metrics=sr_balanced_metrics_mex(kind,theta,cfg,rho,phi);
else
    metrics=sr_balanced_metrics(kind,theta,cfg,rho,phi);
end
end
