function build_accelerator()
% Optional MATLAB Coder JIT accelerator, compiled on the reviewer's machine.
% Simulations/figures work without this function or MATLAB Coder.
here=fileparts(mfilename('fullpath'));root=fileparts(here);core=fullfile(here,'core');addpath(core);
if isempty(which('coder.config')),error('MATLAB Coder is required only for this optional acceleration step. Native MATLAB remains available.');end
build=fullfile(root,'output','codegen');if ~exist(build,'dir'),mkdir(build);end
clear sr_balanced_metrics_mex
cg=coder.config('mex');cg.EnableJIT=true;cg.GenerateReport=false;
codegen('-config',cg,'sr_balanced_metrics','-args',{0,zeros(7,1),zeros(17,1),120,.5},'-d',build,'-o',fullfile(core,'sr_balanced_metrics_mex'));
fixed=load(fullfile(root,'reference_results','frozen_normalization.mat'));
rr=jsondecode(fileread(fullfile(root,'reference_results','selected_parameters.json')));
names=fieldnames(rr);checks=struct();
for i=1:numel(names)
    r=rr.(names{i});theta=[r.parameters(:);zeros(7-numel(r.parameters),1)];
    native=sr_balanced_metrics(r.kind,theta,fixed.cfg,120,r.phi);
    fast=sr_balanced_metrics_mex(r.kind,theta,fixed.cfg,120,r.phi);
    err=max(abs(native-fast)./max(1,abs(native)));assert(err<1e-8,'Native and accelerated evaluators disagree.');
    checks.(names{i})=err;
end
fid=fopen(fullfile(root,'output','accelerator_verification.json'),'w','n','UTF-8');
fwrite(fid,jsonencode(checks,'PrettyPrint',true),'char');fclose(fid);
fprintf('Locally built accelerator agrees with the native evaluator for all five selected controllers.\n');
end
