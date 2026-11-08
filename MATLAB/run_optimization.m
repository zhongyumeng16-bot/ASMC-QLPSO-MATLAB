function results=run_optimization(tasks,seeds,population,iterations)
% Optional retuning. Defaults reproduce the full frozen 5-method/20-seed study.
% run_optimization(1,1003) reruns the QLPSO representative's search.
% Results go to output; reference results are never overwritten.
if nargin<1,tasks=1:5;end
if nargin<2,seeds=1001:1020;end
if nargin<3,population=25;end
if nargin<4,iterations=30;end
assert(all(ismember(tasks,1:5)) && population>=1 && iterations>=1);
here=fileparts(mfilename('fullpath'));root=fileparts(here);addpath(fullfile(here,'core'));
out=fullfile(root,'output','optimization');if ~exist(out,'dir'),mkdir(out);end
fixed=load(fullfile(root,'reference_results','frozen_normalization.mat'));
base=sr_balanced_metrics(2,[100;100;5;10;4;2;0],fixed.cfg,120,0);
assert(max(abs(base([1,5,3,4])-fixed.normalizers)./max(1,fixed.normalizers))<1e-10);
if exist('sr_balanced_metrics_mex','file')~=3
    fprintf('Using native MATLAB. Full retuning is slow; build_accelerator is optional.\n');
end
results=cell(numel(tasks),numel(seeds));
for i=1:numel(tasks)
    for j=1:numel(seeds)
        fprintf('Optimizing task %d seed %d (%d evaluations)...\n',tasks(i),seeds(j),population*(iterations+1));
        result=sr_tune_balanced(tasks(i),seeds(j),fixed.cfg,fixed.normalizers,fixed.weights,population,iterations);
        results{i,j}=result;save(fullfile(out,sprintf('task%d_seed%d.mat',tasks(i),seeds(j))),'result');
    end
end
cfg=fixed.cfg;normalizers=fixed.normalizers;weights=fixed.weights;
save(fullfile(out,'rerun_results.mat'),'results','cfg','normalizers','weights','tasks','seeds','population','iterations');
end
