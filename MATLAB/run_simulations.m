function report=run_simulations()
% Compact reproduction of the revised five-controller numerical model.
% No archived trajectory files are needed; frozen gains are not retuned.
here=fileparts(mfilename('fullpath'));root=fileparts(here);
addpath(fullfile(here,'core'));
out=fullfile(root,'output');if ~exist(out,'dir'),mkdir(out);end
fixed=load(fullfile(root,'reference_results','frozen_normalization.mat'));
rr=jsondecode(fileread(fullfile(root,'reference_results','selected_parameters.json')));
names=fieldnames(rr);trajectories=struct();report=struct();rows=cell(numel(names),7);
for i=1:numel(names)
    r=rr.(names{i});theta=[r.parameters(:);zeros(7-numel(r.parameters),1)];
    fprintf('Simulating %s...\n',names{i});
    [data,info]=sr_balanced_trajectory(r.kind,theta,fixed.cfg,120,r.phi);
    assert(info.success,'Trajectory diverged.');trajectories.(names{i})=data;
    metrics=sr_balanced_metrics(r.kind,theta,fixed.cfg,120,r.phi);
    t=data(:,1);e=data(:,12:13);u=data(:,8:9);
    calculated=[sum(trapz(t,abs(e))),sum(trapz(t,t.*abs(e))),sum(trapz(t,u.^2)),sum(sum(abs(diff(u)))),sqrt(mean(e(:).^2))];
    err=max(abs(calculated-metrics(1:5))./max(1,abs(calculated)));
    assert(err<1e-10,'Metric and trajectory quadrature disagree.');
    report.(names{i})=struct('success',info.success,'quadrature_relative_error',err);
    rows(i,:)={names{i},r.seed,calculated(1),calculated(2),calculated(3),calculated(4),calculated(5)};
end
save(fullfile(out,'revised_trajectories.mat'),'-struct','trajectories');
T=cell2table(rows,'VariableNames',{'Controller','Seed','IAE','ITAE','ISU','TV','RMSE'});
writetable(T,fullfile(out,'recomputed_representative_metrics.csv'));disp(T);
a=trajectories.ASMC_QLPSO;b=trajectories.ERLSMC_PSO;
time=a(:,1);ASMC_surface=a(:,10:11);ERLSMC_surface=b(:,10:11);
save(fullfile(out,'figure7_resimulated.mat'),'time','ASMC_surface','ERLSMC_surface');
end
