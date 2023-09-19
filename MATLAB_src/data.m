%% EECE 244 - Week 6 IBM Data
%
% run a section with 'ctrl+enter'

%% clear workspace, close windows
close all;
clear all;
clc;

%%
% to preview: Tdata. ->> 'tab'
Tdata = readtable('data-0002-S2.csv');
t = Tdata(:,1);
dat = Tdata(:,2);
%% dynamically access with TData.(names{1})(1:10) --> get 1st ten things from column 1
names = fieldnames(Tdata);

%% remove NaNs
for n = 1:(length(names) - 3) % get rid of properties, row, & variable fields
   temp = Tdata.(names{n}); % extract column 'n'
   if sum(isnan(temp))>0 % then NaN
       Dat.(names{n}) = temp(~(isnan(temp)));
   else 
       Dat.(names{n}) = temp;
   end
end

%% plot S1 Data
subplot(1,1,1);
yyaxis left
plot(Dat.timestamp*1E-3, Dat.accel*10, 'b')
%[~, locs_Rwave] = findpeaks(dat, 'MinPeakHeight', 1.85, 'MinPeakDistance', 1800);
%plot(t(locs_Rwave), dat(locs_Rwave), 'rv', 'MarkerFaceColor', 'r')
hold on; grid on;
ylabel('Accel Diff')
yyaxis right
plot(Dat.timestamp*1E-3, Dat.HR, '-r')
plot(Dat.timestamp*1E-3, Dat.o2_, '-k')
xlabel('Time (s)')
ylabel('BPM/O2%')
title('bioSense Data')
legend('Accel-Diff', 'HR [BPM]', 'O2%', 'location', 'southeast')
