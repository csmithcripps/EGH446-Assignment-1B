% Startup Model
close_system('EGH446',0);
 
homedir = pwd; 
addpath( genpath(strcat(homedir,[filesep,'rvctools'])));
addpath( genpath(strcat(homedir,[filesep, 'slprj'])));
addpath( genpath(strcat(homedir,[filesep,'matfiles'])));
 
cd rvctools;
startup_rvc;

cd MRTB;
startMobileRoboticsSimulationToolbox;


cd(homedir);

open_system('EGH446'); %differential robot