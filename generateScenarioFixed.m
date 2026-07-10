function scenario = generateScenarioFixed()

scenario.goal = [12; 0];
%-----------------------------------------------------------------------------
% Static obstacles
%-----------------------------------------------------------------------------
scenario.staticObs = struct([]);

scenario.staticObs(1).pos = [3;  1.2];   scenario.staticObs(1).radius = 0.25;
scenario.staticObs(2).pos = [5; -1.2];   scenario.staticObs(2).radius = 0.25;
scenario.staticObs(3).pos = [7;  1.2];   scenario.staticObs(3).radius = 0.25;
scenario.staticObs(4).pos = [9; -1.2];   scenario.staticObs(4).radius = 0.25;
scenario.staticObs(5).pos = [11; 1.0];   scenario.staticObs(5).radius = 0.25;
scenario.staticObs(6).pos = [6.5;  0.6]; scenario.staticObs(6).radius = 0.22;
scenario.staticObs(7).pos = [6.5; -0.6]; scenario.staticObs(7).radius = 0.22;

%-----------------------------------------------------------------------------
% Dynamic obstacles
%-----------------------------------------------------------------------------
scenario.dynamicObs = struct([]);

scenario.dynamicObs(1).pos = [4;  2.5];  scenario.dynamicObs(1).vel = [0.0; -0.6]; scenario.dynamicObs(1).radius = 0.18;
scenario.dynamicObs(2).pos = [8; -2.5];  scenario.dynamicObs(2).vel = [0.0;  0.6]; scenario.dynamicObs(2).radius = 0.18;
scenario.dynamicObs(3).pos = [2;  0.2];  scenario.dynamicObs(3).vel = [0.4;  0.0]; scenario.dynamicObs(3).radius = 0.16;
scenario.dynamicObs(4).pos = [10; -0.2]; scenario.dynamicObs(4).vel = [-0.5; 0.0]; scenario.dynamicObs(4).radius = 0.16;
scenario.dynamicObs(5).pos = [6;  2.5];  scenario.dynamicObs(5).vel = [-0.3; -0.5]; scenario.dynamicObs(5).radius = 0.17;
scenario.dynamicObs(6).pos = [6.5; 0];   scenario.dynamicObs(6).vel = [0.2;  0.3]; scenario.dynamicObs(6).radius = 0.15;

end