function i = droneguard_missionend(mission_state, soc)
%DRONEGUARD_MISSIONEND  Index of the sample where the mission is over.
%
%   Either the drone got home, or the pack hit empty. Anything the simulation
%   does after that is meaningless - a landed drone is not still drawing hover
%   current, and a dead battery cannot keep flying - so plots and summaries
%   stop here rather than showing a tail that is an artefact of the model.

iHome = find(mission_state >= 2, 1);
iDead = find(soc <= 0, 1);
i = min([iHome; iDead; numel(soc)]);
end
