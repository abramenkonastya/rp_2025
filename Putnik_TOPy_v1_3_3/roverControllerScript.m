function [output] = roverControllerScript(t)
%roverControllerScript 
%   Output - joint torques, [hip,knee,steer,wheel] - for all legs, in order
%   [FR,RR,RL,FL]
J = 4; L = 4;
N = J*L;
output = zeros(1,N);

global currentTask rover

if strcmp(currentTask.moveType,'Empty') 
    return;
elseif isempty(currentTask.initialState)
    if rover.state.isSet
        currentTask.SetInitialState(rover.state);
%         currentTask.initialState = rover.state;
    end
    return;
end

controllerType = 'IDTraj';
output = currentTask.FindControl(rover,controllerType,t);

end