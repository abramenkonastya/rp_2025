classdef Controller < matlab.System
    % Controller Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        currentTask
        rover
        controllerType
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)

    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.controllerType = 'ID';
        end

        function output = stepImpl(obj,u)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            J = 4; L = 4;
            N = J*L;
            output = zeros(1,N);
            
            if strcmp(obj.currentTask.moveType,'Empty') 
                return;
            elseif isempty(obj.currentTask.initialState)
                if obj.rover.state.isSet
                    obj.currentTask.initialState = obj.rover.state;
                end
                return;
            end

            output = obj.currentTask.FindControl(obj.rover,obj.controllerType);            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            
        end
    end
end
