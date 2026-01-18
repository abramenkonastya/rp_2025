classdef CrabTask < Task

    properties
        subTasks
        currentSubTaskIndex

        qInit
        yawInit
        turnAngle
        kV        
    end

    methods
        function obj = CrabTask(controlledVars,varsValue,minTime)

            obj.moveType = 'Crab';
            obj.varsValue = varsValue{1};
            obj.controlledVars = controlledVars;
            obj.minTime = minTime;

            obj.kV = 10;
            obj.endTolerance = 0.003;
            obj.allSet = 0;

            obj.currentSubTaskIndex = 1;
        end

        function torque = FindControl(obj,rover,controllerType,t)

            if ~obj.allSet                
 
                state = rover.state;               

                alpha = obj.varsValue(1,3);                   
                
                varsValue = {[alpha; 0]};
                steerControlledVars = {'P'};
                mTime = 0.1;
                taskTurnWheels = SteerTask(steerControlledVars,varsValue,mTime);
                taskTurnWheels.paramsValue = [0 0 1000 0; 0 0 50 0];
    
                crabVarsValue{1} = obj.varsValue;
                taskCrabLine = CrabLineTask(obj.controlledVars,crabVarsValue,obj.minTime);
    
                obj.subTasks{1} = taskTurnWheels;
                obj.subTasks{2} = taskCrabLine;

                obj.SetInitialState(state);
                obj.SetStartTime(t);
                obj.allSet = 1;
            end
            
            subTask = obj.GetCurrentSubTask();
            torque = subTask.FindControl(rover,controllerType,t);
        end

        function SetStartTime(obj,t)
            obj.tStart = t;
            obj.subTasks{1}.tStart = t;
        end      

        function SetInitialState(obj,state)
            obj.initialState = state;
            obj.subTasks{1}.initialState = state;
        end

        function SetParameters(obj,params)
            if ~isempty(params)
                L = length(obj.subTasks);
                obj.kV = params(1); 
                for ind = 1 : L
                    obj.subTasks{ind}.paramsValue = [obj.kV];
                end
            end            
        end        

        function subTask = GetCurrentSubTask(obj)
            subTask = obj.subTasks{obj.currentSubTaskIndex};
        end

        function isCompleted = Completed(obj,rover,t)
            
            isCompleted = 0;
            if ~obj.allSet
                return;
            end
        
            currentTask = obj.subTasks{obj.currentSubTaskIndex};
            if currentTask.Completed(rover,t)
                if obj.currentSubTaskIndex == length(obj.subTasks)
                    isCompleted = 1;
                    return;
                end
                obj.currentSubTaskIndex = obj.currentSubTaskIndex+1;
                obj.subTasks{obj.currentSubTaskIndex}.tStart = t;
                obj.subTasks{obj.currentSubTaskIndex}.minTime = obj.minTime;
                obj.subTasks{obj.currentSubTaskIndex}.initialState = rover.state;
            end
        end
    end
end