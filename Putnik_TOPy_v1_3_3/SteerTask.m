classdef SteerTask < Task

    properties
        subTasks
        currentSubTaskIndex

        qInit   
    end

    methods
        function obj = SteerTask(controlledVars,varsValue,minTime)

            obj.moveType = 'Steer';
            obj.varsValue = varsValue{1};
            obj.controlledVars = controlledVars;
            obj.minTime = minTime;

            obj.endTolerance = 0.12;
            obj.allSet = 0;

            obj.currentSubTaskIndex = 1;
        end

        function torque = FindControl(obj,rover,controllerType,t)

            if ~obj.allSet
                activeLeg = {'FR','RR','RL','FL'};
                eachJointControlledVars = {'0','0','P','0'};
 
                state = rover.state;
                actualSpeed = state.speed;
                obj.qInit = state.position;
    
                L = size(actualSpeed,1);                

                subVarsValue = cell(1,L);
                jointControlledVars = cell(1,L);
                for lInd = 1 : L                       
                    subVarsValueJoint = [0 0 obj.varsValue(1,1) 0; 0 0 0 0];
                    subVarsValue{lInd} = subVarsValueJoint;
                    jointControlledVars{lInd} = eachJointControlledVars;
                end       
                
                mTime = obj.minTime;
                steerTask = JointTask(activeLeg,jointControlledVars,subVarsValue,mTime); 
                steerTask.paramsValue = [0 0 1000 0; 0 0 50 0];
    
                obj.subTasks{1} = steerTask;

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