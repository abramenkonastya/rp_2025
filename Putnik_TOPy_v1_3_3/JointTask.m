classdef JointTask < Task

    properties
        qDes
        qdDes
    end

    methods
        function obj = JointTask(activeLeg,controlledVars,varsValue,minTime)
            obj.moveType = 'Joint';
            obj.activeLeg = activeLeg;
            obj.controlledVars = controlledVars;
            obj.varsValue = varsValue;
            obj.minTime = minTime;

            obj.paramsValue = [];

            obj.Kp = 500;
            obj.Kd = 250; 

            obj.endTolerance = 0.03;
            obj.allSet = 0;
        end

        function torque = FindControl(obj,rover,controllerType,t)
            
            state = rover.state;
            actualPosition = state.position;
            actualSpeed = state.speed;

            L = rover.numberOfLegs;
            J = length(obj.controlledVars{1});
            tauRequired = zeros(L,J);
            torque = tauRequired;
            
            %first iteration - preparation
            if ~obj.allSet
            obj.qDes = obj.initialState.position;
            obj.qdDes = zeros(size(obj.qDes));
            obj.Kp = ones(L,J)*obj.Kp;
            obj.Kd = ones(L,J)*obj.Kd; 
            lIndArray = obj.GetLegIndex();
            jModes = obj.controlledVars;
            params = obj.paramsValue;

        for ind = 1 : length(lIndArray)
            lInd = lIndArray(ind);
            values = obj.varsValue{ind};

            if iscell(jModes) && length(jModes) == 1 && ismatrix(jModes{1})
                thisLegJointModes = jModes{1}(ind, :);
            else
                thisLegJointModes = jModes{ind};
            end
            
            for jInd = 1 : J

                if iscell(thisLegJointModes)
                    jointMode = thisLegJointModes{jInd};
                else
                    jointMode = thisLegJointModes(jInd);
                end

                if iscell(jointMode)
                    jointMode = jointMode{1};
                end
                
                switch jointMode
                    case '0'

                    case 'P'

                        obj.qDes(lInd,jInd) = values(1,jInd);
                        obj.qdDes(lInd,jInd) = values(2,jInd);
                        if ~isempty(params)
                            obj.Kp(lInd,jInd) = params(1,jInd);
                            obj.Kd(lInd,jInd) = params(2,jInd);
                        end

                    case 'V'

                        obj.qdDes(lInd,jInd) = values(2,jInd);
                        obj.Kp(lInd,jInd) = 0;
                        if ~isempty(params)
                            obj.Kd(lInd,jInd) = params(2,jInd);
                        else
                            obj.Kd(lInd,jInd) = 300;
                        end

                end                
            end
        end
        obj.allSet = 1;
    end

            switch controllerType
                case 'Zero'
                    
                case {'ID','IDTraj'} 
            
                    for lInd = 1 : L
                        treeBaseName = strcat('b',rover.legSet{lInd},'1');
                        leg = subtree(rover.tree,treeBaseName); 
                        leg.Gravity = rover.tree.Gravity;
            
                        q = actualPosition(lInd,:);
                        qd = actualSpeed(lInd,:);
                        desiredAcceleration = obj.Kp(L,:).*(obj.qDes(lInd,:)-q)+obj.Kd(L,:).*(obj.qdDes(lInd,:)-qd);
            
                        if isnan(desiredAcceleration)
                            disp('x')
                        end
                        fInd = (lInd-1)*3+1;
                        wrench = rover.state.reactionForce(fInd:fInd+2)';
                        wrench = [0 0 0 wrench];
                        wheelName = strcat('Wh',rover.legSet{lInd});
                        fExt = externalForce(leg,wheelName,wrench,q);

                        T = inverseDynamics(leg,q,qd,desiredAcceleration);
                        Tmax = [200 100 50 50];
                        Tmin = -Tmax;
                        T = min(T,Tmax);
                        T = max(T,Tmin);
                        
                        tauRequired(lInd,:) = T(:);    
                    end
            end  
            torque = reshape(tauRequired',1,[]);
        end

        function SetInitialState(obj,state)
            obj.initialState = state;
        end

       function isCompleted = Completed(obj, rover, t)
    isCompleted = false;
    if ~obj.allSet
        return;
    end

    tTask = t - obj.tStart;
    if tTask < obj.minTime
        return;
    end


    hasVelocityMode = false;
    
    if iscell(obj.controlledVars)
        if contains(jsonencode(obj.controlledVars), '"V"')
            hasVelocityMode = true;
            fprintf('Режим V по controlledVars\n');
        end
    end

    fprintf('Итоговый режим V: %d\n', hasVelocityMode);

    lIndArray = obj.GetLegIndex();
    
    if hasVelocityMode
        fprintf('Режим V активирован\n');

        targetVx = 0;
        
        if ~isempty(obj.varsValue) && length(obj.varsValue) >= 1
            firstVars = obj.varsValue{1};
            if size(firstVars, 1) >= 2 && size(firstVars, 2) >= 4
                targetVx = firstVars(2, 4);
            end
        end
        
        bodyVelocity = rover.state.bodySpeed;
        vx = bodyVelocity(1);
        tolerance = 0.01;
        
        fprintf('Цель: %.4f м/с, Текущая: %.4f м/с, Разница: %.4f м/с\n', ...
                targetVx, vx, abs(vx - targetVx));
        
        if abs(vx - targetVx) < tolerance
            isCompleted = true;
            fprintf('Целевая скорость достигнута!\n');
        end
        
    else
        fprintf('Режим P активирован\n');

        state = rover.state;
        actualPosition = state.position;
        tolerance = 0.15;
        allJointsOK = true;
        
        for ind = 1:length(lIndArray)
            lInd = lIndArray(ind);
            
            if ind > length(obj.controlledVars) || isempty(obj.controlledVars{ind})
                continue;
            end
            
            thisLegJointModes = obj.controlledVars{ind};
            
            for jInd = 1:length(thisLegJointModes)
                if lInd > size(obj.qDes, 1) || jInd > size(obj.qDes, 2)
                    allJointsOK = false;
                    continue;
                end
                
                posError = abs(obj.qDes(lInd, jInd) - actualPosition(lInd, jInd));
                if (posError >= tolerance)
                    allJointsOK = false;
                end

                if (tTask >= 0.5)
                    allJointsOK = true;
                end
            end
        end
        
        if allJointsOK
            isCompleted = true;
            fprintf('Задача завершена: все суставы достигли целевого положения\n');
        end
    end
end
    end
end