classdef CartesianTask < Task
    properties
        qInit
        legStep
        kV
        kVe
        F_desired
        kF        
        vfLim     
        kP_3      
        traj
        trajd
        trajT
    end

    methods
        function obj = CartesianTask(activeLeg, controlledVars, varsValue)
            obj.moveType = 'Cartesian';
            obj.activeLeg = activeLeg;
            obj.controlledVars = controlledVars;
            obj.varsValue = varsValue;
            obj.paramsValue = [];            

            obj.kV = 10;
            obj.kVe = 0.3;            
            obj.Kp = 0;
            obj.Kd = 100;
            
            obj.F_desired = -15;
            obj.kF = 0.0001;
            obj.vfLim = 0.001;
            obj.kP_3 = 10;
            
            obj.endTolerance = [0.01 0.01];
            obj.allSet = 0;
        end

        function torque = FindControl(obj, rover, controllerType, tcur)
            state = rover.state;
            actualPosition = state.position;
            actualSpeed = state.speed;

            L = size(actualSpeed,1);
            J = size(actualSpeed,2);
            tauRequired = zeros(L,J);
            torque = tauRequired;          
            
            if ~obj.allSet
                params = obj.paramsValue;
                obj.qInit = obj.initialState.position;
                if ~isempty(params)
                    obj.kV = params(1); 
                end
                obj.Kp = ones(L,J)*obj.Kp;
                obj.Kd = ones(L,J)*obj.Kd; 

                lIndArray = obj.GetLegIndex();

                obj.legStep = zeros(L,2);

                tmax = obj.minTime;
                tpts = [0 tmax];
                timeStamps = 0:0.01:tmax;
                obj.traj = cell(1,L);
                obj.trajd = cell(1,L);
                obj.trajT = cell(1,L);

                for ind = 1 : length(lIndArray)
                    lInd = lIndArray(ind);                   
                    values = obj.varsValue{ind};                    
                    
                    obj.Kp(lInd,1:2) = 0;
                    obj.legStep(lInd,:) = values(:);
                end

                for lInd = 1 : L                       
                    treeBaseName = strcat('b',rover.legSet{lInd},'1');
                    leg = subtree(rover.tree,treeBaseName); 
                    leg.Gravity = rover.tree.Gravity;
        
                    q = obj.qInit(lInd,:);
                    
                    pointName = strcat('b',rover.legSet{lInd},'3');
                    TInit = getTransform(leg,q,pointName);
                    pInit = TInit([1 3],4);
                    pDes = pInit+obj.legStep(lInd,:)';

                    wpts = [pInit, pDes];

                    [r,v,acc,pp] = quinticpolytraj(wpts, tpts, timeStamps);
                    obj.traj(lInd) = {r};
                    obj.trajd(lInd) = {v};
                    obj.trajT(lInd) = {timeStamps};
                                     
                end
                obj.allSet = 1;
            end
                   
            switch controllerType
                case 'Zero'
                case 'ID'    
                    F = -reshape(rover.state.reactionForce, [3,4])';
                    
                    for lInd = 1 : L
                        treeBaseName = strcat('b',rover.legSet{lInd},'1');
                        leg = subtree(rover.tree,treeBaseName); 
                        leg.Gravity = rover.tree.Gravity;
            
                        q = actualPosition(lInd,:);
                        qd = actualSpeed(lInd,:);

                        pointName = strcat('b',rover.legSet{lInd},'3');

                        TActual = getTransform(leg,q,pointName);
                        pActual = TActual([1 3],4);
                        qInitLeg = obj.qInit(lInd,:);
                        TInit = getTransform(leg,qInitLeg,pointName);
                        pInit = TInit([1 3],4);
                        pDes = pInit+obj.legStep(lInd,:)';
                        
                        legName = '';
                        switch lInd
                            case 1
                                legName = 'FR';
                            case 2
                                legName = 'RR';
                            case 3
                                legName = 'RL';
                            case 4
                                legName = 'FL';
                        end
                        isActiveLeg = any(strcmp(obj.activeLeg, legName));
                        
                        fActual = F(lInd, [1 3])';

                        if isActiveLeg
                            vDes = obj.kV*(pDes-pActual);
                        else
                            pDes = pInit;
                            fDes = [0; obj.F_desired];
                            
                            force_error = fDes(2) - fActual(2);
                            vfDes = obj.kF * force_error;
                            
                            if force_error > 5
                                vfDes = vfDes + 0.001 * force_error;
                            end
                            
                            vfDes = max(min(vfDes, obj.vfLim), -obj.vfLim);
                            vxDes = obj.kV*(pDes(1) - pActual(1));
                            vDes = [vxDes; vfDes];
                        end
                        
                        Jtotal = geometricJacobian(leg,q,pointName);
                        J = Jtotal([4 6],1:2);

                        vActual = J*qd(1:2)';
                        vDes = vDes + obj.kVe*(vDes - vActual);

                        qdDes_12 = J\vDes;
                        qdDes_12 = qdDes_12';
                        
                        q3_init = obj.qInit(lInd, 3);
                        q_error_3 = q3_init - q(3);
                        qdDes_3 = obj.kP_3 * q_error_3;
                        
                        qdDes = [qdDes_12, qdDes_3, 0];
                        
                        desiredAcceleration = obj.Kd(lInd,:).*(qdDes-qd);
            
                        fInd = (lInd-1)*3+1;
                        wrench = rover.state.reactionForce(fInd:fInd+2)';
                        wrench = [0 0 0 wrench];
                        wheelName = strcat('Wh',rover.legSet{lInd});
                        fExt = externalForce(leg,wheelName,wrench,q);
            
                        T = inverseDynamics(leg,q,qd,desiredAcceleration,fExt);
                        Tmax = [200 100 50 50];
                        Tmin = -Tmax;
                        T = min(T,Tmax);
                        T = max(T,Tmin);
                        
                        tauRequired(lInd,:) = T(:); 
                    end
                case 'IDTraj'
                    F = -reshape(rover.state.reactionForce, [3,4])';
                    
                    for lInd = 1 : L
                        TR = obj.traj{lInd};
                        TV = obj.trajd{lInd};
                        timeStamps = obj.trajT{lInd};
                        t_val = tcur - obj.tStart;
                        r = interp1(timeStamps,TR',min(t_val,timeStamps(end)))';
                        v = interp1(timeStamps,TV',min(t_val,timeStamps(end)))';

                        treeBaseName = strcat('b',rover.legSet{lInd},'1');
                        leg = subtree(rover.tree,treeBaseName); 
                        leg.Gravity = rover.tree.Gravity;
            
                        q = actualPosition(lInd,:);
                        qd = actualSpeed(lInd,:);

                        pointName = strcat('b',rover.legSet{lInd},'3');

                        TActual = getTransform(leg,q,pointName);
                        pActual = TActual([1 3],4);
                        qInitLeg = obj.qInit(lInd,:);
                        
                        % ВЫЧИСЛЯЕМ TInit для текущей ноги
                        TInit = getTransform(leg,qInitLeg,pointName);
                        pInit = TInit([1 3],4);
                        
                        legName = '';
                        switch lInd
                            case 1
                                legName = 'FR';
                            case 2
                                legName = 'RR';
                            case 3
                                legName = 'RL';
                            case 4
                                legName = 'FL';
                        end
                        isActiveLeg = any(strcmp(obj.activeLeg, legName));
                        
                        fActual = F(lInd, [1 3])';

                        if isActiveLeg
                            vDes = v+obj.kV*(r-pActual);
                        else
                            pDes = pInit;
                            fDes = [0; obj.F_desired];
                            
                            force_error = fDes(2) - fActual(2);
                            vfDes = obj.kF * force_error;
                            
                            if force_error > 5
                                vfDes = vfDes + 0.001 * force_error;
                            end
                            
                            vfDes = max(min(vfDes, obj.vfLim), -obj.vfLim);
                            vxDes = obj.kV*(pDes(1) - pActual(1));
                            vDes = [vxDes; vfDes];
                        end
                       
                        Jtotal = geometricJacobian(leg,q,pointName);
                        J = Jtotal([4 6],1:2);

                        vActual = J*qd(1:2)';
                        vDes = vDes + obj.kVe*(vDes - vActual);

                        qdDes_12 = J\vDes;
                        qdDes_12 = qdDes_12';
                        
                        q3_init = obj.qInit(lInd, 3);
                        q_error_3 = q3_init - q(3);
                        qdDes_3 = obj.kP_3 * q_error_3;
                        
                        qdDes = [qdDes_12, qdDes_3, 0];

                        desiredAcceleration = obj.Kd(lInd,:).*(qdDes-qd);
            
                        fInd = (lInd-1)*3+1;
                        wrench = rover.state.reactionForce(fInd:fInd+2)';
                        wrench = [0 0 0 wrench];
                        wheelName = strcat('Wh',rover.legSet{lInd});
                        fExt = externalForce(leg,wheelName,wrench,q);
            
                        T = inverseDynamics(leg,q,qd,desiredAcceleration,fExt);
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

        function isCompleted = Completed(obj,rover,t)
            isCompleted = 0;
            if ~obj.allSet
                return;
            end
            tTask = t - obj.tStart;
            if tTask < obj.minTime
                return;
            end

            state = rover.state;
            actualPosition = state.position;
            lIndArray = obj.GetLegIndex();
            L = length(lIndArray);

            for ind = 1 : L
                lInd = lIndArray(ind);
                q = actualPosition(lInd,:); 

                treeBaseName = strcat('b',rover.legSet{lInd},'1');
                leg = subtree(rover.tree,treeBaseName); 
                leg.Gravity = rover.tree.Gravity;

                pointName = strcat('b',rover.legSet{lInd},'3');
                TActual = getTransform(leg,q,pointName);
                pActual = TActual([1 3],4);
                qInitLeg = obj.qInit(lInd,:);
                TInit = getTransform(leg,qInitLeg,pointName);
                pInit = TInit([1 3],4);
                pDes = pInit+obj.legStep(lInd,:)';                

                errorSize(lInd) = norm(pDes-pActual);
                jointErrorSize(lInd) = norm(q(3)-qInitLeg(3));
            end                       
                            
            tol = sqrt(L)*obj.endTolerance(1);
            tolJoint = sqrt(L)*obj.endTolerance(2);
            err = norm(errorSize);
            errJ = norm(jointErrorSize);

            if err < tol && errJ < tolJoint 
                disp('next')
                isCompleted = 1;
            end 
        end
    end
end