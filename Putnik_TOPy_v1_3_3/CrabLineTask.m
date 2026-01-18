classdef CrabLineTask < Task

    properties
        totalStateDes
        qInit

        kV
        kPath
        kVe
        vLim
        s

        path
        pathType
        lastKeyPointInd
    end

    methods
        function obj = CrabLineTask(controlledVars,varsValue,minTime)
            obj.moveType = 'CrabLine';
            obj.varsValue = varsValue{1};
            obj.minTime = minTime;
            obj.pathType = 'L';
            obj.lastKeyPointInd = 1;


            %check!
            obj.kV = 25;
            obj.Kd = 200;
            obj.Kp = 500;     

            obj.kPath = 5;
            obj.kVe = 0.5;
            obj.vLim = 3;              

            obj.endTolerance = [0.01 15];            
            obj.allSet = 0;
        end

        function [torque] = FindControl(obj,rover,controllerType,tcur)

            state = rover.state;
            actualPosition = state.position;
            actualSpeed = state.speed;

            LN = size(actualSpeed,1);
            JN = size(actualSpeed,2);
            tauRequired = zeros(LN,JN);
            torque = tauRequired;          

            if ~obj.allSet
                obj.qInit = obj.initialState.position;

                obj.Kd = ones(LN,JN)*obj.Kd;
                obj.Kp = ones(LN,JN)*obj.Kp;
                obj.Kp(:,3) = 300;
                obj.Kd(:,3) = 15;
                obj.Kp(:,4) = 0;   
                obj.Kd(:,4) = 200;  

                p0 = state.bodyPosition;
                p0(3) = state.bodyOrientation(3);

                switch obj.pathType
                    case 'L'

                        alpha = obj.varsValue(1,3);
                        obj.path = CreateTrajectoryLine(p0,obj.varsValue(1,2),obj.varsValue(1,1),alpha,10);

                end

                obj.totalStateDes = [obj.path.x(end) obj.path.y(end)];
                obj.s = sign(obj.path.v(1));
                obj.allSet = 1;
            end  

            controllerType = 'ID';
            switch controllerType
                case 'Zero'
                    
                case {'ID';'IDTraj'}
                    phi = state.bodyOrientation(3);
                    V = state.bodySpeed;
                    Vactual = dot(V(1:2),[cos(phi) sin(phi)]);                    

                    keyPointInd = obj.path.FindClosestPointInd(state.bodyPosition,phi,obj.lastKeyPointInd,obj.s);

                    keyPoint = obj.path.GetPoint(keyPointInd);
                    vDes = keyPoint.v;
                    accLim = 30;
                    dt = 0.002;
                    vDes = min([vDes,Vactual+accLim*dt]); %XXX
                    vDes = max([vDes,Vactual-accLim*dt]);

                    sV = obj.s;

                    if (keyPointInd == length(obj.path.x))
                        error = [keyPoint.x-state.bodyPosition(1) keyPoint.y-state.bodyPosition(2)];

                        prevPoint = obj.path.GetPoint(keyPointInd-1); 
                        rV = [keyPoint.x-prevPoint.x;keyPoint.y-prevPoint.y];
                        rV = rV*obj.s;
                        alpha = atan2(rV(2),rV(1));
                        v1 = [cos(alpha) sin(alpha)];                        

                        errAlong = dot(v1,error);
                        r = norm(errAlong);
                        vDesLeft = obj.kPath*r;                        
                        vDesNorm = min([obj.vLim,norm(vDes),vDesLeft]); 

                        sV = sign(errAlong);
                        vDes = vDesNorm*sV;

                    else
                        nextPoint = obj.path.GetPoint(keyPointInd+1); 
                        rV = [nextPoint.x-keyPoint.x;nextPoint.y-keyPoint.y];
                        rV = rV*obj.s;
                        alpha = atan2(rV(2),rV(1));
                    end

                    vDes = vDes + obj.kVe.*(vDes-Vactual);
                    
                    [R,O] = obj.path.FindArcByTwoPoints(keyPointInd,state.bodyPosition,obj.s);                    
                    wWheelDes = rover.FindWheelDesiredSpeed(vDes,O); 
                    
                    dDes = obj.path.phi(1)-alpha;
                    deltaDes = -[1 1 1 1]*dDes;
        
                    phiDes = obj.path.phi(keyPointInd);        
                    phiError = phiDes-phi;  
                    K = [1 -1 -1 1]*sV; 
                    deltaDes = deltaDes+K.*phiError;   

                    deltaCorrection = obj.path.FindDeltaForLine(keyPointInd,state.bodyPosition);
                    kCorr = -[1 0 0 1]*sV;
                    deltaDes = deltaDes + kCorr*deltaCorrection;

                    qdDes = zeros(LN,JN);
                    qdDes(:,4) = wWheelDes(:); 

                    qdF = qdDes(1,4) + qdDes(4,4);
                    qdR = qdDes(2,4) + qdDes(3,4);
                    qdM = (qdF+qdR)/2;
                    if abs(qdM) > 0.01 
                        ratF = 1+(qdF-qdM)/qdM;
                        qdDes(1,4) = qdDes(1,4)/ratF;
                        qdDes(4,4) = qdDes(1,4)/ratF;
                        ratR = 1+(qdR-qdM)/qdM;
                        qdDes([2 3],4) = qdDes([2 3],4)/ratR;
                    end                    

                    qDes = obj.qInit;
                    qDes(:,3) = deltaDes(:);

                    for lInd = 1 : LN
                        treeBaseName = strcat('b',rover.legSet{lInd},'1');
                        leg = subtree(rover.tree,treeBaseName); 
                        leg.Gravity = rover.tree.Gravity;
            
                        q = actualPosition(lInd,:);
                        qd = actualSpeed(lInd,:);

                        desiredAcceleration = obj.Kd(lInd,:).*(qdDes(lInd,:)-qd)+obj.Kp(lInd,:).*(qDes(lInd,:)-q);
            
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
                case 'HybridTraj'  
            end    
            obj.lastKeyPointInd = keyPointInd;
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

    pointX = 1.8250879352;
    xThreshold = 0.15;
    
    currentX = rover.state.bodyPosition(1);
    distanceX = abs(currentX - pointX);
    
    if (currentX - pointX) > 0 
    if distanceX > xThreshold
        isCompleted = true;
        return;
    end
    end

    p = rover.GetTotalState(); 
    error = obj.totalStateDes - p(1:2)'; 
    
    phi = obj.path.phi(end);
    v1 = [cos(phi) sin(phi)];
    errorSize = dot(v1,error);             
    
    tol = obj.endTolerance(1);
    err = norm(errorSize);

    if err < tol 
        isCompleted = true;
    end 
end

    end
end