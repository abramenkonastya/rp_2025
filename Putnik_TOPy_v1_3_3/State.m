classdef State
    %UNTITLED7 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        bodyPosition
        bodySpeed
        bodyOrientation
        bodyAngularSpeed

        position
        speed
        reactionForce
        contactPts
        isSet
    end

    methods
        function obj = State(L,J)
            %State Construct an instance of this class
            %   Detailed explanation goes here
            obj.position = zeros(L,J);
            obj.speed = zeros(L,J);
            obj.reactionForce = zeros(L*3,1);  
            obj.bodyPosition = zeros(3,1);
            obj.bodySpeed = zeros(3,1);
            obj.bodyOrientation = zeros(3,1);
            obj.bodyAngularSpeed = zeros(3,1);
            obj.contactPts = zeros(L,3);
            obj.isSet = 0;
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end