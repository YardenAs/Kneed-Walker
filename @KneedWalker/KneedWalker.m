classdef KneedWalker
    % KneedWalker is a class that defines a kneed biped robot.

    properties
        shin  = [];   % mass, length, moment of inertia
        thigh = [];   % mass, length, moment of inertia
        torso = [];   % mass, length, moment of inertia
        g = 9.81;

        % Support leg coordinates
        xS = 0;
        yS = 0;
        
        % Support Leg
        Support = 2;
        
        % enum
        Left  = 1;
        Right = 2;
       
        % Control torques
        Torques = [0 0 0 0].';
        
        % Event index
        nEvents = 4;
        iEvent  = 4;  
        % 1 - leg contact
        % 2 - knee locked
        % 3 - To Be Decided
        % 4 - To Be Decided
    end
    
    
    methods
        % Class constructor
        function KW = KneedWalker(varargin)
            switch nargin
                case 0
                    KW;%#ok<VUNUS>
                case 3
                    KW.shin  = varargin{1};
                    KW.thigh = varargin{2};
                    KW.torso = varargin{3};
                    KW.xS = 0;
                    KW.yS = 0;
                case 4
                    KW.shin  = varargin{1};
                    KW.thigh = varargin{2};
                    KW.torso = varargin{3};
                    KW.xS = getPos(init);  %%% gets the initial position of the 
                    KW.yS = getPos(init);  %%% the support leg T.B.D
                otherwise
                    error('Couldn'' construct KneedWalker object, check your input');
            end
        end
        
        
        function [x, y] = GetPos(KW, X, which) %%% T.B.D
        if strcmp(which, 'S')
            x = KW.xS;
            y = KW.yS;
            return;
        end
        if strcmp(which, 'NS')
            x = KW.xNS;
            y = KW.yNS;  %%% T.B.D %%%
            return;
        end    
        end
        
        function [xdot, ydot] = GetVel(KW, X, which) %%%T.B.D
        end
        
        function [F] = GetGRF(KW,X) %%%T.B.D
        F = zeros(2,1);
        end
        
        function [Xdot] = Derivative(KW, t, X, iEvents)
        switch iEvents
            case 1
                %%% Xdot = something
            case 2
                %%% Xdot = something else
        end
        end
        
        function [value, isterminal, direction] = Events(KW, X, Floor)
            value = ones(KW.nEvents,1);
            isterminal = ones(KW.nEvents,1);
            direction = ones(KW.nEvents,1);
            
            value(1) = yNS - Floor.Surf(xNS);
            %%% value(2) = knee lock
        end
        
        function [KW, Xa] = HandleEvent(KW, iEvent, Xb, t)
        switch iEvent
            case 1 % Leg contact
                %%% Xa = %%% Impact calculation
                if KW.Support == KW.Left
                   KW.Support = KW.Right;
                elseif KW.Support == KW.Right
                       KW.Support = KW.Left;
                end
                % Update support foot position
                [xNS, yNS] = KW.GetPos(Xb,'NS');
                KW.yS = yNS;
                KW.xS = xNS;
        
%                 % Compute velocity
%                 dT = t-KW.last_t;
%                 KW.curSpeed = KW.GetStepLength(Xb)/dT;
%                 KW.last_t = t;
                
            case 2 % Knee Lock
                %%% T.B.D      
        end
        end
    end     
end



