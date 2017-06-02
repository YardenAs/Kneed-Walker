classdef KneedWalker
    % KneedWalker is a class that defines a kneed biped robot.

    properties
        sh = [];    % shank mass, length, moment of inertia
        th = [];    % thigh mass, length, moment of inertia
        to = [];    % torso mass, length, moment of inertia
        g  = 9.81;

        % Support leg coordinates
        xS = 0;
        yS = 0;
        
        % enum
        Left  = 1;
        Right = 2;
        
        % Support Leg and Phase
        Support = 2;
        Phase = 'KneesFree'; % SKneeFree, NSKneeFree, KneesLock
        
        % Control torques
        Torques = [0 0 0 0].'; % Ship, NShip, Sknee, NSknee
        
        % Event index
        nEvents = 4;
        iEvent  = 4;  
        % 1 - leg contact
        % 2 - support knee locked
        % 3 - non support knee locked
        % 4 - T.B.D (knee lock releases???)
    end
    
    
    methods
        % Class constructor
        function KW = KneedWalker(varargin)
            switch nargin
                case 0
                KW;%#ok<VUNUS>
                case 3
                if (length(varargin{1}) ~= 3) || (length(varargin{2}) ~= 3) ...
                    || (length(varargin{3}) ~= 3)
                error('Robot parameters aren''t valid');
                else
                KW.sh  = varargin{1};
                KW.th = varargin{2};
                KW.to = varargin{3};
                KW.xS = 0;
                KW.yS = 0;
                end
                case 4
                if (length(varargin{1}) ~= 3) || (length(varargin{2}) ~= 3) ...
                    || (length(varargin{3}) ~= 3)
                error('Robot parameters aren''t valid');
                else
                KW.sh  = varargin{1};
                KW.th = varargin{2};
                KW.to = varargin{3};
                KW.xS = getPos(init);  %%% gets the initial position of the 
                KW.yS = getPos(init);  %%% the support leg T.B.D
                end
                otherwise
                    error('Couldn''t construct KneedWalker object, check your input');
            end
        end
               
        function [x, y] = GetPos(KW, X, which)
            switch which
                case 'Sankle'
                x = KW.xS;
                y = KW.yS;
                case 'Sknee'
                rs1 = [X(1) + KW.th(2)*sin(X(7)), X(3) - KW.th(2)*cos(X(7))];
                x = rs1(1);
                y = rs1(2);
                case 'NSankle'
                rns1 = [X(1) - KW.sh(2)*sin(X(9)), X(3) - KW.th(2)*cos(X(9))];  % non support knee          
                rns2 = rns1 + [- KW.sh(2)*sin(X(13)),- KW.th(2)*cos(X(13))];    % non support ankle
                x = rns2(1);
                y = rns2(2);
                case 'NSknee'
                rns1 = [X(1) - KW.sh(2)*sin(X(9)), X(3) - KW.th(2)*cos(X(9))];
                x = rns1(1);
                y = rns1(2);
                case 'TorsoCOM'
                rt = [X(1) + KW.to(2)/2*sin(X(5)), X(3) + KW.to(2)/2*cos(X(5))];
                x = rt(1);
                y = rt(2);
                otherwise
                    error('No such position');
            end
        end
        
        function [xdot, ydot] = GetVel(KW, X, which)
            switch which
                case 'Sankle'
                xdot = X(2) + (X(12)*KW.sh(2)*cos(X(11))) + X(8)*KW.th(2)*cos(X(7));
                ydot = X(4) + (X(12)*KW.sh(2)*sin(X(11))) + X(8)*KW.th(2)*sin(X(7));                
                case 'Sknee'
                xdot = X(2) + X(8)*KW.th(2)*cos(X(7));
                ydot = X(4) + X(8)*KW.th(2)*sin(X(7));
                case 'NSankle'
                xdot = X(2) - (X(14)*KW.sh(2)*cos(X(13))) - X(10)*KW.th(2)*cos(X(9));
                ydot = X(4) + (X(14)*KW.sh(2)*sin(X(13))) + X(10)*KW.th(2)*sin(X(9));
                case 'NSknee'
                xdot = X(2) - X(10)*KW.th(2)*cos(X(9));
                ydot = X(4) + X(10)*KW.th(2)*sin(X(9));
                case 'TorsoCOM'
                xdot = X(2) + X(6)*KW.to(2)*cos(X(5))/2;
                ydot = X(4) - X(6)*KW.to(2)*sin(X(5))/2;
                otherwise
                    error('No such position');
            end
        end
        
        function [F] = GetReactionForces(KW, X)
            [M, B, G, W, Wdot, Fq] = DynEq(KW, X);
            dq = [X(2), X(4), X(6), X(8), X(10), X(12), X(14)].';
            A = [M -W.'; W zeros(2,2)];
            b = [Fq - B - G; -Wdot*dq];
            sol = A\b;
            F = [sol(8) end].';
        end
        
        function [Xdot] = Derivative(KW, t, X) %#ok<INUSL>
            [M, B, G, W, Wdot, Fq] = DynEq(KW, X);
            dq = [X(2), X(4), X(6), X(8), X(10), X(12), X(14)].';
            A = [M -W.'; W zeros(size(W))];
            b = [Fq - B - G; -Wdot*dq];
            sol = A\b;
            Xdot = [X(2) sol(1) X(4) sol(2) X(6) sol(3) X(8) sol(4)...
                    X(10) sol(5) X(12) sol(6) X(14) sol(7)].';      
        end
        
        function [value, isterminal, direction] = Events(KW, X, Floor)
            value = ones(KW.nEvents,1);
            isterminal = ones(KW.nEvents,1);
            direction = [-1 1 1 0]; %% CHECK
            
            yNS = KW.GetPos(X, 'NSankle');
            value(1) = yNS - Floor.Surf(xNS); % ground contact
            if strcmp(KW.Phase, 'KneesFree') || strcmp(KW.Phase, 'SKneeFree')
                value(2) = X(11) - X(7);          % support knee lock
            end
            if strcmp(KW.Phase, 'KneesFree') || strcmp(KW.Phase, 'NSKneeFree')
                value(3) = X(9) - X(13);          % non support knee lock
            end
        end
        
        function [Xa, LAMBDA] = CalcImpact(KW, Xb)
            [M, W] = EventMat(KW, Xb);
            A = [M -W.'; W zeros(size(W))];
            b = [-M*Xb, zeros(size(W,1))];
            sol = A\b;
            Xa = sol(1:7);
            LAMBDA = sol(8:end);
            %%% here get event asummps GetEventAssumps = something %%%
        end
       
        function [KW, Xa] = HandleEvent(KW, iEvent, Xb)
            switch iEvent
                case 1 % ground contact
                     Xa = CalcImpact(KW, Xb);
                    % Update support leg
                    if KW.Support == KW.Left
                    KW.Support = KW.Right;
                    elseif KW.Support == KW.Right
                        KW.Support = KW.Left;
                    end
                    % Update support foot position
                    [xNS, yNS] = KW.GetPos(Xb,'NS');
                    KW.yS = yNS;
                    KW.xS = xNS;
                    % Update walking phase
                    if strcmp(KW.Phase, 'KneesLocked')
                        KW.Phase = 'NSKneeFree'; % otherwise - this is a biped without knees
                    end
                    if strcmp(KW.Phase, 'SKneeFree')
                        KW.Phase = 'NSKneeFree';
                    end
                    if strcmp(KW.Phase, 'NSKneeFree')
                        KW.Phase = 'SKneeFree';
                    end
                case 2 % support knee locked
                    Xa = CalcImpact(KW, Xb);
                    % Update walking phase
                    if strcmp(KW.Phase, 'KneesFree')
                        KW.Phase = 'NSKneeFree';
                    end
                    if strcmp(KW.Phase, 'SKneeFree')
                        KW.Phase = 'KneesLocked';
                    end
                case 3 % non support knee locked
                    Xa = CalcImpact(KW, Xb);
                    % Update walking phase
                    if strcmp(KW.Phase, 'KneesFree')
                        KW.Phase = 'SKneeFree';
                    end
                    if strcmp(KW.Phase, 'NSKneeFree')
                        KW.Phase = 'KneesLocked';
                    end
            end
        end
        
        function CB = SetTorques(CB,T)
            CB.Torques=T;
        end
    end
end     



