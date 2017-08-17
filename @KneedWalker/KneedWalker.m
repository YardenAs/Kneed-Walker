classdef KneedWalker
    % KneedWalker is a class that defines a kneed biped robot.

    properties
        sh = [];       % shank mass, length, moment of inertia
        th = [];       % thigh mass, length, moment of inertia
        to = [];       % torso mass, length, moment of inertia
        grav  = 9.81;
        Order = 14;

        % Support leg coordinates
        xS = 0;
        yS = 0;
        
        % enum
        Left  = 1;
        Right = 2;
        
        % Support Leg and Phase
        Support = 2;
        
        % Control torques
        Torques = [0 0 0 0].'; % Ship, NShip, Sknee, NSknee
        
        % Event index
        nEvents = 5; 
        % 1 - leg contact
        % 2 - robot fell
        % 3 - abs(alpha) >= pi/2 (torso rotation)
        % 4 - Sknee lock
        % 5 - NSknee lock
        
        % Render parameters
        link_width = 0.025;
        link_color= [0.1, 0.3, 0.8];
        RenderObj;
        LinkRes = 10;
        LineWidth = 1;
    end
    
    
    methods
        % Class constructor
        function KW = KneedWalker(varargin)
            switch nargin
                case 0
                KW;%#ok<VUNUS>
                KW.sh = [2*0.15 0.45 1]; 
                KW.th = KW.sh;                  
                KW.to = [5 0.4 1];            
                KW.sh(3) = 1/12*KW.sh(1)*KW.sh(2)^2;
                KW.to(3) = 1/12*KW.to(1)*KW.to(2)^2;
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
                otherwise
                    error('Couldn''t construct KneedWalker object, check your input');
            end
        end
               
        function Pos = GetPos(KW, X, which)
        switch which
            case 'Sankle'
                x = KW.xS;
                y = KW.yS;
                Pos = [x y];
            case 'Sknee'
                rs1 = [X(1) + KW.th(2)*sin(X(7)), X(3) - KW.th(2)*cos(X(7))];
                x = rs1(1);
                y = rs1(2);
                Pos = [x y];
            case 'NSankle'
                rns1 = [X(1) - KW.sh(2)*sin(X(9)), X(3) - KW.th(2)*cos(X(9))];  % non support knee
                rns2 = rns1 + [- KW.sh(2)*sin(X(13)),- KW.th(2)*cos(X(13))];    % non support ankle
                x = rns2(1);
                y = rns2(2);
                Pos = [x y];
            case 'NSknee'
                rns1 = [X(1) - KW.sh(2)*sin(X(9)), X(3) - KW.th(2)*cos(X(9))];
                x = rns1(1);
                y = rns1(2);
                Pos = [x y];
            case 'TorsoCOM'
                rt = [X(1) + KW.to(2)/2*sin(X(5)), X(3) + KW.to(2)/2*cos(X(5))];
                x = rt(1);
                y = rt(2);
                Pos = [x y];
            case 'TorsoEnd'
                rt = [X(1) + KW.to(2)*sin(X(5)), X(3) + KW.to(2)*cos(X(5))];
                x = rt(1);
                y = rt(2);
                Pos = [x y];
            case 'Hip'
                x = X(1);
                y = X(3);
                Pos = [x y];
            otherwise
                error('No such position');
                
        end
        end
        
        function Vel = GetVel(KW, X, which)
        switch which
            case 'Sankle'
                xdot = X(2) + (X(12)*KW.sh(2)*cos(X(11))) + X(8)*KW.th(2)*cos(X(7));
                ydot = X(4) + (X(12)*KW.sh(2)*sin(X(11))) + X(8)*KW.th(2)*sin(X(7));
                Vel = [xdot ydot];
            case 'Sknee'
                xdot = X(2) + X(8)*KW.th(2)*cos(X(7));
                ydot = X(4) + X(8)*KW.th(2)*sin(X(7));
                Vel = [xdot ydot];
            case 'NSankle'
                xdot = X(2) - (X(14)*KW.sh(2)*cos(X(13))) - X(10)*KW.th(2)*cos(X(9));
                ydot = X(4) + (X(14)*KW.sh(2)*sin(X(13))) + X(10)*KW.th(2)*sin(X(9));
                Vel = [xdot ydot];
            case 'NSknee'
                xdot = X(2) - X(10)*KW.th(2)*cos(X(9));
                ydot = X(4) + X(10)*KW.th(2)*sin(X(9));
                Vel = [xdot ydot];
            case 'TorsoCOM'
                xdot = X(2) + X(6)*KW.to(2)*cos(X(5))/2;
                ydot = X(4) - X(6)*KW.to(2)*sin(X(5))/2;
                Vel = [xdot ydot];
            otherwise
                error('No such position');
        end
        end
        
        function [M, B, G, W, Wdot, Fq] = DynEq(KW, X)
            % gamma - shank angle w.r.t y axis
            % beta  - thigh angle w.r.t y axis
            % alpha - torso angle w.r.t y axis
            mt = KW.to(1); lt = KW.to(2); It = KW.to(3);
            msh = KW.sh(1); lsh = KW.sh(2); Ish = KW.sh(3);
            mth = KW.th(1); lth = KW.th(2); Ith = KW.th(3);
            g = KW.grav;
            uShip = KW.Torques(1); uNShip = KW.Torques(2); uSknee = KW.Torques(3);...
                uNSknee = KW.Torques(4);
            x = X(1); dx = X(2); y = X(3); dy = X(4); a = X(5); da = X(6); %#ok
            gs = X(7); dgs = X(8); gns = X(9); dgns = X(10); bs = X(11);
            dbs = X(12); bns = X(13); dbns = X(14);

            M = [            2*msh + mt + 2*mth,                             0,  (lt*mt*cos(a))/2,   (lth*cos(gs)*(3*msh + mth))/2,  -(lth*cos(gns)*(msh + mth))/2,          (3*lsh*msh*cos(bs))/4,          -(lsh*msh*cos(bns))/4
                0,            2*msh + mt + 2*mth, -(lt*mt*sin(a))/2,   (lth*sin(gs)*(3*msh + mth))/2,   (lth*sin(gns)*(msh + mth))/2,          (3*lsh*msh*sin(bs))/4,           (lsh*msh*sin(bns))/4
                (lt*mt*cos(a))/2,             -(lt*mt*sin(a))/2,  (mt*lt^2)/4 + It,                               0,                              0,                              0,                              0
                (lth*cos(gs)*(3*msh + mth))/2, (lth*sin(gs)*(3*msh + mth))/2,                 0, Ith + lth^2*msh + (lth^2*mth)/4,   -(lth^2*msh*cos(gns + gs))/2,   (lsh*lth*msh*cos(bs - gs))/2, -(lsh*lth*msh*cos(bns + gs))/4
                -(lth*cos(gns)*(msh + mth))/2,  (lth*sin(gns)*(msh + mth))/2,                 0,    -(lth^2*msh*cos(gns + gs))/2,            (mth*lth^2)/4 + Ith, -(lsh*lth*msh*cos(bs + gns))/4,                              0
                (3*lsh*msh*cos(bs))/4,         (3*lsh*msh*sin(bs))/4,                 0,    (lsh*lth*msh*cos(bs - gs))/2, -(lsh*lth*msh*cos(bs + gns))/4,            (msh*lsh^2)/4 + Ish,   -(lsh^2*msh*cos(bns + bs))/8
                -(lsh*msh*cos(bns))/4,          (lsh*msh*sin(bns))/4,                 0,  -(lsh*lth*msh*cos(bns + gs))/4,                              0,   -(lsh^2*msh*cos(bns + bs))/8,                            Ish];

            B =    [(lsh*msh*sin(bns)*dbns^2)/4 + ((lth*msh*sin(gns))/2 + (lth*mth*sin(gns))/2)*dgns^2 + (- (3*lth*msh*sin(gs))/2 - (lth*mth*sin(gs))/2)*dgs^2 - (da^2*lt*mt*sin(a))/2 - (3*dbs^2*lsh*msh*sin(bs))/4
                (lsh*msh*cos(bns)*dbns^2)/4 + (3*lsh*msh*cos(bs)*dbs^2)/4 + ((lth*msh*cos(gns))/2 + (lth*mth*cos(gns))/2)*dgns^2 + ((3*lth*msh*cos(gs))/2 + (lth*mth*cos(gs))/2)*dgs^2 - (da^2*lt*mt*cos(a))/2
                0
                (lth*msh*(lsh*sin(bns + gs)*dbns^2 - 2*lsh*sin(bs - gs)*dbs^2 + 2*lth*sin(gns + gs)*dgns^2))/4
                (lsh*msh*sin(bs + gns)*dbs^2*lth)/4 + (msh*sin(gns + gs)*dgs^2*lth^2)/2
                (lsh*msh*(lsh*sin(bns + bs)*dbns^2 + 2*lth*sin(bs + gns)*dgns^2 + 4*lth*sin(bs - gs)*dgs^2))/8
                (msh*sin(bns + bs)*dbs^2*lsh^2)/8 + (lth*msh*sin(bns + gs)*dgs^2*lsh)/4];

            G = [0
                g*(2*msh + mt + 2*mth)
                -(g*lt*mt*sin(a))/2
                (g*lth*sin(gs)*(2*msh + mth))/2
                (g*lth*sin(gns)*(2*msh + mth))/2
                (g*lsh*msh*sin(bs))/2
                (g*lsh*msh*sin(bns))/2];

            W = [ 1, 0, 0, lth*cos(gs), 0, lsh*cos(bs), 0
                0, 1, 0, lth*sin(gs), 0, lsh*sin(bs), 0];

            Wdot = [ 0, 0, 0, -dgs*lth*sin(gs), 0, -dbs*lsh*sin(bs), 0
                0, 0, 0,  dgs*lth*cos(gs), 0,  dbs*lsh*cos(bs), 0];

            Fq = [          0
                0
                uNShip + uShip
                uShip + uSknee
                uNSknee - uNShip
                -uSknee
                -uNSknee];
        end
            
        function [F] = GetReactionForces(KW, X)
            [M, B, G, W, Wdot, Fq] = DynEq(KW, X);
            dq = [X(2), X(4), X(6), X(8), X(10), X(12), X(14)].';
            A = [M -W.'; W zeros(2,2)];
            b = [Fq - B - G; -Wdot*dq];
            sol = A\b;
            F = sol(end-1:end).';
        end
        
        function [Xdot] = Derivative(KW, t, X) %#ok<INUSL>
            [M, B, G, W, Wdot, Fq] = DynEq(KW, X);
            dq = [X(2), X(4), X(6), X(8), X(10), X(12), X(14)].';
            A = [M -W.'; W zeros(2,2)];
            b = [Fq - B - G; -Wdot*dq];
            sol = A\b;
            Xdot = [X(2) sol(1) X(4) sol(2) X(6) sol(3) X(8) sol(4)...
                    X(10) sol(5) X(12) sol(6) X(14) sol(7)].';      
        end
        
        function [value, isterminal, direction] = Events(KW, X, Floor)
            value = ones(KW.nEvents,1);
            isterminal = ones(KW.nEvents,1);
            direction = -ones(KW.nEvents,1);
            % 1 - leg contact
            % 2 - robot fell
            % 3 - abs(alpha) >= pi/2 (torso rotation)
            % 4 - Sknee lock
            % 5 - NSknee lock
            % Event 1 - Ground contact   
            NSPos = KW.GetPos(X, 'NSankle');
            value(1) = NSPos(2) - Floor.Surf(NSPos(1));
            % Event 2 - robot fell
            HipPos = KW.GetPos(X, 'Hip');
            SAnklePos = KW.GetPos(X, 'Sankle');
            value(2) = HipPos(2) - SAnklePos(2) - 0.6*(KW.sh(2) + KW.th(2));
            % Event 3 - abs(alpha) >= pi/2 (torso rotation)  
            value(3) = abs(X(5)) - pi/2;
            % Event 4 - Sknee lock
            value(4) = X(11) - X(7);
            % Event 5 - NSknee lock
            value(5) = X(9) - X(13);
        end
        
        function [Xf] = CalcImpact(KW, Xi)
        [M, ~, ~, W, ~, ~] = DynEq(KW, X);
        Xf = Xi;
        Xf([2 4 6 8 10 12 14]) = (eye(7) - M^-1*W.'*(W*M^-1*W.')^-1*W)*Xi([2 4 6 8 10 12 14]); % refer to Hybrid dynamics course for this equation
        end
       
        function [KW, Xf] = HandleEvent(KW, iEvent, Xi)
        % 1 - leg contact
        % 2 - robot fell
        % 3 - abs(alpha) >= pi/2 (torso rotation)
        % 4 - Sknee lock
        % 5 - NSknee lock
        % Event 1 - Ground contact
            switch iEvent
                case 1 
                     Xf = CalcImpact(KW, Xi);
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
                % Event 2 - Robot fell
                case 2 
                    Xf = Xi;
                % Event 3 - abs(alpha) >= pi/2 (torso rotation)
                case 3
                    Xf = Xi;
                % Event 4 - Sknee lock
                case 4
                    Xf = Xi;   
                % Event 5 - NSknee lock
                case 5
                    Xf = Xi;
            end
        end
        
        function KW = SetTorques(KW ,T)
            KW.Torques = T;
        end
    end
end     



