classdef KneedWalker  < handle & matlab.mixin.Copyable
    % KneedWalker is a class that defines a kneed biped robot.

    properties
        sh = [];       % shank mass, length, moment of inertia
        th = [];       % thigh mass, length, moment of inertia
        to = [];       % torso mass, length, moment of inertia
        grav  = 9.81;
        Order = 14;
        
        % Support
        Support = 'Left'; % Right
        
        % Control torques
        Torques = [0 0 0 0].'; % Ship, NShip, Sknee, NSknee
        
        % Event index
        nEvents = 7; 
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
                KW.to = [5 0.4 1];            
                KW.sh(3) = 1/12*KW.sh(1)*KW.sh(2)^2;
                KW.th = KW.sh;
                KW.to(3) = 1/12*KW.to(1)*KW.to(2)^2;
                case 3
                if (length(varargin{1}) ~= 3) || (length(varargin{2}) ~= 3) ...
                    || (length(varargin{3}) ~= 3)
                error('Robot parameters aren''t valid');
                else
                KW.sh  = varargin{1};
                KW.th = varargin{2};
                KW.to = varargin{3};
                end
                otherwise
                    error('Couldn''t construct KneedWalker object, check your input');
            end
        end
                      
        function Pos = GetPos(KW, X, which)
        switch which
            case 'Sankle'
                x = X(1);
                y = X(2);
                Pos = [x y];
            case 'Sknee'
                rsk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                Pos = rsk;
            case 'NSankle'
                rsk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                rh  = rsk + [KW.th(2)*sin(X(4)), -KW.th(2)*cos(X(4))];          
                rnsk = rh + [-KW.th(2)*sin(X(5)), KW.th(2)*cos(X(5))];
                rns2 = rnsk + [-KW.sh(2)*sin(X(7)), KW.sh(2)*cos(X(7))];
                x = rns2(1);
                y = rns2(2);
                Pos = [x y];
            case 'NSknee'
                rsk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                rh  = rsk + [KW.th(2)*sin(X(4)), -KW.th(2)*cos(X(4))];          
                rnsk = rh + [-KW.th(2)*sin(X(5)), KW.th(2)*cos(X(5))];
                x = rnsk(1);
                y = rnsk(2);
                Pos = [x y];
            case 'TorsoCOM'
                rsk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                rh  = rsk + [KW.th(2)*sin(X(4)), -KW.th(2)*cos(X(4))]; 
                rt  = rh + [-KW.to(2)/2*sin(X(3)), KW.to(2)/2*cos(X(3))];
                x = rt(1);
                y = rt(2);
                Pos = [x y];
            case 'TorsoEnd'
                rsk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                rh  = rsk + [KW.th(2)*sin(X(4)), -KW.th(2)*cos(X(4))]; 
                rt  = rh + [-KW.to(2)*sin(X(3)), KW.to(2)*cos(X(3))];
                x = rt(1);
                y = rt(2);
                Pos = [x y];
            case 'Hip'
                rsk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                rh  = rsk + [KW.th(2)*sin(X(4)), -KW.th(2)*cos(X(4))]; 
                Pos = rh;
            otherwise
                error('No such position');
                
        end
        end
        
        function Vel = GetVel(KW, X, which)
        switch which
            case 'NSankle'
                xdot =  X(8) - X(12)*KW.th(2)*cos(X(5)) + X(11)*KW.th(2)*cos(X(4)) - X(14)*KW.sh(2)*cos(X(7)) + X(13)*KW.sh(2)*cos(X(6));
                ydot =  X(9) - X(12)*KW.th(2)*sin(X(5)) + X(11)*KW.th(2)*sin(X(4)) - X(14)*KW.sh(2)*sin(X(7)) + X(13)*KW.sh(2)*sin(X(6));
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
            x = X(1); y = X(2); a = X(3); bs = X(4); bns = X(5); gs = X(6); gns = X(7); %#ok
            dx = X(8); dy = X(9); da = X(10); dbs = X(11); dbns = X(12); dgs = X(13); dgns = X(14); %#ok
   
            M = [                     2*msh + mt + 2*mth,                                                                  0,          -(lt*mt*cos(a))/2,                                                                                             (lth*cos(bs)*(msh + 2*mt + 3*mth))/2,            -(lth*cos(bns)*(msh + mth))/2,                                                                                           (lsh*cos(gs)*(5*msh + 4*mt + 8*mth))/4,        -(lsh*msh*cos(gns))/4;
                                                       0,                                                 2*msh + mt + 2*mth,          -(lt*mt*sin(a))/2,                                                               (lth*(msh*sin(bs) - mth*cos(bs) + 2*mt*sin(bs) + 2*mth*sin(bs)))/2,            -(lth*sin(bns)*(msh + mth))/2,                                                                                           (lsh*sin(gs)*(5*msh + 4*mt + 8*mth))/4,        -(lsh*msh*sin(gns))/4;
                                       -(lt*mt*cos(a))/2,                                                  -(lt*mt*sin(a))/2,           (mt*lt^2)/4 + It,                                                                                                       -(lt*lth*mt*cos(a - bs))/2,                                        0,                                                                                                       -(lsh*lt*mt*cos(a - gs))/2,                            0;
                    (lth*cos(bs)*(msh + 2*mt + 3*mth))/2, (lth*(msh*sin(bs) - mth*cos(bs) + 2*mt*sin(bs) + 2*mth*sin(bs)))/2, -(lt*lth*mt*cos(a - bs))/2,                                                                       Ish + lth^2*mt + (3*lth^2*mth)/2 - (lth^2*mth*sin(bs)^2)/2,             -(lth^2*mth*cos(bns - bs))/2, (lsh*lth*(mth*cos(bs + gs) - mth*sin(bs + gs) + msh*cos(bs - gs) + 4*mt*cos(bs - gs) + 5*mth*cos(bs - gs) + mth*sin(bs - gs)))/4,                            0;
                           -(lth*cos(bns)*(msh + mth))/2,                                      -(lth*sin(bns)*(msh + mth))/2,                          0,                                                                                                     -(lth^2*mth*cos(bns - bs))/2,                      (mth*lth^2)/4 + Ish,                                                                                         -(lsh*lth*cos(bns - gs)*(msh + 2*mth))/4,                            0;
                  (lsh*cos(gs)*(5*msh + 4*mt + 8*mth))/4,                             (lsh*sin(gs)*(5*msh + 4*mt + 8*mth))/4, -(lsh*lt*mt*cos(a - gs))/2, (lsh*lth*(mth*cos(bs + gs) - mth*sin(bs + gs) + msh*cos(bs - gs) + 4*mt*cos(bs - gs) + 5*mth*cos(bs - gs) + mth*sin(bs - gs)))/4, -(lsh*lth*cos(bns - gs)*(msh + 2*mth))/4,                                                                                   Ith + (3*lsh^2*msh)/4 + lsh^2*mt + 2*lsh^2*mth, -(lsh^2*msh*cos(gns - gs))/8;
                                   -(lsh*msh*cos(gns))/4,                                              -(lsh*msh*sin(gns))/4,                          0,                                                                                                                                0,                                        0,                                                                                                     -(lsh^2*msh*cos(gns - gs))/8,                          Ith];
 
            B = [(lt*mt*sin(a)*da^2)/2 + ((lth*msh*sin(bns))/2 + (lth*mth*sin(bns))/2)*dbns^2 + (- (lth*msh*sin(bs))/2 - lth*mt*sin(bs) - (3*lth*mth*sin(bs))/2)*dbs^2 + (lsh*msh*sin(gns)*dgns^2)/4 + (- (5*lsh*msh*sin(gs))/4 - lsh*mt*sin(gs) - 2*lsh*mth*sin(gs))*dgs^2;
               (- (lth*msh*cos(bns))/2 - (lth*mth*cos(bns))/2)*dbns^2 + ((lth*mth*sin(bs))/2 + (lth*msh*cos(bs))/2 + lth*mt*cos(bs) + lth*mth*cos(bs))*dbs^2 + ((5*lsh*msh*cos(gs))/4 + lsh*mt*cos(gs) + 2*lsh*mth*cos(gs))*dgs^2 - (da^2*lt*mt*cos(a))/2 - (dgns^2*lsh*msh*cos(gns))/4;
               -(lt*mt*(lth*sin(a - bs)*dbs^2 + lsh*sin(a - gs)*dgs^2))/2;
               (dbns^2*lth^2*mth*sin(bns - bs))/2 - (dbs^2*lth^2*mth*sin(2*bs))/4 - (dgs^2*lsh*lth*mth*sin(bs + gs))/4 - (dgs^2*lsh*lth*mth*cos(bs - gs))/4 + (da^2*lt*lth*mt*sin(a - bs))/2 + (dgs^2*lsh*lth*msh*sin(bs - gs))/4 + dgs^2*lsh*lth*mt*sin(bs - gs) + (5*dgs^2*lsh*lth*mth*sin(bs - gs))/4 - (dgs^2*lsh*lth*mth*cos(bs + gs))/4;
               (- (lsh*lth*msh*sin(bns - gs))/4 - (lsh*lth*mth*sin(bns - gs))/2)*dgs^2 - (dbs^2*lth^2*mth*sin(bns - bs))/2;
               -(lsh*(2*dbs^2*lth*mth*cos(bs + gs) + 2*dbs^2*lth*mth*sin(bs + gs) - 2*dbs^2*lth*mth*cos(bs - gs) - 4*da^2*lt*mt*sin(a - gs) - 2*dbns^2*lth*msh*sin(bns - gs) - 4*dbns^2*lth*mth*sin(bns - gs) + 2*dbs^2*lth*msh*sin(bs - gs) + 8*dbs^2*lth*mt*sin(bs - gs) + 10*dbs^2*lth*mth*sin(bs - gs) - dgns^2*lsh*msh*sin(gns - gs)))/8;
               -(dgs^2*lsh^2*msh*sin(gns - gs))/8];
                               
           G =    [               0;
               g*(2*msh + mt + 2*mth);
               -(g*lt*mt*sin(a))/2;
               (g*lth*(2*msh*sin(bs) - mth*cos(bs) + 2*mt*sin(bs) + 2*mth*sin(bs)))/2;
               -(g*lth*sin(bns)*(2*msh + mth))/2;
               (g*lsh*sin(gs)*(3*msh + 2*mt + 4*mth))/2;
               -(g*lsh*msh*sin(gns))/2];
           if strcmp(KW.Support,'Left')
               W = [ 1, 0, 0, 0, 0, 0, 0;
                     0, 1, 0, 0, 0, 0, 0];
               Wdot = zeros(size(W));
           elseif strcmp(KW.Support,'Right')
               W =  [ 1, 0, 0, lth*cos(bs), - lsh*cos(bns) - (lth*cos(bns))/2, lsh*cos(gs), -(lth*cos(gns))/2;
                      0, 1, 0, lth*sin(bs),   lsh*sin(bns) - (lth*sin(bns))/2, lsh*sin(gs),  (lth*sin(gns))/2];
               Wdot = [ 0, 0, 0, -dbs*lth*sin(bs), dbns*(lsh*sin(bns) + (lth*sin(bns))/2), -dgs*lsh*sin(gs), (dgns*lth*sin(gns))/2;
                        0, 0, 0,  dbs*lth*cos(bs), dbns*(lsh*cos(bns) - (lth*cos(bns))/2),  dgs*lsh*cos(gs), (dgns*lth*cos(gns))/2];
           end

            Fq = [             0;
                               0;
                - uNShip - uShip;
                  uShip - uSknee;
                uNShip - uNSknee;
                          uSknee;
                         uNSknee];
        end
            
        function [F] = GetReactionForces(KW, X)
            [M, B, G, W, Wdot, Fq] = DynEq(KW, X);
            dq = X(8:end);
            dim = size(W);
            A = [M -W.'; W zeros(dim(1))];
            b = [Fq - B - G; -Wdot*dq];
            sol = A\b;
            F = sol(end-1:end).';
        end
        
        function [Xdot] = Derivative(KW, t, X) %#ok
            [M, B, G, W, Wdot, Fq] = DynEq(KW, X);
            dq = X(8:end);
            [Wrows, ~] = size(W);
            A = [M -W.'; W zeros(Wrows)];
            b = [Fq - B - G; -Wdot*dq];
            sol = A\b;
            Xdot = [dq; sol(1:7)];      
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
            if strcmp(KW.Support,'Left')
                NSPos = KW.GetPos(X, 'NSankle');
            elseif strcmp(KW.Support,'Right')
                NSPos = KW.GetPos(X, 'Sankle');
            end
            value(1) = NSPos(2) - Floor.Surf(NSPos(1));
            % Event 2 - robot fell
            HipPos = KW.GetPos(X, 'Hip');
            if strcmp(KW.Support,'Left')
                SAnklePos = KW.GetPos(X, 'Sankle');
            elseif strcmp(KW.Support,'Right')
                SAnklePos = KW.GetPos(X, 'NSankle');
            end
            value(2) = HipPos(2) - SAnklePos(2) - 0.6*(KW.sh(2) + KW.th(2));
            % Event 5 - abs(alpha) >= pi/2 (torso rotation)  
            value(3) = pi/2 - abs(X(3));
            % Event 6 - Sknee lock
            value(4) = X(4) - X(6);
            % Event 7 - NSknee lock
            value(5) = X(5) - X(7);
        end
        
        function [Xf] = CalcImpact(KW, Xi)
            [M, ~, ~, Wc, ~, ~] = DynEq(KW, Xi);
%             Xdotnew = (eye(7) - M^-1*Wc.'*((Wc*M^-1*Wc.')^-1)*Wc)*Xi(8:end).'; % refer to Hybrid dynamics course for this equation
            A = [M, -Wc.';Wc, zeros(2)];
            b = [M*Xi(8:end).';zeros(2,1)];
            sol = A\b;
            Xdotnew = sol(1:7);
            disp(num2str(sol(8:9)))
            Xf = [Xi(1:7),Xdotnew.'];
        end
       
        function Xf = HandleEvent(KW, iEvent, Xi)
            % 1 - leg contact
            % 2 - Fn = 0 Right
            % 3 - Fn = 0 Left
            % 4 - robot fell
            % 5 - abs(alpha) >= pi/2 (torso rotation)
            % 6 - Sknee lock
            % 7 - NSknee lock
            switch iEvent
                % Event 1 - Ground contact
                case 1
                    if strcmp(KW.Support,'Left')
                        KW.Support = 'Right';
                    elseif strcmp(KW.Support,'Right')
                        KW.Support = 'Left';
                    end
                    Xf = CalcImpact(KW, Xi);
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
        
        function E = GetEnergy(KW, X)
            mt = KW.to(1); lt = KW.to(2); It = KW.to(3);
            msh = KW.sh(1); lsh = KW.sh(2); Ish = KW.sh(3);
            mth = KW.th(1); lth = KW.th(2); Ith = KW.th(3);
            g = KW.grav;
            uShip = KW.Torques(1); uNShip = KW.Torques(2); uSknee = KW.Torques(3);... %%ok
                uNSknee = KW.Torques(4); %#ok
            x = X(1); y = X(2); a = X(3); bs = X(4); bns = X(5); gs = X(6); gns = X(7); %#ok
            dx = X(8); dy = X(9); da = X(10); dbs = X(11); dbns = X(12); dgs = X(13); dgns = X(14); 
            E  = (msh*((dy + (dgs*lsh*sin(gs))/2)*(dy - dbns*lth*sin(bns) + dbs*lth*sin(bs) - (dgns*lsh*sin(gns))/2 + dgs*lsh*sin(gs)) + (dx + (dgs*lsh*cos(gs))/2)*(dx - dbns*lth*cos(bns) + dbs*lth*cos(bs) - (dgns*lsh*cos(gns))/2 + dgs*lsh*cos(gs))))/2 + (g*(4*msh*y + 2*mt*y + 4*mth*y + lsh*msh*cos(gns) - 3*lsh*msh*cos(gs) - 2*lsh*mt*cos(gs) - 4*lsh*mth*cos(gs) - lth*mth*sin(bs) + lt*mt*cos(a) + 2*lth*msh*cos(bns) + lth*mth*cos(bns) - 2*lth*msh*cos(bs) - 2*lth*mt*cos(bs) - 2*lth*mth*cos(bs)))/2 + (mth*((dx + (dbs*lth*cos(bs))/2 + dgs*lsh*cos(gs))^2 + (dy + dgs*lsh*sin(gs) - (dbs*lth*cos(bs))/2)^2))/2 + (It*da^2)/2 + (Ish*dbns^2)/2 + (Ish*dbs^2)/2 + (Ith*dgns^2)/2 + (Ith*dgs^2)/2 + (mt*((dx - (da*lt*cos(a))/2 + dbs*lth*cos(bs) + dgs*lsh*cos(gs))^2 + (dy - (da*lt*sin(a))/2 + dbs*lth*sin(bs) + dgs*lsh*sin(gs))^2))/2 + (mth*((dx - (dbns*lth*cos(bns))/2 + dbs*lth*cos(bs) + dgs*lsh*cos(gs))^2 + (dy - (dbns*lth*sin(bns))/2 + dbs*lth*sin(bs) + dgs*lsh*sin(gs))^2))/2 + (msh*((dx + (dgs*lsh*cos(gs))/2)^2 + (dy + (dgs*lsh*sin(gs))/2)^2))/2;
        end
        function KW = SetTorques(KW ,T)
            KW.Torques = T;
        end
    end
end     



