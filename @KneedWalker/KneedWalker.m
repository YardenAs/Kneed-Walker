classdef KneedWalker  < handle & matlab.mixin.Copyable
    % KneedWalker is a class that defines a kneed biped robot.
    
    properties
        sh = [];       % shank mass, length, moment of inertia
        th = [];       % thigh mass, length, moment of inertia
        to = [];       % torso mass, length, moment of inertia
        grav  = 9.81;
        Order = 12;
        
        % Support
        Support = 'Left'; % Right
        
        % Flags
        BadImpulse = [];
        BadLiftoff = [];
        
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
                rsk = [X(1) + KW.sh(2)*sin(X(5)), X(2) - KW.sh(2)*cos(X(5))];
                Pos = rsk;
            case 'NSankle'
                rsk = [X(1) + KW.sh(2)*sin(X(5)), X(2) - KW.sh(2)*cos(X(5))];
                rh  = rsk + [KW.th(2)*sin(X(3)), -KW.th(2)*cos(X(3))];
                rnsk = rh + [-KW.th(2)*sin(X(4)), KW.th(2)*cos(X(4))];
                rns2 = rnsk + [-KW.sh(2)*sin(X(6)), KW.sh(2)*cos(X(6))];
                x = rns2(1);
                y = rns2(2);
                Pos = [x y];
            case 'NSknee'
                rsk = [X(1) + KW.sh(2)*sin(X(5)), X(2) - KW.sh(2)*cos(X(5))];
                rh  = rsk + [KW.th(2)*sin(X(3)), -KW.th(2)*cos(X(3))];
                rnsk = rh + [-KW.th(2)*sin(X(4)), KW.th(2)*cos(X(4))];
                x = rnsk(1);
                y = rnsk(2);
                Pos = [x y];
            case 'TorsoCOM'
                rsk = [X(1) + KW.sh(2)*sin(X(5)), X(2) - KW.sh(2)*cos(X(5))];
                rh  = rsk + [KW.th(2)*sin(X(3)), -KW.th(2)*cos(X(3))];
                rt  = rh;
                x = rt(1);
                y = rt(2);
                Pos = [x y];
            case 'TorsoEnd'
                rsk = [X(1) + KW.sh(2)*sin(X(5)), X(2) - KW.sh(2)*cos(X(5))];
                rh  = rsk + [KW.th(2)*sin(X(3)), -KW.th(2)*cos(X(3))];
                rt  = rh;
                x = rt(1);
                y = rt(2);
                Pos = [x y];
            case 'Hip'
                rsk = [X(1) + KW.sh(2)*sin(X(5)), X(2) - KW.sh(2)*cos(X(5))];
                rh  = rsk + [KW.th(2)*sin(X(3)), -KW.th(2)*cos(X(3))];
                Pos = rh;
            otherwise
                error('No such position');
                
        end
        end
        
        function Vel = GetVel(KW, X, which)
        switch which
            case 'Sankle'
                xdot =  X(7);
                ydot =  X(8);
                Vel = [xdot ydot];
            case 'NSankle'
                xdot =  X(7) - X(10)*KW.th(2)*cos(X(4)) + X(9)*KW.th(2)*cos(X(3)) - X(12)*KW.sh(2)*cos(X(6)) + X(11)*KW.sh(2)*cos(X(5));
                ydot =  X(8) - X(10)*KW.th(2)*sin(X(4)) + X(9)*KW.th(2)*sin(X(3)) - X(12)*KW.sh(2)*sin(X(6)) + X(11)*KW.sh(2)*sin(X(5));
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
        x = X(1); y = X(2); bs = X(3); bns = X(4); gs = X(5); gns = X(6); %#ok
        dx = X(7); dy = X(8); dbs = X(9); dbns = X(10); dgs = X(11); dgns = X(12); %#ok
        
        M = [                     2*msh + mt + 2*mth,                                                                  0,                                                                                             (lth*cos(bs)*(msh + 2*mt + 3*mth))/2,            -(lth*cos(bns)*(msh + mth))/2,                                                                                           (lsh*cos(gs)*(5*msh + 4*mt + 8*mth))/4,        -(lsh*msh*cos(gns))/4;
            0,                                                 2*msh + mt + 2*mth,                                                               (lth*(msh*sin(bs) - mth*cos(bs) + 2*mt*sin(bs) + 2*mth*sin(bs)))/2,            -(lth*sin(bns)*(msh + mth))/2,                                                                                           (lsh*sin(gs)*(5*msh + 4*mt + 8*mth))/4,        -(lsh*msh*sin(gns))/4;
            (lth*cos(bs)*(msh + 2*mt + 3*mth))/2, (lth*(msh*sin(bs) - mth*cos(bs) + 2*mt*sin(bs) + 2*mth*sin(bs)))/2,                                                                       Ish + lth^2*mt + (3*lth^2*mth)/2 - (lth^2*mth*sin(bs)^2)/2,             -(lth^2*mth*cos(bns - bs))/2, (lsh*lth*(mth*cos(bs + gs) - mth*sin(bs + gs) + msh*cos(bs - gs) + 4*mt*cos(bs - gs) + 5*mth*cos(bs - gs) + mth*sin(bs - gs)))/4,                            0;
            -(lth*cos(bns)*(msh + mth))/2,                                      -(lth*sin(bns)*(msh + mth))/2,                                                                                                     -(lth^2*mth*cos(bns - bs))/2,                      (mth*lth^2)/4 + Ish,                                                                                         -(lsh*lth*cos(bns - gs)*(msh + 2*mth))/4,                            0;
            (lsh*cos(gs)*(5*msh + 4*mt + 8*mth))/4,                             (lsh*sin(gs)*(5*msh + 4*mt + 8*mth))/4, (lsh*lth*(mth*cos(bs + gs) - mth*sin(bs + gs) + msh*cos(bs - gs) + 4*mt*cos(bs - gs) + 5*mth*cos(bs - gs) + mth*sin(bs - gs)))/4, -(lsh*lth*cos(bns - gs)*(msh + 2*mth))/4,                                                                                   Ith + (3*lsh^2*msh)/4 + lsh^2*mt + 2*lsh^2*mth, -(lsh^2*msh*cos(gns - gs))/8;
            -(lsh*msh*cos(gns))/4,                                              -(lsh*msh*sin(gns))/4,                                                                                                                                0,                                        0,                                                                                                     -(lsh^2*msh*cos(gns - gs))/8,                          Ith];
        
        B = [(lth*sin(bns)*(msh + mth)*dbns^2)/2 + (lsh*msh*sin(gns)*dgns^2)/4 - (dgs^2*lsh*sin(gs)*(5*msh + 4*mt + 8*mth))/4 - (dbs^2*lth*sin(bs)*(msh + 2*mt + 3*mth))/2;
            (lth*(msh*cos(bs) + 2*mt*cos(bs) + 2*mth*cos(bs) + mth*sin(bs))*dbs^2)/2 + (lsh*cos(gs)*(5*msh + 4*mt + 8*mth)*dgs^2)/4 - (dbns^2*lth*cos(bns)*(msh + mth))/2 - (dgns^2*lsh*msh*cos(gns))/4;
            (dbns^2*lth^2*mth*sin(bns - bs))/2 - (dbs^2*lth^2*mth*sin(2*bs))/4 - (dgs^2*lsh*lth*mth*sin(bs + gs))/4 - (dgs^2*lsh*lth*mth*cos(bs - gs))/4 + (dgs^2*lsh*lth*msh*sin(bs - gs))/4 + dgs^2*lsh*lth*mt*sin(bs - gs) + (5*dgs^2*lsh*lth*mth*sin(bs - gs))/4 - (dgs^2*lsh*lth*mth*cos(bs + gs))/4;
            (- (lsh*lth*msh*sin(bns - gs))/4 - (lsh*lth*mth*sin(bns - gs))/2)*dgs^2 - (dbs^2*lth^2*mth*sin(bns - bs))/2;
            -(lsh*(2*dbs^2*lth*mth*cos(bs + gs) + 2*dbs^2*lth*mth*sin(bs + gs) - 2*dbs^2*lth*mth*cos(bs - gs) - 2*dbns^2*lth*msh*sin(bns - gs) - 4*dbns^2*lth*mth*sin(bns - gs) + 2*dbs^2*lth*msh*sin(bs - gs) + 8*dbs^2*lth*mt*sin(bs - gs) + 10*dbs^2*lth*mth*sin(bs - gs) - dgns^2*lsh*msh*sin(gns - gs)))/8;
            -(dgs^2*lsh^2*msh*sin(gns - gs))/8];
        
    G =     [0;
        g*(2*msh + mt + 2*mth);
        (g*lth*(2*msh*sin(bs) - mth*cos(bs) + 2*mt*sin(bs) + 2*mth*sin(bs)))/2;
        -(g*lth*sin(bns)*(2*msh + mth))/2;
        (g*lsh*sin(gs)*(3*msh + 2*mt + 4*mth))/2;
        -(g*lsh*msh*sin(gns))/2];
    W = [ 1, 0, 0,  0,  0, 0;
        0, 1, 0,  0,  0, 0;
        0, 0, 1,  0, -1, 0;
        0, 0, 0, -1,  0, 1];
        Wdot = zeros(size(W));
        
        Fq =               [0;
            0;
            uShip - uSknee;
            uNShip - uNSknee;
            uSknee;
            uNSknee];
        end
        
        function [F] = GetReactionForces(KW, X)
        [M, B, G, W, Wdot, Fq] = DynEq(KW, X);
        dq = X(7:end);
        dim = size(W);
        A = [M -W.'; W zeros(dim(1))];
        b = [Fq - B - G; -Wdot*dq];
        sol = A\b;
        F = sol(end-3:end-2).';
        end
        
        function [Xdot] = Derivative(KW, t, X) %#ok
        [M, B, G, W, Wdot, Fq] = DynEq(KW, X);
        dq = X(7:end);
        [Wrows, ~] = size(W);
        A = [M -W.'; W zeros(Wrows)];
        b = [Fq - B - G; -Wdot*dq];
        sol = A\b;
        Xdot = [dq; sol(1:6)];
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
        value(3) = 2*pi; %NOTE: THIS EVENT IS NOT VALID BECAUSE THERE IS NO ALHPA
        % Event 6 - Sknee lock
        value(4) = X(3) - X(5);
        % Event 7 - NSknee lock
        value(5) = X(4) - X(6);
        end
        
        function [Xf, Lambda] = CalcImpact(KW, Xi)
        [M, ~, ~, ~, ~, ~] = DynEq(KW, Xi);
        mt = KW.to(1); lt = KW.to(2); It = KW.to(3); %#ok
        msh = KW.sh(1); lsh = KW.sh(2); Ish = KW.sh(3); %#ok
        mth = KW.th(1); lth = KW.th(2); Ith = KW.th(3); %#ok
        g = KW.grav; %#ok
        uShip = KW.Torques(1); uNShip = KW.Torques(2); uSknee = KW.Torques(3);... %#ok
            uNSknee = KW.Torques(4); %#ok
        x = Xi(1); y = Xi(2); bs = Xi(3); bns = Xi(4); gs = Xi(5); gns = Xi(6); %#ok
        dx = Xi(7); dy = Xi(8); dbs = Xi(9); dbns = Xi(10); dgs = Xi(11); dgns = Xi(12); %#ok
        Wc =  [ 1, 0, lth*cos(bs), -lth*cos(bns), lsh*cos(gs), -lsh*cos(gns);
            0, 1, lth*sin(bs), -lth*sin(bns), lsh*sin(gs), -lsh*sin(gns);
            0, 0, 1,  0, -1, 0;
            0, 0, 0, -1,  0, 1];
        A = [M, -Wc.';Wc, zeros(4)];
        b = [M*Xi(7:end).';zeros(4,1)];
        sol = A\b;
        NSanklePos = KW.GetPos(Xi,'NSankle');
        Xdotnew = sol(1:6);
%         if (dot(KW.GetVel([Xi(1:6).'; Xdotnew],'NSankle'),[0 1]) <= 0)
%             KW.BadLiftoff = 1;
%         end
        Xf = [NSanklePos ,Xi(4), Xi(3), Xi(6), Xi(5), 0, 0,...
            Xdotnew(4), Xdotnew(3), Xdotnew(6), Xdotnew(5)];
        Lambda = sol(7:end);
        
        end
        
        function Xf = HandleEvent(KW, iEvent, Xi, Floor)
        % 1 - leg contact
        % 2 - robot fell
        % 3 - abs(alpha) >= pi/2 (torso rotation)
        % 4 - Sknee lock
        % 5 - NSknee lock
        switch iEvent
            % Event 1 - Ground contact
            case 1
                [Xf, Lambda] = CalcImpact(KW, Xi);
                anklePos = KW.GetPos(Xf, 'NSankle');
                ankleVel = KW.GetVel(Xf, 'Sankle');
                alpha = Floor.SurfSlope(anklePos(1));
                n = [sin(alpha), cos(alpha)];
                LambdaN = dot(Lambda(1:2),n);
                ankleVelN = dot(ankleVel,n);
                if LambdaN < 0
                    KW.BadImpulse = 1;
                end
                if ankleVelN <= 0
                    KW.BadLiftoff = 1;
                end
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
        uShip = KW.Torques(1); uNShip = KW.Torques(2); uSknee = KW.Torques(3);...
            uNSknee = KW.Torques(4); %#ok
        x = X(1); y = X(2); bs = X(3); bns = X(4); gs = X(5); gns = X(6); %#ok
        dx = X(7); dy = X(8); dbs = X(9); dbns = X(10); dgs = X(11); dgns = X(12); 
        E  = (msh*((dy + (dgs*lsh*sin(gs))/2)*(dy - dbns*lth*sin(bns) + dbs*lth*sin(bs) - (dgns*lsh*sin(gns))/2 + dgs*lsh*sin(gs)) + (dx + (dgs*lsh*cos(gs))/2)*(dx - dbns*lth*cos(bns) + dbs*lth*cos(bs) - (dgns*lsh*cos(gns))/2 + dgs*lsh*cos(gs))))/2 - (g*(3*lsh*msh*cos(gs) - 2*mt*y - 4*mth*y - lsh*msh*cos(gns) - 4*msh*y + 2*lsh*mt*cos(gs) + 4*lsh*mth*cos(gs) + lth*mth*sin(bs) - 2*lth*msh*cos(bns) - lth*mth*cos(bns) + 2*lth*msh*cos(bs) + 2*lth*mt*cos(bs) + 2*lth*mth*cos(bs)))/2 + (mth*((dx + (dbs*lth*cos(bs))/2 + dgs*lsh*cos(gs))^2 + (dy + dgs*lsh*sin(gs) - (dbs*lth*cos(bs))/2)^2))/2 + (Ish*dbns^2)/2 + (Ish*dbs^2)/2 + (Ith*dgns^2)/2 + (Ith*dgs^2)/2 + (mth*((dx - (dbns*lth*cos(bns))/2 + dbs*lth*cos(bs) + dgs*lsh*cos(gs))^2 + (dy - (dbns*lth*sin(bns))/2 + dbs*lth*sin(bs) + dgs*lsh*sin(gs))^2))/2 + (mt*((dx + dbs*lth*cos(bs) + dgs*lsh*cos(gs))^2 + (dy + dbs*lth*sin(bs) + dgs*lsh*sin(gs))^2))/2 + (msh*((dx + (dgs*lsh*cos(gs))/2)^2 + (dy + (dgs*lsh*sin(gs))/2)^2))/2;
        end
        function KW = SetTorques(KW ,T)
        KW.Torques = T;
        end
    end
end



