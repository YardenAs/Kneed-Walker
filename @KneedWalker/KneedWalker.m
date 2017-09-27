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
        
        % Flags
        BadImpulse = [];
        BadLiftoff = [];
        
        % Control torques
        Torques = [0 0 0 0].'; % Ship, Rhip, Sknee, Rknee
        
        % Event index
        nEvents = 5; 
        % 1 - leg contact
        % 2 - robot fell
        % 3 - abs(alpha) >= pi/2 (torso rotation)
        % 4 - Lknee lock
        % 5 - Rknee lock
        
        % Render parameters
        link_width = 0.025;
        link_color= [0.1, 0.3, 0.8];
        RenderObj;
        LinkRes = 10;
        LineWidth = 1;
    end
    
    
    methods
        % Class cortructor
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
                    error('Couldn''t cortruct KneedWalker object, check your input');
            end
        end
                      
        function Pos = GetPos(KW, X, which)
        switch which
            case 'Lankle'
                x = X(1);
                y = X(2);
                Pos = [x y];
            case 'Lknee'
                rlk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                Pos = rlk;
            case 'Rankle'
                rlk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                rh  = rlk + [KW.th(2)*sin(X(4)), -KW.th(2)*cos(X(4))];          
                rrk = rh + [-KW.th(2)*sin(X(5)), KW.th(2)*cos(X(5))];
                rr2 = rrk + [-KW.sh(2)*sin(X(7)), KW.sh(2)*cos(X(7))];
                x = rr2(1);
                y = rr2(2);
                Pos = [x y];
            case 'Rknee'
                rlk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                rh  = rlk + [KW.th(2)*sin(X(4)), -KW.th(2)*cos(X(4))];          
                rrk = rh + [-KW.th(2)*sin(X(5)), KW.th(2)*cos(X(5))];
                x = rrk(1);
                y = rrk(2);
                Pos = [x y];
            case 'TorsoCOM'
                rlk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                rh  = rlk + [KW.th(2)*sin(X(4)), -KW.th(2)*cos(X(4))]; 
                rt  = rh + [-KW.to(2)/2*sin(X(3)), KW.to(2)/2*cos(X(3))];
                x = rt(1);
                y = rt(2);
                Pos = [x y];
            case 'TorsoEnd'
                rlk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                rh  = rlk + [KW.th(2)*sin(X(4)), -KW.th(2)*cos(X(4))]; 
                rt  = rh + [-KW.to(2)*sin(X(3)), KW.to(2)*cos(X(3))];
                x = rt(1);
                y = rt(2);
                Pos = [x y];
            case 'Hip'
                rlk = [X(1) + KW.sh(2)*sin(X(6)), X(2) - KW.sh(2)*cos(X(6))];
                rh  = rlk + [KW.th(2)*sin(X(4)), -KW.th(2)*cos(X(4))]; 
                Pos = rh;
            otherwise
                error('No such position');
                
        end
        end
        
        function Vel = GetVel(KW, X, which)
        switch which
            case 'Lankle'
                xdot =  X(8);
                ydot =  X(9);
                Vel = [xdot ydot];
            case 'Rankle'
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
            uLhip = KW.Torques(1); uRhip = KW.Torques(2); uLknee = KW.Torques(3);...
                uRknee = KW.Torques(4);
            x = X(1); y = X(2); a = X(3); bl = X(4); br = X(5); gl = X(6); gr = X(7); %#ok
            dx = X(8); dy = X(9); da = X(10); dbl = X(11); dbr = X(12); dgl = X(13); dgr = X(14); %#ok
   
            M = [                     2*msh + mt + 2*mth,                                                                  0,          -(lt*mt*cos(a))/2,                                                                                             (lth*cos(bl)*(msh + 2*mt + 3*mth))/2,            -(lth*cos(br)*(msh + mth))/2,                                                                                           (lsh*cos(gl)*(5*msh + 4*mt + 8*mth))/4,        -(lsh*msh*cos(gr))/4;
                                                       0,                                                 2*msh + mt + 2*mth,          -(lt*mt*sin(a))/2,                                                               (lth*(msh*sin(bl) - mth*cos(bl) + 2*mt*sin(bl) + 2*mth*sin(bl)))/2,            -(lth*sin(br)*(msh + mth))/2,                                                                                           (lsh*sin(gl)*(5*msh + 4*mt + 8*mth))/4,        -(lsh*msh*sin(gr))/4;
                                       -(lt*mt*cos(a))/2,                                                  -(lt*mt*sin(a))/2,           (mt*lt^2)/4 + It,                                                                                                       -(lt*lth*mt*cos(a - bl))/2,                                        0,                                                                                                       -(lsh*lt*mt*cos(a - gl))/2,                            0;
                    (lth*cos(bl)*(msh + 2*mt + 3*mth))/2, (lth*(msh*sin(bl) - mth*cos(bl) + 2*mt*sin(bl) + 2*mth*sin(bl)))/2, -(lt*lth*mt*cos(a - bl))/2,                                                                       Ish + lth^2*mt + (3*lth^2*mth)/2 - (lth^2*mth*sin(bl)^2)/2,             -(lth^2*mth*cos(br - bl))/2, (lsh*lth*(mth*cos(bl + gl) - mth*sin(bl + gl) + msh*cos(bl - gl) + 4*mt*cos(bl - gl) + 5*mth*cos(bl - gl) + mth*sin(bl - gl)))/4,                            0;
                           -(lth*cos(br)*(msh + mth))/2,                                      -(lth*sin(br)*(msh + mth))/2,                          0,                                                                                                     -(lth^2*mth*cos(br - bl))/2,                      (mth*lth^2)/4 + Ish,                                                                                         -(lsh*lth*cos(br - gl)*(msh + 2*mth))/4,                            0;
                  (lsh*cos(gl)*(5*msh + 4*mt + 8*mth))/4,                             (lsh*sin(gl)*(5*msh + 4*mt + 8*mth))/4, -(lsh*lt*mt*cos(a - gl))/2, (lsh*lth*(mth*cos(bl + gl) - mth*sin(bl + gl) + msh*cos(bl - gl) + 4*mt*cos(bl - gl) + 5*mth*cos(bl - gl) + mth*sin(bl - gl)))/4, -(lsh*lth*cos(br - gl)*(msh + 2*mth))/4,                                                                                   Ith + (3*lsh^2*msh)/4 + lsh^2*mt + 2*lsh^2*mth, -(lsh^2*msh*cos(gr - gl))/8;
                                   -(lsh*msh*cos(gr))/4,                                              -(lsh*msh*sin(gr))/4,                          0,                                                                                                                                0,                                        0,                                                                                                     -(lsh^2*msh*cos(gr - gl))/8,                          Ith];
 
            B = [(lt*mt*sin(a)*da^2)/2 + ((lth*msh*sin(br))/2 + (lth*mth*sin(br))/2)*dbr^2 + (- (lth*msh*sin(bl))/2 - lth*mt*sin(bl) - (3*lth*mth*sin(bl))/2)*dbl^2 + (lsh*msh*sin(gr)*dgr^2)/4 + (- (5*lsh*msh*sin(gl))/4 - lsh*mt*sin(gl) - 2*lsh*mth*sin(gl))*dgl^2;
               (- (lth*msh*cos(br))/2 - (lth*mth*cos(br))/2)*dbr^2 + ((lth*mth*sin(bl))/2 + (lth*msh*cos(bl))/2 + lth*mt*cos(bl) + lth*mth*cos(bl))*dbl^2 + ((5*lsh*msh*cos(gl))/4 + lsh*mt*cos(gl) + 2*lsh*mth*cos(gl))*dgl^2 - (da^2*lt*mt*cos(a))/2 - (dgr^2*lsh*msh*cos(gr))/4;
               -(lt*mt*(lth*sin(a - bl)*dbl^2 + lsh*sin(a - gl)*dgl^2))/2;
               (dbr^2*lth^2*mth*sin(br - bl))/2 - (dbl^2*lth^2*mth*sin(2*bl))/4 - (dgl^2*lsh*lth*mth*sin(bl + gl))/4 - (dgl^2*lsh*lth*mth*cos(bl - gl))/4 + (da^2*lt*lth*mt*sin(a - bl))/2 + (dgl^2*lsh*lth*msh*sin(bl - gl))/4 + dgl^2*lsh*lth*mt*sin(bl - gl) + (5*dgl^2*lsh*lth*mth*sin(bl - gl))/4 - (dgl^2*lsh*lth*mth*cos(bl + gl))/4;
               (- (lsh*lth*msh*sin(br - gl))/4 - (lsh*lth*mth*sin(br - gl))/2)*dgl^2 - (dbl^2*lth^2*mth*sin(br - bl))/2;
               -(lsh*(2*dbl^2*lth*mth*cos(bl + gl) + 2*dbl^2*lth*mth*sin(bl + gl) - 2*dbl^2*lth*mth*cos(bl - gl) - 4*da^2*lt*mt*sin(a - gl) - 2*dbr^2*lth*msh*sin(br - gl) - 4*dbr^2*lth*mth*sin(br - gl) + 2*dbl^2*lth*msh*sin(bl - gl) + 8*dbl^2*lth*mt*sin(bl - gl) + 10*dbl^2*lth*mth*sin(bl - gl) - dgr^2*lsh*msh*sin(gr - gl)))/8;
               -(dgl^2*lsh^2*msh*sin(gr - gl))/8];
                               
           G =    [               0;
               g*(2*msh + mt + 2*mth);
               -(g*lt*mt*sin(a))/2;
               (g*lth*(2*msh*sin(bl) - mth*cos(bl) + 2*mt*sin(bl) + 2*mth*sin(bl)))/2;
               -(g*lth*sin(br)*(2*msh + mth))/2;
               (g*lsh*sin(gl)*(3*msh + 2*mt + 4*mth))/2;
               -(g*lsh*msh*sin(gr))/2];
           if strcmp(KW.Support,'Left')
               W = [ 1, 0, 0, 0, 0, 0, 0;
                     0, 1, 0, 0, 0, 0, 0];
               Wdot = zeros(size(W));
           elseif strcmp(KW.Support,'Right')
               W =  [ 1, 0, 0, lth*cos(bl), -lth*cos(br), lsh*cos(gl), -lsh*cos(gr);
                      0, 1, 0, lth*sin(bl), -lth*sin(br), lsh*sin(gl), -lsh*sin(gr)];
               Wdot = [ 0, 0, 0, -dbl*lth*sin(bl),  dbr*lth*sin(br), -dgl*lsh*sin(gl),  dgr*lsh*sin(gr);
                        0, 0, 0,  dbl*lth*cos(bl), -dbr*lth*cos(br),  dgl*lsh*cos(gl), -dgr*lsh*cos(gr)];
           end

            Fq = [             0;
                               0;
                - uRhip - uLhip;
                  uLhip - uLknee;
                uRhip - uRknee;
                          uLknee;
                         uRknee];
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
            % 4 - Lknee lock
            % 5 - Rknee lock
            % Event 1 - Ground contact
            if strcmp(KW.Support,'Left')
                NSanklPos = KW.GetPos(X, 'Rankle');
            elseif strcmp(KW.Support,'Right')
                NSanklPos = KW.GetPos(X, 'Lankle');
            end
            value(1) = NSanklPos(2) - Floor.Surf(NSanklPos(1));
            % Event 2 - robot fell
            HipPos = KW.GetPos(X, 'Hip');
            if strcmp(KW.Support,'Left')
                SanklePos = KW.GetPos(X, 'Lankle');
            elseif strcmp(KW.Support,'Right')
                SanklePos = KW.GetPos(X, 'Rankle');
            end
            value(2) = HipPos(2) - SanklePos(2) - 0.6*(KW.sh(2) + KW.th(2));
            % Event 5 - abs(alpha) >= pi/2 (torso rotation)  
            value(3) = pi/2 - abs(X(3));
            % Event 6 - Lknee lock
            value(4) = X(4) - X(6);
            % Event 7 - Rknee lock
            value(5) = X(5) - X(7);
        end
        
        function [Xf, Lambda] = CalcImpact(KW, Xi)
            [M, ~, ~, Wc, ~, ~] = DynEq(KW, Xi);
            A = [M, -Wc.';Wc, zeros(2)];
            b = [M*Xi(8:end).';zeros(2,1)];
            sol = A\b;
            Xdotnew = sol(1:7);
            Xf = [Xi(1:7),Xdotnew.'];
            Lambda = sol(8:end);
        end
       
        function Xf = HandleEvent(KW, iEvent, Xi, Floor)
            % 1 - leg contact
            % 2 - robot fell
            % 3 - abs(alpha) >= pi/2 (torso rotation)
            % 4 - Lknee lock
            % 5 - Rknee lock
            switch iEvent
                % Event 1 - Ground contact
                case 1
                    if strcmp(KW.Support,'Left')
                        KW.Support = 'Right';
                        NSanklePos = KW.GetPos(Xi,'Rankle');
                    elseif strcmp(KW.Support,'Right')
                        KW.Support = 'Left';
                        NSanklePos = KW.GetPos(Xi,'Lankle');
                    end
                    [Xf, Lambda] = CalcImpact(KW, Xi);
                    if strcmp(KW.Support,'Left')
                        SankleVel = KW.GetVel(Xf,'Rankle');
                    elseif strcmp(KW.Support,'Right')
                        SankleVel = KW.GetVel(Xf,'Lankle');
                    end
                    alpha = Floor.SurfSlope(NSanklePos(1));
                    n = [sin(alpha), cos(alpha)];
                    LambdaN = dot(Lambda,n);
                    SankleVelN = dot(SankleVel,n); 
                    if LambdaN < 0
                        KW.BadImpulse = 1;
                    end
                    if SankleVelN <= 0
                        KW.BadLiftoff = 1;
                    end
                    % Event 2 - Robot fell
                case 2
                    Xf = Xi;
                    % Event 3 - abs(alpha) >= pi/2 (torso rotation)
                case 3
                    Xf = Xi;
                    % Event 4 - Lknee lock
                case 4
                    Xf = Xi;
                    % Event 5 - Rknee lock
                case 5
                    Xf = Xi;
            end
        end
        
        function E = GetEnergy(KW, X)
            mt = KW.to(1); lt = KW.to(2); It = KW.to(3);
            msh = KW.sh(1); lsh = KW.sh(2); Ish = KW.sh(3);
            mth = KW.th(1); lth = KW.th(2); Ith = KW.th(3);
            g = KW.grav;
            uShip = KW.Torques(1); uRhip = KW.Torques(2); uSknee = KW.Torques(3);... %%ok
                uRknee = KW.Torques(4); %#ok
            x = X(1); y = X(2); a = X(3); bl = X(4); br = X(5); gl = X(6); gr = X(7); %#ok
            dx = X(8); dy = X(9); da = X(10); dbl = X(11); dbr = X(12); dgl = X(13); dgr = X(14); 
            E  = (msh*((dy + (dgl*lsh*sin(gl))/2)*(dy - dbr*lth*sin(br) + dbl*lth*sin(bl) - (dgr*lsh*sin(gr))/2 + dgl*lsh*sin(gl)) + (dx + (dgl*lsh*cos(gl))/2)*(dx - dbr*lth*cos(br) + dbl*lth*cos(bl) - (dgr*lsh*cos(gr))/2 + dgl*lsh*cos(gl))))/2 + (g*(4*msh*y + 2*mt*y + 4*mth*y + lsh*msh*cos(gr) - 3*lsh*msh*cos(gl) - 2*lsh*mt*cos(gl) - 4*lsh*mth*cos(gl) - lth*mth*sin(bl) + lt*mt*cos(a) + 2*lth*msh*cos(br) + lth*mth*cos(br) - 2*lth*msh*cos(bl) - 2*lth*mt*cos(bl) - 2*lth*mth*cos(bl)))/2 + (mth*((dx + (dbl*lth*cos(bl))/2 + dgl*lsh*cos(gl))^2 + (dy + dgl*lsh*sin(gl) - (dbl*lth*cos(bl))/2)^2))/2 + (It*da^2)/2 + (Ish*dbr^2)/2 + (Ish*dbl^2)/2 + (Ith*dgr^2)/2 + (Ith*dgl^2)/2 + (mt*((dx - (da*lt*cos(a))/2 + dbl*lth*cos(bl) + dgl*lsh*cos(gl))^2 + (dy - (da*lt*sin(a))/2 + dbl*lth*sin(bl) + dgl*lsh*sin(gl))^2))/2 + (mth*((dx - (dbr*lth*cos(br))/2 + dbl*lth*cos(bl) + dgl*lsh*cos(gl))^2 + (dy - (dbr*lth*sin(br))/2 + dbl*lth*sin(bl) + dgl*lsh*sin(gl))^2))/2 + (msh*((dx + (dgl*lsh*cos(gl))/2)^2 + (dy + (dgl*lsh*sin(gl))/2)^2))/2;
        end
        function KW = SetTorques(KW ,T)
            KW.Torques = T;
        end
    end
end     



