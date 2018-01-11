% %%%%%% % Renders Kneed-Walker % %%%%%% %
function status = RenderSim(Sim, X, Min, Max)
    % Get model positions
    Lankle  = Sim.Mod.GetPos(X,'Lankle');
    Lknee   = Sim.Mod.GetPos(X,'Lknee');
    Hip     = Sim.Mod.GetPos(X,'Hip');
    Head    = Sim.Mod.GetPos(X,'TorsoEnd');
    Rankle  = Sim.Mod.GetPos(X,'Rankle');
    Rknee   = Sim.Mod.GetPos(X,'Rknee');

    if isempty(Sim.Mod.RenderObj)
        % Model hasn't been rendered yet
        
        % Render links
        Sim.Mod.RenderObj.nL1 = DrawLink(Sim.Mod, Lknee(1), Lknee(2), Hip(1), Hip(2), 0, []);           % Support Thigh
        Sim.Mod.RenderObj.nL2 = DrawLink(Sim.Mod, Lankle(1), Lankle(2), Lknee(1), Lknee(2), 0, []);     % Support Shank
        Sim.Mod.RenderObj.nL3 = DrawLink(Sim.Mod, Hip(1), Hip(2), Head(1), Head(2), 0, []);             % Torso
        Sim.Mod.RenderObj.nL4 = DrawLink(Sim.Mod, Rknee(1), Rknee(2), Hip(1), Hip(2), 0, []);         % NSupport Thigh
        Sim.Mod.RenderObj.nL5 = DrawLink(Sim.Mod, Rankle(1), Rankle(2), Rknee(1), Rknee(2), 0, []); % NSupport Shank

        axis equal
%         axis([-0.5 1 0 1])
        RenderFloor(Sim.Env,Min,Max);
        
        % Finished rendering
        % Call function again to proceed with the code below
        RenderSim(Sim, X, Min,Max);
    else
        DrawLink(Sim.Mod, Lknee(1), Lknee(2), Hip(1), Hip(2), 0, Sim.Mod.RenderObj.nL1);           % Support Thigh
        DrawLink(Sim.Mod, Lankle(1), Lankle(2), Lknee(1), Lknee(2), 0, Sim.Mod.RenderObj.nL2);     % Support Shank
        DrawLink(Sim.Mod, Hip(1), Hip(2), Head(1), Head(2), 0, Sim.Mod.RenderObj.nL3);             % Torso
        DrawLink(Sim.Mod, Rknee(1), Rknee(2), Hip(1), Hip(2), 0, Sim.Mod.RenderObj.nL4);         % NSupport Thigh
        DrawLink(Sim.Mod, Rankle(1), Rankle(2), Rknee(1), Rknee(2), 0, Sim.Mod.RenderObj.nL5); % NSupport Shank
    end
    status = 0;

    % %%%%%%%% Auxiliary nested functions %%%%%%%% %
    % %%%% Draw Circle %%%% %
    % Draws a circle of radius R in pos (x,y,z)
    function [ KW ] = DrawCircle(KW, x, y, z, R, color, ID) %#ok
        coordX=zeros(1,KW.CircRes);
        coordY=zeros(1,KW.CircRes);
        coordZ=zeros(1,KW.CircRes);

        for r=1:KW.CircRes
            coordX(1,r)=x+R*cos(r/KW.CircRes*2*pi);
            coordY(1,r)=y+R*sin(r/KW.CircRes*2*pi);
            coordZ(1,r)=z;
        end

        h=patch(coordX,coordY,coordZ,color);
        set(h,'EdgeColor',color.^4);
        set(h,'LineWidth',2*KW.LineWidth);

        switch ID
            case 1
                KW.RenderObj.Cm1=h;
            case 2
                KW.RenderObj.Cm2=h;
            case 3
                KW.RenderObj.Cmh=h;
            otherwise
                return;
        end                    
    end

    % %%%% Draw Link %%%% %
    % Draws a link of from (x0,y0) to (x1,y1)
    function [ res ] = DrawLink(KW, x0, y0, x1, y1, z, Obj)
        if isempty(Obj)
            Length=sqrt((x1-x0)^2+(y1-y0)^2);
            Center=[(x0+x1)/2;
                    (y0+y1)/2];
            Orientation=atan2(y1-y0,x1-x0);

            res.Trans=hgtransform('Parent',gca);
            Txy=makehgtform('translate',[Center(1) Center(2) 0]);
            Rz=makehgtform('zrotate',Orientation-pi/2);

            coordX=zeros(1,2*KW.LinkRes+1);
            coordY=zeros(1,2*KW.LinkRes+1);
            coordZ=zeros(1,2*KW.LinkRes+1);

            x=0;
            y=Length-KW.link_width/2;
            for r=1:KW.LinkRes
                coordX(1,r)=x+KW.link_width/2*cos(r/KW.LinkRes*pi);
                coordY(1,r)=y+KW.link_width/2*sin(r/KW.LinkRes*pi);
                coordZ(1,r)=0;
            end

            y=KW.link_width/2;
            for r=KW.LinkRes:2*KW.LinkRes
                coordX(1,r+1)=x+KW.link_width/2*cos(r/KW.LinkRes*pi);
                coordY(1,r+1)=y+KW.link_width/2*sin(r/KW.LinkRes*pi);
                coordZ(1,r+1)=0;
            end

            res.Geom=patch(coordX,coordY,coordZ,KW.link_color); 
            set(res.Geom,'EdgeColor',[0 0 0]);
            set(res.Geom,'LineWidth',2*KW.LineWidth);

            set(res.Geom,'Parent',res.Trans);
            set(res.Trans,'Matrix',Txy*Rz);
        else
            Orientation=atan2(y1-y0,x1-x0);
            Length=sqrt((x1-x0)^2+(y1-y0)^2); %#ok

            Txy=makehgtform('translate',[x0 y0 z]);
            Rz=makehgtform('zrotate',Orientation-pi/2);
%             Sx=makehgtform('scale',[1,Length/(2*Sim.Mod.l),1]);
            set(Obj.Trans,'Matrix',Txy*Rz);
            res=1;
        end
    end

    % %%%% Draw Vector %%%% %
    % Draws a vector from x0 to x1
    function DrawVector(KW,x0,x1,zIndex,Color) %#ok
        VecScale=KW.link_width*0.75;
        Length=sqrt((x1(1)-x0(1))^2+(x1(2)-x0(2))^2);
        if Length<VecScale*4
            return;
        end

        Dir=(x1-x0)/Length;
        DirPerp=[-Dir(2); Dir(1)];

        Points=zeros(3,7);
        Points(1:2,1)=x0-DirPerp*VecScale/2;
        Points(1:2,2)=x0+DirPerp*VecScale/2;
        Points(1:2,3)=x1-Dir*VecScale*4+DirPerp*VecScale/2;
        Points(1:2,4)=x1-Dir*VecScale*4+DirPerp*VecScale*1.5;
        Points(1:2,5)=x1;
        Points(1:2,6)=x1-Dir*VecScale*4-DirPerp*VecScale*1.5;
        Points(1:2,7)=x1-Dir*VecScale*4-DirPerp*VecScale/2;
        Points(3,:)=zIndex*ones(1,7);

        patch(Points(1,:),Points(2,:),Points(3,:),Color);
    end

    function [Te] = RenderFloor(Te,Min,Max)
        FloorX=Min:Te.FloorStep:Max;
        VLStep=(Max-Min)/(Te.VertLines);
        
        FloorY=Te.Surf(FloorX);
        
        if Te.FloorLine==0 || ishandle(Te.FloorLine)==0
            % Draw horizontal line
            Te.FloorLine=line(FloorX,FloorY, 'LineWidth', 3*Te.LineWidth, 'Color', Te.FloorColor);
            
            % Draw vertical lines
            for v=1:Te.VertLines
                Te.FloorVLx(v)=Min+v*VLStep;
                Te.FloorVL(v)=line([Te.FloorVLx(v) Te.FloorVLx(v)-1/15],[Te.Surf(Te.FloorVLx(v)) Te.Surf(Te.FloorVLx(v))-1/5],...
                    'LineWidth', 2*Te.LineWidth, 'Color', Te.FloorColor);
            end
        else
            % Update horizontal line
            set(Te.FloorLine, 'XData', FloorX);
            set(Te.FloorLine, 'YData', FloorY);
            
            % Update vertical lines
            if Te.FloorVLx(1)<Min
                Te.FloorVLx(1:end-1)=Te.FloorVLx(2:end);
                Te.FloorVLx(end)=Te.FloorVLx(end-1)+VLStep;
                
                for v=1:Te.VertLines
                    set(Te.FloorVL(v), 'XData', [Te.FloorVLx(v) Te.FloorVLx(v)-1/15]);
                    set(Te.FloorVL(v), 'YData', [Te.Surf(Te.FloorVLx(v)) Te.Surf(Te.FloorVLx(v))-1/5]);
                end
            end
            
            if Te.FloorVLx(end)>Max
                Te.FloorVLx(2:end)=Te.FloorVLx(1:end-1);
                Te.FloorVLx(1)=Te.FloorVLx(2)-VLStep;
                
                for v=1:Te.VertLines
                    set(Te.FloorVL(v), 'XData', [Te.FloorVLx(v) Te.FloorVLx(v)-1/15]);
                    set(Te.FloorVL(v), 'YData', [Te.Surf(Te.FloorVLx(v)) Te.Surf(Te.FloorVLx(v))-1/5]);
                end
            end
        end
    end
end