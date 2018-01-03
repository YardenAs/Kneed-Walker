% %%%%%% % Renders Kneed-Walker % %%%%%% %
function status = Render(Sim, t, X, flag)
    % Get model positions
    Sankle  = Sim.Mod.GetPos(X,'Sankle');
    Sknee   = Sim.Mod.GetPos(X,'Sknee');
    Hip     = Sim.Mod.GetPos(X,'Hip');
    Head    = Sim.Mod.GetPos(X,'TorsoEnd');
    NSankle = Sim.Mod.GetPos(X,'NSankle');
    NSknee  = Sim.Mod.GetPos(X,'NSknee');

    if isempty(Sim.Mod.RenderObj)
        % Model hasn't been rendered yet
        
        % Render links
        Sim.Mod.RenderObj.nL1 = DrawLink(Sim.Mod, Sknee(1), Sknee(2), Hip(1), Hip(2), 0, []);           % Support Thigh
        Sim.Mod.RenderObj.nL2 = DrawLink(Sim.Mod, Sankle(1), Sankle(2), Sknee(1), Sknee(2), 0, []);     % Support Shank
        Sim.Mod.RenderObj.nL3 = DrawLink(Sim.Mod, Hip(1), Hip(2), Head(1), Head(2), 0, []);             % Torso
        Sim.Mod.RenderObj.nL4 = DrawLink(Sim.Mod, NSknee(1), NSknee(2), Hip(1), Hip(2), 0, []);         % NSupport Thigh
        Sim.Mod.RenderObj.nL5 = DrawLink(Sim.Mod, NSankle(1), NSankle(2), NSknee(1), NSknee(2), 0, []); % NSupport Shank

        axis equal
        axis([-0.5 1 0 1])
        
        % Finished rendering
        % Call function again to proceed with the code below
        Render(Sim.Mod, t, X, flag);
    else
        Sim.Mod.RenderObj.nL1 = DrawLink(Sim.Mod, Sknee(1), Sknee(2), Hip(1), Hip(2), 0, Sim.Mod.RenderObj.nL1);           % Support Thigh
        Sim.Mod.RenderObj.nL2 = DrawLink(Sim.Mod, Sankle(1), Sankle(2), Sknee(1), Sknee(2), 0, Sim.Mod.RenderObj.nL2);     % Support Shank
        Sim.Mod.RenderObj.nL3 = DrawLink(Sim.Mod, Hip(1), Hip(2), Head(1), Head(2), 0, Sim.Mod.RenderObj.nL3);             % Torso
        Sim.Mod.RenderObj.nL4 = DrawLink(Sim.Mod, NSknee(1), NSknee(2), Hip(1), Hip(2), 0, Sim.Mod.RenderObj.nL4);         % NSupport Thigh
        Sim.Mod.RenderObj.nL5 = DrawLink(Sim.Mod, NSankle(1), NSankle(2), NSknee(1), NSknee(2), 0, Sim.Mod.RenderObj.nL5); % NSupport Shank
    end
    status = 0;

    % %%%%%%%% Auxiliary nested functions %%%%%%%% %
    % %%%% Draw Circle %%%% %
    % Draws a circle of radius R in pos (x,y,z)
    function [ Sim.Mod ] = DrawCircle(KW, x, y, z, R, color, ID) %#ok
        coordX=zeros(1,Sim.Mod.CircRes);
        coordY=zeros(1,Sim.Mod.CircRes);
        coordZ=zeros(1,Sim.Mod.CircRes);

        for r=1:Sim.Mod.CircRes
            coordX(1,r)=x+R*cos(r/Sim.Mod.CircRes*2*pi);
            coordY(1,r)=y+R*sin(r/Sim.Mod.CircRes*2*pi);
            coordZ(1,r)=z;
        end

        h=patch(coordX,coordY,coordZ,color);
        set(h,'EdgeColor',color.^4);
        set(h,'LineWidth',2*Sim.Mod.LineWidth);

        switch ID
            case 1
                Sim.Mod.RenderObj.Cm1=h;
            case 2
                Sim.Mod.RenderObj.Cm2=h;
            case 3
                Sim.Mod.RenderObj.Cmh=h;
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

            coordX=zeros(1,2*Sim.Mod.LinkRes+1);
            coordY=zeros(1,2*Sim.Mod.LinkRes+1);
            coordZ=zeros(1,2*Sim.Mod.LinkRes+1);

            x=0;
            y=Length-Sim.Mod.link_width/2;
            for r=1:Sim.Mod.LinkRes
                coordX(1,r)=x+Sim.Mod.link_width/2*cos(r/Sim.Mod.LinkRes*pi);
                coordY(1,r)=y+Sim.Mod.link_width/2*sin(r/Sim.Mod.LinkRes*pi);
                coordZ(1,r)=0;
            end

            y=Sim.Mod.link_width/2;
            for r=Sim.Mod.LinkRes:2*Sim.Mod.LinkRes
                coordX(1,r+1)=x+Sim.Mod.link_width/2*cos(r/Sim.Mod.LinkRes*pi);
                coordY(1,r+1)=y+Sim.Mod.link_width/2*sin(r/Sim.Mod.LinkRes*pi);
                coordZ(1,r+1)=0;
            end

            res.Geom=patch(coordX,coordY,coordZ,Sim.Mod.link_color); 
            set(res.Geom,'EdgeColor',[0 0 0]);
            set(res.Geom,'LineWidth',2*Sim.Mod.LineWidth);

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
        VecScale=Sim.Mod.link_width*0.75;
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
end