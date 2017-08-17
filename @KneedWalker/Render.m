% %%%%%% % Renders Kneed-Walker % %%%%%% %
function status = Render(KW, t, X, flag)
    % Get model positions
    Sankle  = KW.GetPos(X,'Sankle');
    Sknee   = KW.GetPos(X,'Sknee');
    Hip     = KW.GetPos(X,'Hip');
    Head    = KW.GetPos(X,'TorsoEnd');
    NSankle = KW.GetPos(X,'NSankle');
    NSknee  = KW.GetPos(X,'NSknee');

    if isempty(KW.RenderObj)
        % Model hasn't been rendered yet
        
        % Render links
        KW.RenderObj.nL1 = DrawLink(KW, Sknee(1), Sknee(2), Hip(1), Hip(2), 0, []);           % Support Thigh
        KW.RenderObj.nL2 = DrawLink(KW, Sankle(1), Sankle(2), Sknee(1), Sknee(2), 0, []);     % Support Shank
        KW.RenderObj.nL3 = DrawLink(KW, Hip(1), Hip(2), Head(1), Head(2), 0, []);             % Torso
        KW.RenderObj.nL4 = DrawLink(KW, NSknee(1), NSknee(2), Hip(1), Hip(2), 0, []);         % NSupport Thigh
        KW.RenderObj.nL5 = DrawLink(KW, NSankle(1), NSankle(2), NSknee(1), NSknee(2), 0, []); % NSupport Shank

        axis equal
        axis([-0.5 1 0 1])
        
        % Finished rendering
        % Call function again to proceed with the code below
        Render(KW, t, X, flag);
    else
        KW.RenderObj.nL1 = DrawLink(KW, Sknee(1), Sknee(2), Hip(1), Hip(2), 0, KW.RenderObj.nL1);           % Support Thigh
        KW.RenderObj.nL2 = DrawLink(KW, Sankle(1), Sankle(2), Sknee(1), Sknee(2), 0, KW.RenderObj.nL2);     % Support Shank
        KW.RenderObj.nL3 = DrawLink(KW, Hip(1), Hip(2), Head(1), Head(2), 0, KW.RenderObj.nL3);             % Torso
        KW.RenderObj.nL4 = DrawLink(KW, NSknee(1), NSknee(2), Hip(1), Hip(2), 0, KW.RenderObj.nL4);         % NSupport Thigh
        KW.RenderObj.nL5 = DrawLink(KW, NSankle(1), NSankle(2), NSknee(1), NSknee(2), 0, KW.RenderObj.nL5); % NSupport Shank
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
%             Sx=makehgtform('scale',[1,Length/(2*KW.l),1]);
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
end