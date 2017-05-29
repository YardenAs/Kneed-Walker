classdef KneedWalker
    %KneedWalker is a class that defines a kneed biped robot.
    
    
    %  Important note: don't forget to set the last value of the initial
    %  state vector to the index that corresponds with the initial 
    %  condition's dynamic model.
    %  
    %  Yarden As (2017)
   
    
    properties
        initstate
        % the initial state of the system. the last element should hold the
        % event index (A.K.A ie) that corresponds with the dynamic behavior
        % at the initial state.
        params
        controls
        dynamics    % a function that holds all the dynamic behaviors of the system.
        ground      % the ground's geometry.
        event       % a function that holds all the discrete events of the system.
        event_relay % a function handle that returns the state after an event.    
        simspan
    end
    
    
    methods
        function obj = walker(Initstate, Dynamics, Params, Controls,...
                       Ground, Event, Event_relay, Simspan)
                       
        if nargin == 8
            obj.initstate   = Initstate;
            obj.dynamics    = Dynamics;
            obj.params      = Params;
            obj.controls    = Controls;
            obj.ground      = Ground;
            obj.event       = Event;
            obj.event_relay = Event_relay;
            obj.simspan     = Simspan;
        end
        if (~isempty(obj.dynamics)) && (~isempty(obj.params)) && (~isempty(obj.controls))
        % this should take all the dynamic behaviors and load the bots
        % parameters and controls onto them
        dynamic_models = obj.dynamics;
        for i = 1:length(obj.dynamics)
            dynamic_models{i} = @(t,y) dynamics_models{i}(t,y,obj.params,obj.controls);    
        end
        end
        end
        
        function [tout, stateout] = sim(obj)
        opt = odeset('Events',obj.event);
        init_state = obj.initstate(1:end-1);
        dynamic_model = obj.dynamics(obj.initstate(end));
        tstart = obj.simspan(1); tfinal = obj.simspan(2);
        tout = tstart; stateout = init_state.';
        t = 0; ie = [];
        while t < tfinal 
            [t,state,~,~,ie] = ode45(dynamic_model,[tstart tfinal],init_state,opt);
            tout = [tout; t(2:end)];                          
            stateout = [stateout; state(2:end,1:end)];
            if ~isempty(ie)
                % get the dynamic equations that correspond 
                % to the event index (ie).
                dynamic_model = obj.dynamics(ie);
                % calculate the state after the discrete event that
                % corresponds to the event index (ie).
                relay = obj.event_relay(ie); 
                init_state = relay(tout(end),stateout(end,:)).';
                tstart = tout(end);
            end
        end
        end
        
        
    end
    
end

