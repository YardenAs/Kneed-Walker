classdef nnController < handle & matlab.mixin.Copyable
    
    properties
        % change nEvents and Order if a derivatives function is needed
        % i.e. activating the controller every period.
        % more info can be found in the documentation.
        nEvents         = 0; 
        Order           = 0; 
        % hiddenSizes accepts a vector that hold the number of neurons
        % in every layer. transfer Fcn accepts a cell array of activation 
        % function for every layer (including output)
        net             = feedforwardnet(0);
    end
    
    methods
        % %%%%%% % Class constructor % %%%%%% %
        function C = nnController(varargin)
        if nargin ~= 2
            error('controller accepts 2 arguments in');
        end
        switch varargin{1}
            case 'nn'
                C.net = feedforwardnet(varargin{2});
                for ii = 1:length(C.hiddenSizes)
                    C.net.layers{ii}.transferFcn = C.transferFcn{ii};
                end
        end
        end  
        
        function Torques = Output(C, t, X) %#ok
            Torques = C.net(X); % get the output from the NN
        end
    end
end