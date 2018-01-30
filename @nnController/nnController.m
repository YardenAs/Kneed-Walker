classdef nnController < handle & matlab.mixin.Copyable
    
    properties
        % change nEvents and Order if a derivatives function is needed
        % i.e. activating the controller every period.
        % more info can be found in the documentation.
        nEvents         = 0;
        Order           = 0;
        net             = feedforwardnet([]);
    end
    
    methods
        % %%%%%% % Class constructor % %%%%%% %
        % varargin{1} = vector that consists the number of neurons on each
        % layer
        % varargin{2} = cell array of the transfer function of each layer
        function C = nnController(varargin)
        if nargin == 2
            C.net = feedforwardnet(varargin{1});
            tfs   = varargin{2};
            for ii = 1:length(varargin{1})
                C.net.layers{ii}.transferFcn = tfs{ii};
            end
        end
        end
        
        
        
        function Torques = Output(C, t, X) %#ok
        filtered_X = X([3,4,9,10]);
        Torques = C.net(filtered_X); % get the output from the NN
        end
    end
end