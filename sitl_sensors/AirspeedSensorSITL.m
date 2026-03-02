%% AirspeedSensorSITL.m
% Dumb SITL airspeed sensor emulator:
% - Takes an input value each step()
% - Optionally applies an Injector (noise/faults)
% - Outputs the noisy value
% - No filtering, no rate limiting, no validity checking, no health state machine

classdef AirspeedSensorSITL < handle
    properties
        Fs (1,1) double = 20;     % Sample rate [Hz]
        dt (1,1) double = 0.05;   % Sample period [s]

        % TODO: external injector (noise + faults)
        % Handle object with method:
        %   [y, inj] = apply(x, dt)
        Injector = [];
    end

    properties (SetAccess = private)
        Raw (1,1) double = NaN;   % last input value received
        Out (1,1) double = NaN;   % last output value after injector
    end

    properties (Access = private)
        dtWasExplicitlySet (1,1) logical = false;
    end

    methods
        function obj = AirspeedSensorSITL(varargin)
            % Name-value init
            for k = 1:2:numel(varargin)
                name = varargin{k};
                val  = varargin{k+1};

                if strcmpi(name, "dt")
                    obj.dtWasExplicitlySet = true;
                end

                obj.(name) = val;
            end

            % If dt wasn't explicitly set, derive it from Fs
            if ~obj.dtWasExplicitlySet
                obj.dt = 1/obj.Fs;
            end
        end

        function obj = set.Fs(obj, newFs)
            obj.Fs = newFs;
            if ~obj.dtWasExplicitlySet
                obj.dt = 1/obj.Fs;
            end
        end

        function obj = set.dt(obj, newDt)
            obj.dt = newDt;
            obj.dtWasExplicitlySet = true;
        end

        function [y, meta] = step(obj, xTrue)
            % step() takes the "true" signal and returns a noisy/faulted measurement.
        
            % Store raw truth (optional but useful)
            obj.Raw = xTrue;
        
            % Default meta
            meta = struct("faultActive", false, "faultType", "NONE");
        
            % Start from truth
            y = xTrue;
        
            % Apply injector once (noise + faults)
            if ~isempty(obj.Injector)
                [y, meta] = obj.Injector.apply(y, obj.dt);   % obj.dt should be 1/obj.Fs
            end
        
            % Store output
            obj.Out = y;
        end
    end
end