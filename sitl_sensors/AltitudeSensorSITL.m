%% AltitudeSensorSITL.m
% Dumb SITL altitude sensor emulator:
% - Takes an input value each step()
% - Optionally applies an Injector (noise/faults)
% - Outputs the noisy value
% - No filtering, no rate limiting, no validity checking, no health state machine

classdef AltitudeSensorSITL < handle
    properties
        Fs (1,1) double = 50;     % Sample rate [Hz]
        dt (1,1) double = 0.02;   % Sample period [s]

        % TODO: external injector (noise + faults)
        % Handle object with method:
        %   [y, inj] = apply(x, dt)
        Injector = [];
    end

    properties (SetAccess = private)
        AltitudeRaw (1,1) double = NaN;   % last input value received
        AltitudeOut (1,1) double = NaN;   % last output value after injector
    end

    properties (Access = private)
        dtWasExplicitlySet (1,1) logical = false;
    end

    methods
        function obj = AltitudeSensorSITL(varargin)
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

        function [altOut, meta] = step(obj, trueAltitude)
            % step() takes the "true" altitude and returns a noisy/faulted measurement.
        
            obj.AltitudeRaw = trueAltitude;
        
            y = trueAltitude;
        
            % Default meta (no injector)
            meta = struct("faultActive", false, "faultType", "NONE");
        
            % Apply injector if present (adds noise / faults)
            if ~isempty(obj.Injector)
                [y, meta] = obj.Injector.apply(y, obj.dt);
            end
        
            obj.AltitudeOut = y;
            altOut = y;
        end
    end
end