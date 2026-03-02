%% AltitudeSensorSITL.m
% Dumb SITL altitude sensor emulator:
% - Takes an input value each step()
% - Optionally applies an Injector (noise/faults)
% - Outputs the noisy value
% - No filtering, no rate limiting, no validity checking, no health state machine

classdef AltitudeSensorSITL < handle
    properties
        Fs (1,1) double = 50;     % Sample rate [Hz]
        dt (1,1) double = 1/Fs;   % Sample period [s]

        % TODO: external injector (noise + faults)
        % Handle object with method:
        %   [y, inj] = apply(x, dt)
        Injector = [];
    end

    properties (SetAccess = private)
        AltitudeRaw (1,1) double = NaN;   % last input value received
        AltitudeOut (1,1) double = NaN;   % last output value after injector
    end

    methods
        function obj = AltitudeSensorSITL(varargin)
            % Name-value init
            for k = 1:2:numel(varargin)
                obj.(varargin{k}) = varargin{k+1};
            end
            if isempty(obj.dt) || obj.dt <= 0
                obj.dt = 1/obj.Fs;
            end
        end

        function altOut = step(obj, trueAltitude)
            % step() takes the "true" altitude and returns a noisy measurement.

            obj.AltitudeRaw = trueAltitude;

            y = trueAltitude;

            % Apply injector if present (adds noise / faults)
            if ~isempty(obj.Injector)
                % Injector is responsible for "how" noise is applied.
                % We ignore any extra metadata it returns.
                y = obj.Injector.apply(y, obj.dt);
                % If Injector.apply returns [y, inj], MATLAB will put the
                % whole first output into y, which is what we want.
            end

            obj.AltitudeOut = y;
            altOut = y;
        end
    end
end