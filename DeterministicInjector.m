classdef DeterministicInjector < handle
    % Deterministic injector for behavioural verification scenarios.
    % Fault behaviour is pre-defined rather than random.

    properties
        NoiseSigma (1,1) double = 0.0

        Enabled (1,1) logical = false
        FaultType (1,1) string = "NONE"   % "NONE","STUCK","BIAS","DRIFT","DROPOUT","SPIKE"
        FaultStartTime (1,1) double = inf
        FaultDuration (1,1) double = 0.0

        BiasMagnitude (1,1) double = 0.0
        DriftRateMagnitude (1,1) double = 0.0
        SpikeMagnitude (1,1) double = 0.0

        DropoutAsNaN (1,1) logical = true
    end

    properties (Access = private)
        tNow (1,1) double = 0.0
        faultActive (1,1) logical = false

        stuckValue double = NaN
        driftAccum (1,1) double = 0.0
        spikeDone (1,1) logical = false
    end

    methods
        function obj = DeterministicInjector(varargin)
            for k = 1:2:numel(varargin)
                obj.(varargin{k}) = varargin{k+1};
            end
        end

        function reset(obj)
            obj.tNow = 0.0;
            obj.faultActive = false;
            obj.stuckValue = NaN;
            obj.driftAccum = 0.0;
            obj.spikeDone = false;
        end

        function [y, meta] = apply(obj, x, dt)
            obj.tNow = obj.tNow + dt;

            % Fault active window
            if obj.Enabled && obj.tNow >= obj.FaultStartTime && ...
                    obj.tNow < (obj.FaultStartTime + obj.FaultDuration)
                if ~obj.faultActive
                    obj.startFault(x);
                end
            else
                obj.clearFault();
            end

            % Base measurement + optional noise
            y = x;
            if obj.NoiseSigma > 0
                y = y + obj.NoiseSigma * randn();
            end

            % Apply scripted fault
            if obj.faultActive
                switch obj.FaultType
                    case "STUCK"
                        y = obj.stuckValue;

                    case "BIAS"
                        y = y + obj.BiasMagnitude;

                    case "DRIFT"
                        obj.driftAccum = obj.driftAccum + obj.DriftRateMagnitude * dt;
                        y = y + obj.driftAccum;

                    case "DROPOUT"
                        if obj.DropoutAsNaN
                            y = NaN;
                        else
                            y = 0;
                        end

                    case "SPIKE"
                        if ~obj.spikeDone
                            y = y + obj.SpikeMagnitude;
                            obj.spikeDone = true;
                        end
                end
            end

            meta = struct( ...
                "faultActive", obj.faultActive, ...
                "faultType", obj.FaultType, ...
                "time", obj.tNow);
        end
    end

    methods (Access = private)
        function startFault(obj, xAtStart)
            obj.faultActive = true;
            obj.driftAccum = 0.0;
            obj.spikeDone = false;

            if obj.FaultType == "STUCK"
                obj.stuckValue = xAtStart;
            end
        end

        function clearFault(obj)
            obj.faultActive = false;
            obj.stuckValue = NaN;
            obj.driftAccum = 0.0;
            obj.spikeDone = false;
        end
    end
end