%% EngineTempMonitor.m
% Health + conditioning state machine for engine temperature (degC).
% States: OK, SUSPECT, FAILED
%
% Input: measured temperature from EngineTempSensorSITL (may be noisy/NaN/faulty)
% Output: conditioned temperature, health state and diagnostic flags

classdef EngineTempMonitor < handle
    properties
        % Timing
        Fs (1,1) double = 10
        dt (1,1) double = 0.1

        % Valid range
        TempMin (1,1) double = -40     % [degC]
        TempMax (1,1) double = 250     % [degC]

        NearThreshBand (1,1) double = 5     % [degC]

        % Conditioning
        LPFAlpha (1,1) double = 0.15

        % Rate limit on conditioned output
        MaxRiseRate (1,1) double = 20       % [degC/s]

        % State machine timing (s)
        T_stable (1,1) double = 1.0
        T_fail   (1,1) double = 0.6
        T_recover(1,1) double = 2.0

        % Spike detection
        SpikeDelta (1,1) double = 10        % [degC]
    end

    properties (SetAccess = private)
        State (1,1) string = "OK"
        Raw (1,1) double = NaN
        Cond (1,1) double = NaN
        IsValid (1,1) logical = false
        HealthFlags struct
    end

    properties (Access = private)
        t_sinceNormal double = 0
        t_sinceInvalid double = 0
        t_sinceRecoverCandidate double = 0

        lastRaw double = NaN
        lastCond double = NaN
        lastValidRaw double = NaN

        sawDataReturn logical = false

        dtWasExplicitlySet (1,1) logical = false
    end

    methods
        function obj = EngineTempMonitor(varargin)
            % Name-value init
            for k = 1:2:numel(varargin)
                name = varargin{k};
                val  = varargin{k+1};

                if strcmpi(name, "dt")
                    obj.dtWasExplicitlySet = true;
                end

                obj.(name) = val;
            end

            if ~obj.dtWasExplicitlySet
                obj.dt = 1/obj.Fs;
            end

            obj.HealthFlags = obj.defaultFlags();
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

        function [tempCond, state, flags] = step(obj, measuredTempDegC)
            dt = obj.dt;

            raw = measuredTempDegC;
            obj.Raw = raw;

            % Validity checks
            isFinite = isfinite(raw);
            inRange = isFinite && (raw >= obj.TempMin) && (raw <= obj.TempMax);
            obj.IsValid = inRange;

            dropout = ~isFinite;

            spike = false;
            if isFinite && isfinite(obj.lastRaw)
                spike = abs(raw - obj.lastRaw) >= obj.SpikeDelta;
            end

            nearThresh = false;
            if isFinite
                nearThresh = (raw - obj.TempMin) <= obj.NearThreshBand || (obj.TempMax - raw) <= obj.NearThreshBand;
            end
            noiseExcursionNearThreshold = nearThresh && (spike || ~inRange);

            % Timers
            if inRange && ~spike
                obj.t_sinceNormal = obj.t_sinceNormal + dt;
            else
                obj.t_sinceNormal = 0;
            end

            if ~inRange || dropout
                obj.t_sinceInvalid = obj.t_sinceInvalid + dt;
            else
                obj.t_sinceInvalid = 0;
            end

            % Conditioning input selection (hold last valid)
            if inRange
                obj.lastValidRaw = raw;
            end

            rawForCond = raw;
            if ~inRange
                rawForCond = obj.lastValidRaw;
            end

            % Conditioning (LPF + rate limit)
            if ~isfinite(obj.lastCond)
                cond = rawForCond;
            else
                condLP = obj.lastCond + obj.LPFAlpha * (rawForCond - obj.lastCond);

                maxStep = obj.MaxRiseRate * dt; % [degC] per sample
                d = condLP - obj.lastCond;
                d = max(-maxStep, min(maxStep, d));
                cond = obj.lastCond + d;
            end

            obj.Cond = cond;

            % Flags
            flags = obj.defaultFlags();
            flags.valid = inRange;
            flags.outOfRange = isFinite && ~inRange;
            flags.dropout = dropout;
            flags.spike = spike;
            flags.noiseExcursionNearThreshold = noiseExcursionNearThreshold;

            % State machine
            switch obj.State
                case "OK"
                    if flags.outOfRange || flags.dropout || flags.spike || flags.noiseExcursionNearThreshold
                        obj.State = "SUSPECT";
                        obj.t_sinceNormal = 0;
                    end

                    if obj.t_sinceInvalid >= obj.T_fail
                        obj.State = "FAILED";
                        obj.sawDataReturn = false;
                        obj.t_sinceRecoverCandidate = 0;
                    end

                case "SUSPECT"
                    if obj.t_sinceInvalid >= obj.T_fail
                        obj.State = "FAILED";
                        obj.sawDataReturn = false;
                        obj.t_sinceRecoverCandidate = 0;
                    end

                    if obj.t_sinceNormal >= obj.T_stable
                        obj.State = "OK";
                    end

                case "FAILED"
                    % FAILED -> SUSPECT once clean data returns (needs validating again)
                    if inRange && ~spike
                        if ~obj.sawDataReturn
                            obj.sawDataReturn = true;
                            obj.State = "SUSPECT";
                            obj.t_sinceNormal = 0;
                            obj.t_sinceRecoverCandidate = 0;
                        end
                    end
            end

            % Post-fail revalidation to OK
            if obj.State == "SUSPECT" && obj.sawDataReturn
                if inRange && ~spike
                    obj.t_sinceRecoverCandidate = obj.t_sinceRecoverCandidate + dt;
                else
                    obj.t_sinceRecoverCandidate = 0;
                end

                if obj.t_sinceRecoverCandidate >= obj.T_recover
                    obj.State = "OK";
                    obj.sawDataReturn = false;
                    obj.t_sinceRecoverCandidate = 0;
                end
            end

            % Save history
            obj.lastRaw = raw;
            obj.lastCond = cond;

            % Publish
            obj.HealthFlags = flags;
            tempCond = obj.Cond;
            state = obj.State;
        end
    end

    methods (Access = private)
        function flags = defaultFlags(~)
            flags = struct( ...
                "valid", false, ...
                "outOfRange", false, ...
                "dropout", false, ...
                "spike", false, ...
                "noiseExcursionNearThreshold", false);
        end
    end
end