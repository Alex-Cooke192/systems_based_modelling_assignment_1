%% AirspeedMonitor.m
% Health + conditioning state machine for airspeed measurements (m/s).
% States: OK, SUSPECT, FAILED
%
% Input: measured airspeed from AirspeedSensorSITL (may be noisy/NaN/faulty)
% Output: conditioned airspeed + health state + diagnostic flags

classdef AirspeedMonitor < handle
    properties
        % Timing
        Fs (1,1) double = 20
        dt (1,1) double = 0.05

        % Valid airspeed range (tune)
        Vmin (1,1) double = 0        % [m/s]
        Vmax (1,1) double = 200      % [m/s]

        NearThreshBand (1,1) double = 5   % [m/s] near range edge band

        % Conditioning
        LPFAlpha (1,1) double = 0.2       % low-pass smoothing
        MaxAccel (1,1) double = 25        % [m/s^2] accel limiter on conditioned output

        % State machine timing (s) 
        T_stable (1,1) double = 0.8
        T_fail   (1,1) double = 0.4
        T_recover(1,1) double = 1.5

        % Spike detection (instantaneous jump) 
        SpikeDelta (1,1) double = 12      % [m/s]
    end

    properties (SetAccess = private)
        % --- Outputs ---
        State (1,1) string = "OK"
        Raw (1,1) double = NaN
        Cond (1,1) double = NaN
        IsValid (1,1) logical = false
        HealthFlags struct
    end

    properties (Access = private)
        % --- Internal memory/timers ---
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
        function obj = AirspeedMonitor(varargin)
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

        function [vCond, state, flags] = step(obj, measuredAirspeed)
            % measuredAirspeed: sensor output (may be NaN/Inf/out-of-range)

            dt = obj.dt;

            raw = measuredAirspeed;
            obj.Raw = raw;

            % Validity checks
            isFinite = isfinite(raw);
            inRange = isFinite && (raw >= obj.Vmin) && (raw <= obj.Vmax);
            obj.IsValid = inRange;

            dropout = ~isFinite;

            spike = false;
            if isFinite && isfinite(obj.lastRaw)
                spike = abs(raw - obj.lastRaw) >= obj.SpikeDelta;
            end

            nearThresh = false;
            if isFinite
                nearThresh = (raw - obj.Vmin) <= obj.NearThreshBand || (obj.Vmax - raw) <= obj.NearThreshBand;
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

            % Conditioning (LPF + accel limit) 
            if ~isfinite(obj.lastCond)
                cond = rawForCond;
            else
                condLP = obj.lastCond + obj.LPFAlpha * (rawForCond - obj.lastCond);

                maxStep = obj.MaxAccel * dt; % [m/s] per sample
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

            % State machine (OK / SUSPECT / FAILED)
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
                    % FAILED -> SUSPECT once data returns cleanly (needs validating again)
                    if inRange && ~spike
                        if ~obj.sawDataReturn
                            obj.sawDataReturn = true;
                            obj.State = "SUSPECT";
                            obj.t_sinceNormal = 0;
                            obj.t_sinceRecoverCandidate = 0;
                        end
                    end
            end

            % Post-fail stricter revalidation window to OK
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
            vCond = obj.Cond;
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