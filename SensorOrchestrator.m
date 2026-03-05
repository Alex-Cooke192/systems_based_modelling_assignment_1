%% SensorOrchestrator.m
% System states: "NORMAL", "WARN", "FAULT"
% Sensor states: "OK", "SUSPECT", "FAIL"
%
% Rules:
% - FAULT if any safety-critical sensor == "FAIL" OR redundancy mismatch duration >= T_fault
% - WARN  if any sensor == "SUSPECT" OR any non-critical sensor == "FAIL"
%         OR redundancy mismatch duration >= T_redundancyThreshold
% - Otherwise NORMAL

classdef SensorOrchestrator < handle
    properties
        Fs (1,1) double = 20
        dt (1,1) double = 0.05

        T_warn (1,1) double = 0.5
        T_fault (1,1) double = 0.7
        T_redundancyThreshold (1,1) double = 0.2

        % Time Threshold for FAIL -> WARN / FAULT recovery gating
        T_recover (1,1) double = 0.4
        % Time threshold for WARN -> CLEAR
        T_clear (1,1) double = 0.4

        mismatchThreshold (1,1) double = 0.1  % e.g. |altSlope - verticalSpeed|
        derivWindowSec (1,1) double = 0.5     % use altitude from ~0.5s ago

        logger = []
    end

    properties (SetAccess = private)
        State (1,1) string = "NORMAL"
        PrevState (1,1) string = "NORMAL"

        % Redundancy tracking
        RedundancyMismatch (1,1) logical = false
        RedundancyMismatchDuration (1,1) double = 0  % seconds

        % Altitude history for slope estimation
        AltitudeHistory (:,1) double = []

        % Latch to store if Log file has been made for this run
        LogEnabled = false

        % Timers for transitions between states to add hysteresis
        tWarnHold (1,1) double = 0
        tFaultHold (1,1) double = 0
        tClearHold (1,1) double = 0
    end

    properties (Access = private)

        % init flag
        hasFirstValid (1,1) logical = false

        % timers
        tSuspect (1,1) double = 0
        tFailed  (1,1) double = 0
        tClear   (1,1) double = 0
        tRecover (1,1) double = 0
        tRedMis  (1,1) double = 0
        tRedSev  (1,1) double = 0
    
        state (1,1) string = "INIT"
    end

    methods
        function obj = SensorOrchestrator(varargin)
            % Name-value init
            dtWasSet = false;

            for k = 1:2:numel(varargin)
                name = varargin{k};
                val  = varargin{k+1};

                if strcmpi(name,"dt"), dtWasSet = true; end
                obj.(name) = val;
            end

            if ~dtWasSet
                obj.dt = 1/obj.Fs;
            end
        end

        function reset(obj)
            obj.State = "NORMAL";
            obj.PrevState = "NORMAL";
            obj.RedundancyMismatch = false;
            obj.RedundancyMismatchDuration = 0;
            obj.AltitudeHistory = [];

            % keep private state machine consistent after reset
            obj.state = "INIT";
            obj.hasFirstValid = false;

            % reset hysteresis timers
            obj.tSuspect = 0; obj.tFailed = 0; obj.tClear = 0; obj.tRecover = 0;
            obj.tRedMis = 0; obj.tRedSev = 0;
        end

        function [state, logMsg, diagnostics] = step(obj, states, values, tt)
            obj.PrevState = obj.State;
            logMsg = "";

            % Keep external State in sync with internal state machine
            if obj.state ~= obj.State
                obj.state = obj.State;
            end

            % 1) Update altitude history
            obj.pushAltitude(values.Altitude);

            % 2) Compute redundancy mismatch (altitude slope vs vertical speed)
            obj.RedundancyMismatch = obj.computeRedundancyMismatch(values);

            % 3) Update redundancy mismatch duration timer
            if obj.RedundancyMismatch
                obj.RedundancyMismatchDuration = obj.RedundancyMismatchDuration + obj.dt;
            else
                obj.RedundancyMismatchDuration = 0;
            end

            % 4) Compute "facts"
            criticalStates = [ ...
                string(states.AltitudeSensor), ...
                string(states.AirspeedSensor), ...
                string(states.VerticalSpeedSensor), ...
                string(states.PitchSensor), ...
                string(states.RollSensor) ];

            nonCriticalStates = [ ...
                string(states.TemperatureSensor), ...
                string(states.PressureSensor) ];

            allStates = [criticalStates, nonCriticalStates];

            % tolerate either "FAIL" or "FAILED" for robustness
            isFail = (allStates == "FAIL") | (allStates == "FAILED");

            criticalFailNow    = any((criticalStates == "FAIL") | (criticalStates == "FAILED"));
            nonCriticalFailNow = any((nonCriticalStates == "FAIL") | (nonCriticalStates == "FAILED"));
            anyFailNow         = any(isFail);
            anySuspectNow      = any(allStates == "SUSPECT");

            % "all OK" means ALL are OK
            allOK              = all(allStates == "OK");

            % Redundancy "now" flags based on duration (kept for diagnostics)
            redundancyWarnNow  = obj.RedundancyMismatchDuration >= obj.T_redundancyThreshold;
            redundancyFaultNow = obj.RedundancyMismatchDuration >= obj.T_fault;

            % Hysteresis accumulators
            if anySuspectNow && ~anyFailNow
                obj.tSuspect = obj.tSuspect + obj.dt;
            else
                obj.tSuspect = 0;
            end

            if anyFailNow
                obj.tFailed = obj.tFailed + obj.dt;
            else
                obj.tFailed = 0;
            end

            % Redundancy timers: integrate the raw mismatch condition
            if obj.RedundancyMismatch
                obj.tRedMis = obj.tRedMis + obj.dt;
                obj.tRedSev = obj.tRedSev + obj.dt;  % no separate severity yet
            else
                obj.tRedMis = 0;
                obj.tRedSev = 0;
            end

            % CHANGED: split "clear" vs "recover" so FAULT doesn't flicker
            % between on/off
            % Clear (for WARN->NORMAL): truly clean system
            clearCond = allOK && ~obj.RedundancyMismatch;

            if clearCond
                obj.tClear = obj.tClear + obj.dt;
            else
                obj.tClear = 0;
            end

            % Recover (for FAULT exit): failures are gone and redundancy mismatch gone.
            % (does NOT require all sensors OK; allows landing in WARN if SUSPECT remains)
            recoverCond = (~anyFailNow) && (~obj.RedundancyMismatch);

            if recoverCond
                obj.tRecover = obj.tRecover + obj.dt;
            else
                obj.tRecover = 0;
            end

            % Convert redundancy timers to debounced warn/fault thresholds
            redWarnHold  = obj.tRedMis >= obj.T_warn;
            redFaultHold = obj.tRedSev >= obj.T_fault;

            switch obj.state
                case "INIT"
                    obj.hasFirstValid = true;
                    if obj.hasFirstValid
                        obj.state = "NORMAL";
                        obj.tSuspect = 0; obj.tFailed = 0; obj.tRedMis = 0; obj.tRedSev = 0;
                        obj.tClear = 0; obj.tRecover = 0;
                    end

                case "NORMAL"
                    if obj.tFailed >= obj.T_fault || redFaultHold
                        obj.state = "FAULT";
                    elseif obj.tSuspect >= obj.T_warn || redWarnHold
                        obj.state = "WARN";
                    end

                case "WARN"
                    if obj.tFailed >= obj.T_fault || redFaultHold
                        obj.state = "FAULT";
                    elseif obj.tClear >= obj.T_clear
                        obj.state = "NORMAL";
                    end

                case "FAULT"
                    % CHANGED: latch FAULT until recoverCond holds for T_recover
                    if obj.tRecover >= obj.T_recover
                        % after recovery time is met, decide where to go
                        % next based on other sensors states
                        if anySuspectNow || redWarnHold
                            obj.state = "WARN";
                        else
                            obj.state = "NORMAL";
                        end
                    end
                    % ------------------------------------------------------------------
            end

            % publish internal state machine state outward
            obj.State = obj.state;

            % Logging (only when state changes)
            if obj.State ~= obj.PrevState && ~isempty(obj.logger)
                prevState = obj.PrevState;
                time = string(datetime("now"), "dd:MM:yyyy HH:mm:ss.SSS");
                newState = obj.State;
                obj.logger.logStateChange(prevState, time, tt, newState);
            end

            % Diagnostics output
            diagnostics = struct();
            diagnostics.criticalFailNow = criticalFailNow;
            diagnostics.nonCriticalFailNow = nonCriticalFailNow;
            diagnostics.anySuspectNow = anySuspectNow;
            diagnostics.redundancyMismatch = obj.RedundancyMismatch;
            diagnostics.redundancyMismatchDuration = obj.RedundancyMismatchDuration;
            diagnostics.redundancyWarnNow = redundancyWarnNow;
            diagnostics.redundancyFaultNow = redundancyFaultNow;

            state = obj.State;
        end
    end

    methods (Access = private)
        function pushAltitude(obj, altitude)
            nKeep = max(2, ceil(obj.derivWindowSec / obj.dt) + 1);

            obj.AltitudeHistory(end+1,1) = altitude;

            if numel(obj.AltitudeHistory) > nKeep
                obj.AltitudeHistory(1:(end-nKeep)) = [];
            end
        end

        function mismatch = computeRedundancyMismatch(obj, values)
            nBack = round(obj.derivWindowSec / obj.dt);
            if numel(obj.AltitudeHistory) <= nBack
                mismatch = false;
                return;
            end

            altNow = obj.AltitudeHistory(end);
            altPast = obj.AltitudeHistory(end - nBack);

            altSlope = (altNow - altPast) / (nBack * obj.dt);
            vs = values.VerticalSpeed;

            mismatch = abs(altSlope - vs) > obj.mismatchThreshold;
        end
    end
end