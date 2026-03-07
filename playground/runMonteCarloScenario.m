%% runBehaviouralScenario.m
clear; clc;

scenarioId = "MCV-2";   % choose: MCV-1, MCV-2, MCV-3, (MCV-4?)

%% Simulation parameters
N = 1000;       % Number of trials
Fs = 10;
dt = 1/Fs;
Tsim = 20;
t = 0:dt:Tsim;

% Number of runs where warn/fault occurs during at least one timestep
warnOccured = zeros(N, 1); 
faultOccured = zeros(N, 1);

warnEvents = 0;
faultEvents = 0;

warnPresent = zeros(1, length(t)); 
warnTimesteps = zeros(N, length(t)); 

%% Instantiate sensors
for run = 1:N

    altSensor   = AltitudeSensorSITL("Fs", Fs);
    vsSensor    = VerticalSpeedSensorSITL("Fs", Fs);
    asSensor    = AirspeedSensorSITL("Fs", Fs);
    pitchSensor = PitchSensorSITL("Fs", Fs);
    rollSensor  = RollSensorSITL("Fs", Fs);
    tempSensor  = EngineTempSensorSITL("Fs", Fs);
    oilSensor   = OilPressureSensorSITL("Fs", Fs);
    
    %% Default: no faults
    switch scenarioId
        case "MCV-1"
            altSensor.Injector   = SimpleInjector();
            vsSensor.Injector    = SimpleInjector();
            asSensor.Injector    = SimpleInjector();
            pitchSensor.Injector = SimpleInjector();
            rollSensor.Injector  = SimpleInjector();
            tempSensor.Injector  = SimpleInjector();
            oilSensor.Injector   = SimpleInjector();

        case "MCV-2"
            altSensor.Injector   = DeterministicInjector();
            vsSensor.Injector    = DeterministicInjector();
            asSensor.Injector    = DeterministicInjector();
            pitchSensor.Injector = DeterministicInjector();
            rollSensor.Injector  = DeterministicInjector();
            tempSensor.Injector  = DeterministicInjector();
            oilSensor.Injector   = DeterministicInjector();

        case "MCV-3"
            % Do some stuff
    end

    %% Model for noise
    NoiseMu = 0; 
    NoiseSigma = NaN; % empty for now - decided by scenario 
    
    %% Apply scenario-specific noise
    switch scenarioId
        case "MCV-1"
            NoiseSigma = 0.3; 
            altSensor.Injector = SimpleInjector(...
                "NoiseSigma", 0.3,...
                "FaultRatePerSecond", 0.0...
                ); 

            vsSensor.Injector = SimpleInjector(...
                "NoiseSigma", 0.1,...
                "FaultRatePerSecond", 0.0...
                ); 

            asSensor.Injector = SimpleInjector(...
                "NoiseSigma", 0.1,...
                "FaultRatePerSecond", 0.0...
                ); 

            pitchSensor.Injector = SimpleInjector(...
                "NoiseSigma", 0.3,...
                "FaultRatePerSecond", 0.0...
                ); 

            rollSensor.Injector = SimpleInjector(...
                "NoiseSigma", 0.3,...
                "FaultRatePerSecond", 0.0...
                ); 

            tempSensor.Injector = SimpleInjector(...
                "NoiseSigma", 0.5,...
                "FaultRatePerSecond", 0.0...
                ); 

            oilSensor.Injector = SimpleInjector(...
                "NoiseSigma", 0.2,...
                "FaultRatePerSecond", 0.0...
                ); 

        case "MCV-2"
            failureClasses = ["BIAS", "DRIFT", "STUCK", "DROPOUT"];
            idx = randi(length(failureClasses));
            FaultType = failureClasses(idx);
            
            commonArgs = { ...
                "NoiseSigma", 0.3, ...
                "Enabled", true, ...
                "FaultType", FaultType, ...
                "FaultStartTime", 5.0, ...
                "FaultDuration", 0.8 ...
            };
            
            specificArgs = {};
            
            switch FaultType
                case "BIAS"
                    specificArgs = { ...
                        "BiasMagnitude", 1 + 4*rand() ...
                    };
            
                case "DRIFT"
                    specificArgs = { ...
                        "DriftRateMagnitude", 0.5 + 2*rand() ...
                    };
            
                case "SPIKE"
                    specificArgs = { ...
                        "SpikeMagnitude", 5 + 15*rand() ...
                    };
            
                case "DROPOUT"
                    specificArgs = { ...
                        "DropoutAsNaN", true ...
                    };
            
                case "STUCK"
                    specificArgs = {};
            end

            altSensor.Injector = DeterministicInjector(commonArgs{:}, specificArgs{:});
            vsSensor.Injector = DeterministicInjector(commonArgs{:}, specificArgs{:});
            asSensor.Injector = DeterministicInjector(commonArgs{:}, specificArgs{:}); 
            pitchSensor.Injector = DeterministicInjector(commonArgs{:}, specificArgs{:});
            rollSensor.Injector = DeterministicInjector(commonArgs{:}, specificArgs{:});
            tempSensor.Injector = DeterministicInjector(commonArgs{:}, specificArgs{:});
            oilSensor.Injector = DeterministicInjector(commonArgs{:}, specificArgs{:});

        case "MCV-3"
            altSensor.Injector = DeterministicInjector(...
                "NoiseSigma", 0.3, ...
                "FaultRatePerSecond", 0.1, ...
                "W_Stuck", 1, ...
                "W_Bias", 1, ...
                "W_Drift", 1, ...
                "W_Dropout", 1, ...
                "W_Spike", 1 ...
                ); 

            vsSensor.Injector = DeterministicInjector(...
                "NoiseSigma", 0.3, ...
                "FaultRatePerSecond", 0.1, ...
                "W_Stuck", 1, ...
                "W_Bias", 1, ...
                "W_Drift", 1, ...
                "W_Dropout", 1, ...
                "W_Spike", 1 ...
                );

            asSensor.Injector = DeterministicInjector(...
                "NoiseSigma", 0.3, ...
                "FaultRatePerSecond", 0.1, ...
                "W_Stuck", 1, ...
                "W_Bias", 1, ...
                "W_Drift", 1, ...
                "W_Dropout", 1, ...
                "W_Spike", 1 ...
                );

            pitchSensor.Injector = DeterministicInjector(...
                "NoiseSigma", 0.3, ...
                "FaultRatePerSecond", 0.1, ...
                "W_Stuck", 1, ...
                "W_Bias", 1, ...
                "W_Drift", 1, ...
                "W_Dropout", 1, ...
                "W_Spike", 1 ...
                );

            rollSensor.Injector = DeterministicInjector(...
                "NoiseSigma", 0.3, ...
                "FaultRatePerSecond", 0.1, ...
                "W_Stuck", 1, ...
                "W_Bias", 1, ...
                "W_Drift", 1, ...
                "W_Dropout", 1, ...
                "W_Spike", 1 ...
                );

            tempSensor.Injector = DeterministicInjector(...
                "NoiseSigma", 0.3, ...
                "FaultRatePerSecond", 0.1, ...
                "W_Stuck", 1, ...
                "W_Bias", 1, ...
                "W_Drift", 1, ...
                "W_Dropout", 1, ...
                "W_Spike", 1 ...
                );

            oilSensor.Injector = DeterminsticInjector(...
                "NoiseSigma", 0.3, ...
                "FaultRatePerSecond", 0.1, ...
                "W_Stuck", 1, ...
                "W_Bias", 1, ...
                "W_Drift", 1, ...
                "W_Dropout", 1, ...
                "W_Spike", 1 ...
                );
    
        otherwise
            error("Unknown scenarioId: %s", scenarioId);
    end
    
    %% Instantiate monitors
    altMon   = AltitudeMonitor("Fs", Fs);
    vsMon    = VerticalSpeedMonitor("Fs", Fs);
    asMon    = AirspeedMonitor("Fs", Fs);
    pitchMon = PitchMonitor("Fs", Fs);
    rollMon  = RollMonitor("Fs", Fs);
    tempMon  = EngineTempMonitor("Fs", Fs);
    oilMon   = OilPressureMonitor("Fs", Fs);
    
    %% Truth signals
    trueAlt   = @(tt) 1000 + 2*tt + 20*sin(0.5*tt);
    trueVS    = @(tt) 2 + 10*cos(0.5*tt);
    trueAS    = @(tt) 60 + 5*sin(0.8*tt);
    truePitch = @(tt) 5*sin(0.6*tt);
    trueRoll  = @(tt) 10*sin(0.4*tt);
    trueTemp  = @(tt) 80 + 0.5*tt;
    trueOil   = @(tt) 400 + 10*sin(0.3*tt);
    
    %% Orchestrator + logger
    orchestrator = SensorOrchestrator("Fs", Fs);
    logger = Logger();
    orchestrator.logger = logger;
    
    %% Evidence storage
    sensorStates = strings(length(t), 7);
    systemStates = strings(length(t), 1);
    classifiedStates = zeros(length(t), 1);   % for BV-4 scenarios only
    
    %% Redundancy Mismatch storage
    altC_hist = nan(length(t),1);
    vsC_hist  = nan(length(t),1);
    sysStateNum = nan(length(t),1);
    
    for k = 1:length(t)
        tt = t(k);
    
        altT = trueAlt(tt);
        vsT  = trueVS(tt);
        asT  = trueAS(tt);
        pitT = truePitch(tt);
        rolT = trueRoll(tt);
        tmpT = trueTemp(tt);
        oilT = trueOil(tt);
    
        [altMeas, altMeta] = altSensor.step(altT);
        [vsMeas,  vsMeta ] = vsSensor.step(vsT);
        [asMeas,  asMeta ] = asSensor.step(asT);
        [pitMeas, pitMeta] = pitchSensor.step(pitT);
        [rolMeas, rolMeta] = rollSensor.step(rolT);
        [tmpMeas, tmpMeta] = tempSensor.step(tmpT);
        [oilMeas, oilMeta] = oilSensor.step(oilT);
    
        [altC, altS] = altMon.step(altMeas);
        [vsC,  vsS ] = vsMon.step(vsMeas);
        [asC,  asS ] = asMon.step(asMeas);
        [pitC, pitS] = pitchMon.step(pitMeas);
        [rolC, rolS] = rollMon.step(rolMeas);
        [tmpC, tmpS] = tempMon.step(tmpMeas);
        [oilC, oilS] = oilMon.step(oilMeas);
    
        altC_hist(k) = altC;
        vsC_hist(k)  = vsC;
    
        states = struct( ...
            "AltitudeSensor", altS, ...
            "AirspeedSensor", asS, ...
            "VerticalSpeedSensor", vsS, ...
            "PitchSensor", pitS, ...
            "RollSensor", rolS, ...
            "TemperatureSensor", tmpS, ...
            "PressureSensor", oilS);
    
        values = struct( ...
            "Altitude", altC, ...
            "VerticalSpeed", vsC);
    
        [state, logMsg, diagnostics] = orchestrator.step(states, values, tt);
    
        switch state
            case "NORMAL"
                sysStateNum(k) = 0;
            case "WARN"
                sysStateNum(k) = 1;
            case "FAULT"
                sysStateNum(k) = 2;
        end
    
        sensorStates(k,:) = [altS, asS, vsS, pitS, rolS, tmpS, oilS];
        systemStates(k) = state;
    
        fprintf("t=%.2f  SYS=%s  ALT=%s  VS=%s  AS=%s  PIT=%s  ROL=%s  TMP=%s  OIL=%s\n", ...
            tt, state, altS, vsS, asS, pitS, rolS, tmpS, oilS);

        if state == "WARN"
            warnPresent(k) = 1; 
        end
    end

    for i = 2:length(sysStateNum)

        % Detect entry into WARN
        if sysStateNum(i) == 1 && sysStateNum(i-1) ~= 1
            warnEvents = warnEvents + 1;
            detectionTime = k; 
        end
    
        % Detect entry into FAULT
        if sysStateNum(i) == 2 && sysStateNum(i-1) ~= 2
            faultEvents = faultEvents + 1;
            detectionTime = k; 
        end
    end

    %% Store overall state for plotting after simulation 
    if any(sysStateNum == 1)
        warnOccured(run) = 1; 
    end

    if any(sysStateNum == 2)
        faultOccured(run) = 1;  
    end 

    warnTimesteps = [warnTimesteps; warnPresent]; 

    %% Calculate the rate of change of the altitude
    altSlope_hist = nan(length(t),1);
    
    for k = 2:length(t)
        if ~isnan(altC_hist(k)) && ~isnan(altC_hist(k-1))
            altSlope_hist(k) = (altC_hist(k) - altC_hist(k-1)) / dt;
        end
    end
end

totalTimesteps = N * length(t);

warnEventRate = warnEvents / totalTimesteps;
faultEventRate = faultEvents / totalTimesteps;

warnRateRuns = (sum(warnOccured == 1))/N;
warnRateTimestepsRows = sum(warnTimesteps == 1); 
warnRateTimesteps = (sum(warnRateTimestepsRows))/(numel(warnTimesteps)); 
faultRate = (sum(faultOccured == 1))/N; 

fprintf("Monte Carlo Results for %d runs in scenario %s:\n", N, scenarioId)

fprintf("False WARN events: %d\n", warnEvents)
fprintf("WARN event rate: %.5f\n", warnEventRate)

fprintf("FAULT events: %d\n", faultEvents)
fprintf("FAULT event rate (runs): %.3f\n", faultEventRate)

if scenarioId == "MCV-1"
    % Make Graph(s) showing noise distribution and need to show % which caused warnings/fails etc.  
    
    % Hiustogram of warning outcomes

    % Bar chart of warning/fault rates
    figure
    bar([warnEventRate*100 faultEventRate*100])
    ylabel("Percentage (%)")
    xticklabels(["False WARN","False FAULT"])
    title("Monte Carlo Robustness Results under 1000 runs, treating WARN and FAULT transitions as event-driven")
    annotation("textbox", ...
    [0.65 0.6 0.25 0.25], ...
    "String", { ...
    "Noise model:", ...
    "Altitude σ = 0.3 m", ...
    "Vertical Speed σ = 0.1 m/s", ...
    "Airspeed σ = 0.1 m/s", ...
    "Pitch σ = 0.3°", ...
    "Roll σ = 0.3°", ...
    "Temp σ = 0.5 °C", ...
    "Oil σ = 0.2 bar"}, ...
    "FitBoxToText","on");
    grid on

end 

disp("Scenario complete.");

%% Local helper function
function classId = FaultClassifier(meta)
% Converts injector metadata into a numeric fault-class label.
%
% Output mapping:
%   0 = Normal
%   1 = Stuck-at
%   2 = Bias
%   3 = Drift
%   4 = Spike
%   5 = Dropout

    classId = 0;  % default = Normal

    if ~isstruct(meta)
        return;
    end

    if ~isfield(meta, "faultActive") || ~isfield(meta, "faultType")
        return;
    end

    if ~meta.faultActive
        classId = 0;
        return;
    end

    switch upper(string(meta.faultType))
        case "STUCK"
            classId = 1;
        case "BIAS"
            classId = 2;
        case "DRIFT"
            classId = 3;
        case "SPIKE"
            classId = 4;
        case "DROPOUT"
            classId = 5;
        otherwise
            classId = 0;
    end
end
