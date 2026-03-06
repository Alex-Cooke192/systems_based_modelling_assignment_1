%% runBehaviouralScenario.m
clear; clc;

scenarioId = "BV-2";   % choose: BV-1, BV-2, BV-3, BV-4a, BV-4b, BV-4c, BV-4d, BV-5

%% Simulation parameters
Fs = 10;
dt = 1/Fs;
Tsim = 5;
t = 0:dt:Tsim;

%% Instantiate sensors
altSensor   = AltitudeSensorSITL("Fs", Fs);
vsSensor    = VerticalSpeedSensorSITL("Fs", Fs);
asSensor    = AirspeedSensorSITL("Fs", Fs);
pitchSensor = PitchSensorSITL("Fs", Fs);
rollSensor  = RollSensorSITL("Fs", Fs);
tempSensor  = EngineTempSensorSITL("Fs", Fs);
oilSensor   = OilPressureSensorSITL("Fs", Fs);

%% Default: no faults, small optional noise
altSensor.Injector   = ScriptedInjector("NoiseSigma", 0.05);
vsSensor.Injector    = ScriptedInjector("NoiseSigma", 0.05);
asSensor.Injector    = ScriptedInjector("NoiseSigma", 0.05);
pitchSensor.Injector = ScriptedInjector("NoiseSigma", 0.05);
rollSensor.Injector  = ScriptedInjector("NoiseSigma", 0.05);
tempSensor.Injector  = ScriptedInjector("NoiseSigma", 0.05);
oilSensor.Injector   = ScriptedInjector("NoiseSigma", 0.05);

%% Apply scenario-specific scripted fault
switch scenarioId
    case "BV-1"
        % Nominal: no injected faults

    case "BV-2"
        % Safety-critical sensor invalid/out-of-range
        % Example: altitude dropout
        altSensor.Injector = ScriptedInjector( ...
            "NoiseSigma", 0.05, ...
            "Enabled", true, ...
            "FaultType", "DROPOUT", ...
            "FaultStartTime", 1.0, ...
            "FaultDuration", 0.8);

    case "BV-3"
        % Non-safety-critical sensor invalid/out-of-range
        % Example: engine temperature dropout
        tempSensor.Injector = ScriptedInjector( ...
            "NoiseSigma", 0.05, ...
            "Enabled", true, ...
            "FaultType", "DROPOUT", ...
            "FaultStartTime", 1.0, ...
            "FaultDuration", 0.8);

    case "BV-4a"
        % Stuck-at fault
        asSensor.Injector = ScriptedInjector( ...
            "NoiseSigma", 0.05, ...
            "Enabled", true, ...
            "FaultType", "STUCK", ...
            "FaultStartTime", 1.0, ...
            "FaultDuration", 1.2);

    case "BV-4b"
        % Bias fault
        pitchSensor.Injector = ScriptedInjector( ...
            "NoiseSigma", 0.05, ...
            "Enabled", true, ...
            "FaultType", "BIAS", ...
            "FaultStartTime", 1.0, ...
            "FaultDuration", 1.2, ...
            "BiasMagnitude", 5.0);

    case "BV-4c"
        % Drift fault
        rollSensor.Injector = ScriptedInjector( ...
            "NoiseSigma", 0.05, ...
            "Enabled", true, ...
            "FaultType", "DRIFT", ...
            "FaultStartTime", 1.0, ...
            "FaultDuration", 1.5, ...
            "DriftRateMagnitude", 3.0);

    case "BV-4d"
        % Dropout fault
        oilSensor.Injector = ScriptedInjector( ...
            "NoiseSigma", 0.05, ...
            "Enabled", true, ...
            "FaultType", "DROPOUT", ...
            "FaultStartTime", 1.0, ...
            "FaultDuration", 0.8);

    case "BV-5"
        % Redundancy mismatch: bias VS so it disagrees with d(alt)/dt
        vsSensor.Injector = ScriptedInjector( ...
            "NoiseSigma", 0.05, ...
            "Enabled", true, ...
            "FaultType", "BIAS", ...
            "FaultStartTime", 1.0, ...
            "FaultDuration", 1.5, ...
            "BiasMagnitude", 3.0);

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

    sensorStates(k,:) = [altS, asS, vsS, pitS, rolS, tmpS, oilS];
    systemStates(k) = state;

    fprintf("t=%.2f  SYS=%s  ALT=%s  VS=%s  AS=%s  PIT=%s  ROL=%s  TMP=%s  OIL=%s\n", ...
        tt, state, altS, vsS, asS, pitS, rolS, tmpS, oilS);
end

disp("Scenario complete.");