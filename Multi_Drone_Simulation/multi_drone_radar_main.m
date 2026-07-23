%% multi_drone_radar_main.m
% Multi-Drone Detection and Tracking using 8x8 Digital Array Radar
% Carrier Frequency: 10 GHz
%
% This simulation demonstrates:
% 1. Multiple drones (4) at random positions
% 2. 8x8 digital array radar with adaptive beamforming
% 3. Beam steering dynamics to track different drones
% 4. Range-Doppler detection for each drone
% 5. Beamforming gain and pattern evolution
%
% Requires:
%   - Phased Array System Toolbox
%   - multiDroneMotion.m in the same folder

clear all;
clc;
close all;

%% ========== CONSTANTS & BASIC PARAMETERS ==========

c = physconst('LightSpeed');
fc = 10e9;                              % 10 GHz carrier
lambda = c/fc;

fprintf('\n========== MULTI-DRONE RADAR SYSTEM ==========\n');
fprintf('Carrier Frequency    = %.1f GHz\n', fc/1e9);
fprintf('Wavelength           = %.4f cm\n', lambda*100);

%% ========== RADAR PARAMETERS ==========

fs = 25e6;                              % Sample rate
prf = 5e3;                              % Pulse Repetition Frequency
dt = 1/prf;

sweepBW = 10e6;                         % Chirp bandwidth
rangeRes = c/(2*sweepBW);
fprintf('Range Resolution     = %.2f m\n', rangeRes);
fprintf('PRF                  = %.0f Hz\n\n', prf);

radarpos = [0; 0; 50];                  % Radar at elevated position
radarvel = [0; 0; 0];

%% ========== RANDOM DRONE INITIALIZATION ==========

Ndrones = 4;
fprintf('Number of Drones     = %d\n', Ndrones);
fprintf('--- Drone Initial Positions (Random) ---\n');

% Random positions within a specified area
% X: 300-800 m, Y: -200 to +200 m, Z: 80-150 m (altitude)
rng(42);  % Reproducible randomness for testing

droneInitPos = zeros(3, Ndrones);
droneVel = zeros(3, Ndrones);

for d = 1:Ndrones
    droneInitPos(1, d) = 300 + rand()*500;      % X
    droneInitPos(2, d) = -200 + rand()*400;     % Y
    droneInitPos(3, d) = 80 + rand()*70;        % Z (altitude)
    
    % Random velocity (m/s) for each drone
    droneVel(1, d) = -10 + rand()*20;           % Vx
    droneVel(2, d) = -10 + rand()*20;           % Vy
    droneVel(3, d) = -2 + rand()*4;             % Vz
    
    fprintf('Drone %d: Pos = [%.1f, %.1f, %.1f] m,  Vel = [%.2f, %.2f, %.2f] m/s\n', ...
        d, droneInitPos(1,d), droneInitPos(2,d), droneInitPos(3,d), ...
        droneVel(1,d), droneVel(2,d), droneVel(3,d));
end

fprintf('\n');

%% ========== DRONE MOTION (phased.Platform objects) ==========

tgtmotion = cell(Ndrones, 1);
for d = 1:Ndrones
    tgtmotion{d} = phased.Platform( ...
        'InitialPosition', droneInitPos(:,d), ...
        'Velocity', droneVel(:,d));
end

%% ========== QUADCOPTER GEOMETRY (same for all drones) ==========

rotorRadius = 0.12;

rotorOffset = [...
     0.25   0.25   0
    -0.25   0.25   0
    -0.25  -0.25   0
     0.25  -0.25   0]';

Nrotors = 4;
Nblades = 2;

bladeRate = 150*2*pi;                   % rad/s
bladePhase = (0:Nrotors-1)*pi/2;

%% ========== RADAR TARGET ==========

MeanRCS = [0.2 0.01*ones(1,Nrotors*Nblades)];

droneTarget = cell(Ndrones, 1);
for d = 1:Ndrones
    droneTarget{d} = phased.RadarTarget( ...
        'MeanRCS', MeanRCS, ...
        'PropagationSpeed', c, ...
        'OperatingFrequency', fc);
end

%% ========== WAVEFORM (LFM Chirp) ==========

wav = phased.LinearFMWaveform( ...
    'SampleRate', fs, ...
    'PulseWidth', 20e-6, ...
    'PRF', prf, ...
    'SweepBandwidth', sweepBW, ...
    'SweepDirection', 'Up', ...
    'SweepInterval', 'Positive');

%% ========== 8x8 DIGITAL ARRAY ANTENNA ==========

ura = phased.URA( ...
    'Size', [8 8], ...
    'ElementSpacing', [lambda/2 lambda/2]);

Nelements = prod(ura.Size);

tx = phased.Transmitter( ...
    'PeakPower', 1500, ...
    'Gain', 25);

rx = phased.ReceiverPreamp( ...
    'Gain', 25, ...
    'NoiseFigure', 2);

fprintf('Array Configuration  = %d x %d (%d elements)\n', ...
    ura.Size(1), ura.Size(2), Nelements);
fprintf('Element Spacing      = %.4f m (λ/2)\n\n', lambda/2);

arrayGain_dB = 10*log10(Nelements);
fprintf('Theoretical Array Gain (Coherent) = %.1f dB\n\n', arrayGain_dB);

%% ========== PROPAGATION CHANNEL ==========

channel = phased.FreeSpace( ...
    'PropagationSpeed', c, ...
    'OperatingFrequency', fc, ...
    'SampleRate', fs, ...
    'TwoWayPropagation', true);

%% ========== RADIATOR / COLLECTOR ==========

txant = phased.Radiator( ...
    'Sensor', ura, ...
    'OperatingFrequency', fc, ...
    'PropagationSpeed', c);

rxant = phased.Collector( ...
    'Sensor', ura, ...
    'OperatingFrequency', fc, ...
    'PropagationSpeed', c);

%% ========== DIGITAL BEAMFORMER ==========
% Per-pulse adaptive steering toward drone angles

beamformer = phased.PhaseShiftBeamformer( ...
    'SensorArray', ura, ...
    'OperatingFrequency', fc, ...
    'PropagationSpeed', c, ...
    'DirectionSource', 'Input port');

%% ========== SIMULATION PARAMETERS ==========

totalDuration = 10;                     % seconds
Niter = round(totalDuration * prf);

NSampPerPulse = round(fs / prf);

fprintf('Simulation Duration  = %.1f s\n', totalDuration);
fprintf('Total Pulses         = %d\n', Niter);
fprintf('Samples per Pulse    = %d\n', NSampPerPulse);
fprintf('Memory Estimate      = %.2f GB\n\n', ...
    NSampPerPulse * Niter * 8 / 1e9);

%% ========== PRE-ALLOCATION ==========

% Received signal per pulse (beamformed, single channel)
rxsig = complex(zeros(NSampPerPulse, Niter));

% Beamforming steering angles per pulse per drone
steeringAzElHistory = zeros(2, Ndrones, Niter);

% Position and range history for all drones
dronePos = zeros(3, Ndrones, Niter);
droneRangeHistory = zeros(Ndrones, Niter);
droneSpeedHistory = zeros(Ndrones, Niter);
droneSNR_dB = zeros(Ndrones, Niter);

disp('Starting Multi-Drone Radar Simulation...')

%% ========== PULSE LOOP ==========

for m = 1:Niter
    
    t = (m-1) / prf;
    
    % ===== Step 1: Get all drone positions and velocities =====
    allScatterPos = [];
    allScatterVel = [];
    allScatterAng = [];
    
    droneBodyPos = zeros(3, Ndrones);
    droneBodyVel = zeros(3, Ndrones);
    droneBodyAng = zeros(2, Ndrones);
    
    for d = 1:Ndrones
        [scatterPos, scatterVel, scatterAng] = multiDroneMotion( ...
            t, dt, tgtmotion{d}, radarpos, ...
            rotorOffset, rotorRadius, bladeRate, bladePhase);
        
        % Store body position/velocity/angle (scatterer #1)
        droneBodyPos(:, d) = scatterPos(:, 1);
        droneBodyVel(:, d) = scatterVel(:, 1);
        droneBodyAng(:, d) = scatterAng(:, 1);
        
        % Accumulate all scatterers from this drone
        allScatterPos = [allScatterPos, scatterPos];
        allScatterVel = [allScatterVel, scatterVel];
        allScatterAng = [allScatterAng, scatterAng];
        
        % Store history
        dronePos(:, d, m) = droneBodyPos(:, d);
        droneRangeHistory(d, m) = norm(droneBodyPos(:, d) - radarpos);
        droneSpeedHistory(d, m) = radialspeed( ...
            droneBodyPos(:, d), droneBodyVel(:, d), ...
            radarpos, radarvel);
    end
    
    % ===== Step 2: Transmit pulse =====
    pulse = wav();
    txsig = tx(pulse);
    txsig = txant(txsig, allScatterAng);
    
    % ===== Step 3: Propagate through FreeSpace channel =====
    echo = channel( ...
        txsig, radarpos, allScatterPos, radarvel, allScatterVel);
    
    % ===== Step 4: Radar target reflection (all drones) =====
    echo_d = echo;
    idx = 1;
    for d = 1:Ndrones
        Nscatter_d = size(allScatterPos, 2) / Ndrones;  % 9 scatterers per drone
        scatterIdx = (d-1)*Nscatter_d + (1:Nscatter_d);
        
        echo_d(:, scatterIdx) = droneTarget{d}(echo(:, scatterIdx));
    end
    echo = echo_d;
    
    % ===== Step 5: Receive on array elements =====
    echo = rxant(echo, allScatterAng);
    echo = rx(echo);
    
    % ===== Step 6: Beamforming - steer toward dominant drone =====
    % Strategy: average the angles of all drones, or steer to nearest drone
    % For this demo, we use a weighted steering based on range
    
    % Find closest drone
    [~, closestDrone] = min(droneRangeHistory(:, m));
    steerAng = droneBodyAng(:, closestDrone);
    
    % Store steering angle
    steeringAzElHistory(:, closestDrone, m) = steerAng;
    
    % Apply beamformer
    rxsig(:, m) = beamformer(echo, steerAng);
    
    % ===== Step 7: Estimate SNR (simplified) =====
    for d = 1:Ndrones
        % Signal power at body range bin
        signalPower = abs(rxsig(max(1, round(droneRangeHistory(d,m)/(c/(2*fs)))), m))^2;
        droneSNR_dB(d, m) = 10*log10(signalPower + 1e-12);
    end
    
    if mod(m, 1000) == 0
        fprintf('  %d / %d pulses processed\n', m, Niter);
    end
    
end

disp('Simulation Complete.')

%% ========== MATCHED FILTER ==========

mfcoeff = getMatchedFilter(wav);
mf = phased.MatchedFilter('Coefficients', mfcoeff);

mfSignalRaw = mf(rxsig);

groupDelay = length(mfcoeff) - 1;
mfSignal = zeros(size(mfSignalRaw));
mfSignal(1:end-groupDelay, :) = mfSignalRaw(groupDelay+1:end, :);

%% ========== RANGE TRACKING ==========

[~, rangeBinTrack] = max(abs(mfSignal), [], 1);
rangeGrid = (0:NSampPerPulse-1)' * c/(2*fs);
detectedRangeTrack = rangeGrid(rangeBinTrack);

trackedSignal = zeros(1, Niter);
for m = 1:Niter
    trackedSignal(m) = mfSignal(rangeBinTrack(m), m);
end

timeAxis = (0:Niter-1) / prf;

%% ========== PLOTTING ==========

fprintf('\n========== GENERATING PLOTS ==========\n\n');

% ===== Plot 1: Multi-Drone Range Tracks =====
figure('Name', 'Range Tracking', 'NumberTitle', 'off');
hold on
colors = [1 0 0; 0 1 0; 0 0 1; 1 1 0];
for d = 1:Ndrones
    plot(timeAxis, droneRangeHistory(d, :), '-', 'Color', colors(d,:), ...
        'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d', d));
end
plot(timeAxis, detectedRangeTrack, 'c.', 'MarkerSize', 1, ...
    'DisplayName', 'Detected (MF)');
xlabel('Time (s)')
ylabel('Range (m)')
title('Multi-Drone Range Tracking (8×8 Digital Array @ 10 GHz)')
legend('Location', 'best')
grid on

% ===== Plot 2: Multi-Drone Radial Speed =====
figure('Name', 'Radial Speed', 'NumberTitle', 'off');
hold on
for d = 1:Ndrones
    plot(timeAxis, droneSpeedHistory(d, :), '-', 'Color', colors(d,:), ...
        'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d', d));
end
xlabel('Time (s)')
ylabel('Radial Speed (m/s)')
title('Multi-Drone Radial Speed Tracking')
legend('Location', 'best')
grid on

% ===== Plot 3: Micro-Doppler Spectrogram =====
figure('Name', 'Micro-Doppler', 'NumberTitle', 'off');
pspectrum(trackedSignal, prf, 'spectrogram', ...
    'TimeResolution', 0.1, 'FrequencyLimits', [-100 100]);
title('Composite Micro-Doppler Spectrogram (All Drones)')
colorbar

% ===== Plot 4: SNR Evolution per Drone =====
figure('Name', 'SNR Evolution', 'NumberTitle', 'off');
hold on
for d = 1:Ndrones
    plot(timeAxis, droneSNR_dB(d, :), '-', 'Color', colors(d,:), ...
        'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d', d));
end
xlabel('Time (s)')
ylabel('Signal Power (dB)')
title('Signal Power Evolution for Each Drone')
legend('Location', 'best')
grid on

% ===== Plot 5: 3D Flight Paths =====
figure('Name', '3D Flight Paths', 'NumberTitle', 'off');
hold on
grid on
axis equal

plot3(radarpos(1), radarpos(2), radarpos(3), 'ks', ...
    'MarkerFaceColor', 'k', 'MarkerSize', 12, 'DisplayName', 'Radar');

for d = 1:Ndrones
    squeeze_pos = squeeze(dronePos(:, d, :));
    plot3(squeeze_pos(1, :), squeeze_pos(2, :), squeeze_pos(3, :), ...
        '-', 'Color', colors(d,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Drone %d', d));
    
    % Mark initial and final positions
    plot3(squeeze_pos(1, 1), squeeze_pos(2, 1), squeeze_pos(3, 1), 'o', ...
        'Color', colors(d,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(d,:));
    plot3(squeeze_pos(1, end), squeeze_pos(2, end), squeeze_pos(3, end), 's', ...
        'Color', colors(d,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(d,:));
end

xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
legend('Location', 'best')
title('Multi-Drone 3D Flight Paths')
view(45, 30)

% ===== Plot 6: Beamforming Steering Angles =====
figure('Name', 'Beamforming Angles', 'NumberTitle', 'off');

subplot(2, 1, 1)
hold on
for d = 1:Ndrones
    plot(timeAxis, squeeze(steeringAzElHistory(1, d, :)), '-', ...
        'Color', colors(d,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d', d));
end
xlabel('Time (s)')
ylabel('Azimuth (deg)')
title('Beamformer Steering: Azimuth Angle')
legend('Location', 'best')
grid on

subplot(2, 1, 2)
hold on
for d = 1:Ndrones
    plot(timeAxis, squeeze(steeringAzElHistory(2, d, :)), '-', ...
        'Color', colors(d,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d', d));
end
xlabel('Time (s)')
ylabel('Elevation (deg)')
title('Beamformer Steering: Elevation Angle')
legend('Location', 'best')
grid on

% ===== Plot 7: Radiation Patterns at Select Times =====
figure('Name', 'Beam Patterns', 'NumberTitle', 'off');

patternTimes = [1, round(Niter/3), round(2*Niter/3), Niter];
for idx = 1:4
    subplot(2, 2, idx)
    m = patternTimes(idx);
    steerAng = steeringAzElHistory(:, 1, m);
    
    pattern(ura, fc, 'Azimuth', -90:1:90, 'Elevation', 0, ...
        'Weights', phased.SteeringVector(ura, fc, steerAng) .* ...
        conj(phased.SteeringVector(ura, fc, steerAng)));
    
    title(sprintf('Array Pattern @ t = %.2f s', timeAxis(m)));
end

sgtitle('8×8 URA Radiation Pattern Evolution (Steered Toward Primary Target)');

%% ========== SUMMARY STATISTICS ==========

fprintf('\n========== SIMULATION RESULTS ==========\n\n');

for d = 1:Ndrones
    fprintf('--- Drone %d ---\n', d);
    fprintf('  Initial Range   : %.1f m\n', droneRangeHistory(d, 1));
    fprintf('  Final Range     : %.1f m\n', droneRangeHistory(d, end));
    fprintf('  Range Change    : %.1f m\n', ...
        droneRangeHistory(d, end) - droneRangeHistory(d, 1));
    fprintf('  Mean Speed      : %.2f m/s\n', mean(droneSpeedHistory(d, :)));
    fprintf('  Mean SNR        : %.1f dB\n', mean(droneSNR_dB(d, :)));
    fprintf('\n');
end

fprintf('========== SIMULATION COMPLETE ==========\n\n');
