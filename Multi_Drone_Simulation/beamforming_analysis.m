%% beamforming_analysis.m
% Advanced Beamforming Analysis for Multi-Drone Digital Array Radar
%
% This script performs detailed beamforming analysis:
% 1. Beam pattern visualization at different steering angles
% 2. Beamforming gain and sidelobe analysis
% 3. Spatial resolution and interference rejection capability
% 4. Adaptive beam steering dynamics
% 5. Multi-beam formation for simultaneous drone tracking

clear all;
clc;
close all;

%% Constants
c = physconst('LightSpeed');
fc = 10e9;
lambda = c/fc;

fprintf('\n========== BEAMFORMING ANALYSIS ==========\n\n');

%% 8x8 URA Array Setup
ura = phased.URA( ...
    'Size', [8 8], ...
    'ElementSpacing', [lambda/2 lambda/2]);

Nelements = prod(ura.Size);

fprintf('Array: %dx%d URA (%d elements)\n', ura.Size(1), ura.Size(2), Nelements);
fprintf('Frequency: %.1f GHz\n', fc/1e9);
fprintf('Element Spacing: %.4f m (λ/2)\n\n', lambda/2);

%% ===== BEAMFORMING SCENARIO =====
% Assume 4 drones at different angles; show beam steering toward each

droneAzEl = [
    30,  15;      % Drone 1: Az=30°, El=15°
    -45, 20;      % Drone 2: Az=-45°, El=20°
    0,   25;      % Drone 3: Az=0°, El=25°
    -20, 10       % Drone 4: Az=-20°, El=10°
    ];

Ndrones = size(droneAzEl, 1);

%% ===== FIGURE 1: RADIATION PATTERNS FOR EACH DRONE =====

figure('Name', 'Individual Beam Patterns', 'NumberTitle', 'off', ...
    'Position', [100 100 1200 800]);

for d = 1:Ndrones
    subplot(2, 2, d)
    
    steerAng = [droneAzEl(d, 1); droneAzEl(d, 2)];  % [Az; El] in degrees
    
    % Compute steering vector for this direction
    sv = phased.SteeringVector(ura, fc, steerAng);
    
    % Uniform weighting (for now; can be optimized with Hamming, etc.)
    w = sv / norm(sv);
    
    % Plot radiation pattern
    pattern(ura, fc, ...
        'Azimuth', -90:1:90, ...
        'Elevation', 0, ...
        'Weights', w, ...
        'Type', 'gain');
    
    title(sprintf('Beam Pattern: Drone %d (Az=%.0f°, El=%.0f°)', ...
        d, droneAzEl(d, 1), droneAzEl(d, 2)));
    grid on
end

sgtitle('Individual Beam Patterns Steered Toward Each Drone');

%% ===== FIGURE 2: 3D BEAM PATTERNS =====

figure('Name', '3D Beam Patterns', 'NumberTitle', 'off');

droneToPlot = 1;  % Show detailed 3D pattern for drone 1
steerAng = [droneAzEl(droneToPlot, 1); droneAzEl(droneToPlot, 2)];
sv = phased.SteeringVector(ura, fc, steerAng);
w = sv / norm(sv);

pattern(ura, fc, ...
    'Type', 'gain', ...
    'Weights', w, ...
    'CoordinateSystem', 'rectangular');

title(sprintf('3D Gain Pattern Steered Toward Drone %d', droneToPlot));

%% ===== FIGURE 3: NORMALIZED GAIN vs AZIMUTH (Multiple Beams) =====

figure('Name', 'Multi-Beam Gain Response', 'NumberTitle', 'off');

azimuthScan = -90:1:90;
elevationRef = 15;  % Reference elevation

hold on
for d = 1:Ndrones
    steerAng = [droneAzEl(d, 1); elevationRef];  % Normalize elevation for comparison
    sv = phased.SteeringVector(ura, fc, steerAng);
    w = sv / norm(sv);
    
    % Compute gain across azimuth
    gain_dB = zeros(length(azimuthScan), 1);
    for az_idx = 1:length(azimuthScan)
        scanAng = [azimuthScan(az_idx); elevationRef];
        sv_scan = phased.SteeringVector(ura, fc, scanAng);
        gain = abs(w' * sv_scan)^2;
        gain_dB(az_idx) = 10*log10(gain);
    end
    
    plot(azimuthScan, gain_dB, '-', 'LineWidth', 2, ...
        'DisplayName', sprintf('Steer to Drone %d (Az=%.0f°)', d, droneAzEl(d, 1)));
end

xlabel('Scan Azimuth (degrees)')
ylabel('Normalized Gain (dB)')
title('Multi-Beam Gain Response Across Azimuth')
legend('Location', 'best')
grid on
xlim([-90 90])

%% ===== FIGURE 4: BEAM PATTERNS IN POLAR COORDINATES =====

figure('Name', 'Polar Beam Patterns', 'NumberTitle', 'off', ...
    'Position', [100 100 1000 800]);

for d = 1:Ndrones
    subplot(2, 2, d)
    
    steerAng = [droneAzEl(d, 1); droneAzEl(d, 2)];
    sv = phased.SteeringVector(ura, fc, steerAng);
    w = sv / norm(sv);
    
    % Compute gain in polar pattern
    theta_scan = 0:1:360;
    r_gain = zeros(length(theta_scan), 1);
    
    for idx = 1:length(theta_scan)
        scanAng = [theta_scan(idx); droneAzEl(d, 2)];
        sv_scan = phased.SteeringVector(ura, fc, scanAng);
        gain = abs(w' * sv_scan)^2;
        r_gain(idx) = 10*log10(gain + 1e-10);  % dB, with floor
    end
    
    % Normalize to max
    r_gain = r_gain - max(r_gain);
    
    polarplot(deg2rad(theta_scan), r_gain, 'LineWidth', 2)
    thetaticklabels({'0°', '45°', '90°', '135°', '180°', ...
        '-135°', '-90°', '-45°'})
    title(sprintf('Drone %d (Az=%.0f°)', d, droneAzEl(d, 1)))
    rlim([-40 5])
    grid on
end

sgtitle('Polar Radiation Patterns (Elevation Slice)');

%% ===== FIGURE 5: BEAMFORMING GAIN ANALYSIS =====

figure('Name', 'Beamforming Gain Analysis', 'NumberTitle', 'off');

% Compute gain for each drone in its own direction
gainOnAxis_dB = zeros(Ndrones, 1);
gainOffAxis_dB = zeros(Ndrones, 1);

for d = 1:Ndrones
    % Steering direction
    steerAng = [droneAzEl(d, 1); droneAzEl(d, 2)];
    sv = phased.SteeringVector(ura, fc, steerAng);
    w = sv / norm(sv);
    
    % On-axis gain (perfect match)
    gainOnAxis_dB(d) = 10*log10(abs(w' * sv)^2);
    
    % Off-axis gain (perpendicular direction)
    offAxisAng = [droneAzEl(d, 1) + 90; droneAzEl(d, 2)];
    sv_offaxis = phased.SteeringVector(ura, fc, offAxisAng);
    gainOffAxis_dB(d) = 10*log10(abs(w' * sv_offaxis)^2);
end

subplot(1, 2, 1)
bar(1:Ndrones, gainOnAxis_dB, 'b', 'FaceAlpha', 0.7)
hold on
bar(1:Ndrones + 0.15, gainOffAxis_dB, 'r', 'FaceAlpha', 0.7)
xlabel('Drone Index')
ylabel('Gain (dB)')
title('On-Axis vs Off-Axis Gain')
legend('On-Axis (Steered)', 'Off-Axis (Perpendicular)')
grid on
xticks(1:Ndrones)

% Sidelobe level
subplot(1, 2, 2)
sidelobeSuppression = gainOnAxis_dB - gainOffAxis_dB;
bar(1:Ndrones, sidelobeSuppression, 'g', 'FaceAlpha', 0.7)
xlabel('Drone Index')
ylabel('Suppression (dB)')
title('Sidelobe Suppression (On-Axis - Off-Axis)')
grid on
xticks(1:Ndrones)
ylim([0 50])

%% ===== FIGURE 6: SPATIAL RESOLUTION =====

figure('Name', 'Spatial Resolution', 'NumberTitle', 'off');

% 3dB beamwidth analysis
steerAng = [0; 15];
sv_ref = phased.SteeringVector(ura, fc, steerAng);
w_ref = sv_ref / norm(sv_ref);

azimuthScan = -10:0.1:10;
beamWidth_gain = zeros(length(azimuthScan), 1);

for idx = 1:length(azimuthScan)
    scanAng = [azimuthScan(idx); 15];
    sv_scan = phased.SteeringVector(ura, fc, scanAng);
    gain = abs(w_ref' * sv_scan)^2;
    beamWidth_gain(idx) = 10*log10(gain);
end

% Normalize
beamWidth_gain = beamWidth_gain - max(beamWidth_gain);

subplot(1, 2, 1)
plot(azimuthScan, beamWidth_gain, 'LineWidth', 2)
hold on
yline(-3, 'r--', '3dB points')
xlabel('Azimuth Offset from Boresight (degrees)')
ylabel('Normalized Gain (dB)')
title('Beamwidth Analysis (3dB Beamwidth)')
grid on

% Find 3dB beamwidth
idx_3dB = find(beamWidth_gain >= -3);
beamwidth_3dB = azimuthScan(idx_3dB(end)) - azimuthScan(idx_3dB(1));

% Theoretical beamwidth approximation: λ/D where D is array dimension
arrayDim = 8 * (c/(2*fc));  % Physical size
theoreticalBW = rad2deg(lambda / arrayDim);

subplot(1, 2, 2)
data = [beamwidth_3dB, theoreticalBW];
bar(data, 'FaceAlpha', 0.7)
set(gca, 'XTickLabel', {'Measured', 'Theoretical'})
ylabel('3dB Beamwidth (degrees)')
title('3dB Beamwidth Comparison')
grid on
ylim([0 5])

fprintf('Measured 3dB Beamwidth   : %.2f deg\n', beamwidth_3dB);
fprintf('Theoretical Beamwidth    : %.2f deg\n', theoreticalBW);

%% ===== FIGURE 7: BEAM STEERING DYNAMICS =====

figure('Name', 'Beam Steering Dynamics', 'NumberTitle', 'off');

% Simulate beamformer steering through azimuth sweep
steerAngles = [0:5:60]';
Nsteers = length(steerAngles);
elevationFixed = 15;

gainMatrix = zeros(Nsteers, Nsteers);

for steer_idx = 1:Nsteers
    steerAng = [steerAngles(steer_idx); elevationFixed];
    sv = phased.SteeringVector(ura, fc, steerAng);
    w = sv / norm(sv);
    
    for scan_idx = 1:Nsteers
        scanAng = [steerAngles(scan_idx); elevationFixed];
        sv_scan = phased.SteeringVector(ura, fc, scanAng);
        gain = abs(w' * sv_scan)^2;
        gainMatrix(scan_idx, steer_idx) = 10*log10(gain);
    end
end

% Normalize for display
gainMatrix = gainMatrix - max(gainMatrix(:));

imagesc(steerAngles, steerAngles, gainMatrix)
colorbar
xlabel('Steering Angle (degrees)')
ylabel('Scan Angle (degrees)')
title('Beamformer Response Heatmap (Gain vs Steer vs Scan)')
set(gca, 'YDir', 'normal')
colormap('hot')

%% ===== FIGURE 8: ADAPTIVE BEAMFORMING WEIGHTS VISUALIZATION =====

figure('Name', 'Adaptive Weights', 'NumberTitle', 'off', ...
    'Position', [100 100 1200 600]);

% Show array element weights for different steering directions
droneSteer = 2;  % Show weights for drone 2
steerAng = [droneAzEl(droneSteer, 1); droneAzEl(droneSteer, 2)];
sv = phased.SteeringVector(ura, fc, steerAng);
w_norm = sv / norm(sv);

% Reshape weights for visualization
arraySize = ura.Size;
w_grid = reshape(w_norm, arraySize(2), arraySize(1));  % [Y, X]
w_mag = abs(w_grid);
w_phase = angle(w_grid);

subplot(1, 2, 1)
imagesc(w_mag)
colorbar('Label', 'Magnitude')
title(sprintf('Weight Magnitude (Steer to Drone %d)', droneSteer))
xlabel('Element X Index')
ylabel('Element Y Index')
axis equal

subplot(1, 2, 2)
imagesc(rad2deg(w_phase))
colorbar('Label', 'Phase (degrees)')
title(sprintf('Weight Phase (Steer to Drone %d)', droneSteer))
xlabel('Element X Index')
ylabel('Element Y Index')
axis equal
caxis([-180 180])

%% ===== FIGURE 9: MULTI-BEAM TRACKING CAPABILITY =====

figure('Name', 'Multi-Beam Simultaneous Tracking', 'NumberTitle', 'off');

% Show how beamformer can approximately track multiple drones
% by creating multiple beams

subplot(1, 2, 1)
hold on
colors = [1 0 0; 0 1 0; 0 0 1; 1 1 0];
for d = 1:Ndrones
    steerAng = [droneAzEl(d, 1); droneAzEl(d, 2)];
    sv = phased.SteeringVector(ura, fc, steerAng);
    
    % Compute response across azimuth
    azScan = -90:1:90;
    resp = zeros(length(azScan), 1);
    for az_idx = 1:length(azScan)
        scanAng = [azScan(az_idx); droneAzEl(d, 2)];
        sv_scan = phased.SteeringVector(ura, fc, scanAng);
        gain = abs(sv' * sv_scan)^2;
        resp(az_idx) = 10*log10(gain);
    end
    
    % Normalize and plot
    resp_norm = resp - max(resp);
    plot(azScan, resp_norm, 'Color', colors(d,:), 'LineWidth', 2, ...
        'DisplayName', sprintf('Drone %d Beam', d));
end

xlabel('Azimuth (degrees)')
ylabel('Normalized Gain (dB)')
title('Approximate Multi-Beam Response (Overlaid)')
legend('Location', 'best')
grid on
xlim([-90 90])
ylim([-40 5])

% Interference rejection capability
subplot(1, 2, 2)
hold on

% Main beam steered to drone 1
mainSteer = [droneAzEl(1, 1); droneAzEl(1, 2)];
sv_main = phased.SteeringVector(ura, fc, mainSteer);
w_main = sv_main / norm(sv_main);

azScan = -90:1:90;
mainBeamResp = zeros(length(azScan), 1);
for az_idx = 1:length(azScan)
    scanAng = [azScan(az_idx); droneAzEl(1, 2)];
    sv_scan = phased.SteeringVector(ura, fc, scanAng);
    gain = abs(w_main' * sv_scan)^2;
    mainBeamResp(az_idx) = 10*log10(gain);
end

mainBeamResp = mainBeamResp - max(mainBeamResp);

plot(azScan, mainBeamResp, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Main Beam (Drone 1)');

% Show interference from other drones
for d = 2:Ndrones
    plot(droneAzEl(d, 1), mainBeamResp(round(droneAzEl(d, 1)) + 91), ...
        's', 'Color', colors(d,:), 'MarkerSize', 10, 'LineWidth', 2, ...
        'DisplayName', sprintf('Drone %d Interference', d));
end

xlabel('Azimuth (degrees)')
ylabel('Normalized Gain (dB)')
title('Interference Rejection (Main Beam Steered to Drone 1)')
legend('Location', 'best')
grid on
xlim([-90 90])
ylim([-40 5])

%% ===== SUMMARY STATISTICS =====

fprintf('\n========== BEAMFORMING PERFORMANCE SUMMARY ==========\n\n');

theoreticalGain = 10*log10(Nelements);
fprintf('Theoretical Coherent Gain (all elements): %.1f dB\n', theoreticalGain);

fprintf('\nPer-Drone Analysis:\n');
for d = 1:Ndrones
    fprintf('  Drone %d (Az=%.0f°, El=%.0f°):\n', d, droneAzEl(d, 1), droneAzEl(d, 2));
    fprintf('    On-Axis Gain      : %.1f dB\n', gainOnAxis_dB(d));
    fprintf('    Off-Axis Gain     : %.1f dB\n', gainOffAxis_dB(d));
    fprintf('    Sidelobe Suppression : %.1f dB\n', gainOnAxis_dB(d) - gainOffAxis_dB(d));
end

fprintf('\nArray Characteristics:\n');
fprintf('  3dB Beamwidth        : %.2f degrees\n', beamwidth_3dB);
fprintf('  Array Elements       : %d\n', Nelements);
fprintf('  Element Spacing      : %.4f m\n', lambda/2);
fprintf('  Operating Frequency  : %.1f GHz\n', fc/1e9);

fprintf('\n========== BEAMFORMING ANALYSIS COMPLETE ==========\n\n');
