%% drone_microdoppler_digital_array_main.m
% Micro-Doppler Simulation of a Quadcopter Drone flying A -> B -> C,
% now with a DIGITAL ARRAY RADAR receive chain.
%
% Everything upstream of the receiver is unchanged from the previous
% waypoint version (waveform, platform, FreeSpace channel, Radiator,
% RadarTarget). The receive side is now digital: instead of collapsing
% the array to a single channel with sum(echo,2) (an analog-style,
% unweighted combine), every element's signal is kept individually and
% combined explicitly with a phase-shift (delay-and-sum) digital
% beamformer, steered pulse-by-pulse toward the target's angle. This is
% the key capability a digital array adds over an analog/summed array:
% the combining weights can be changed per pulse (or even applied to
% multiple simultaneous beams) because every element is digitized
% separately before combining.
%
% Also fixed in this version: the standalone phased.MatchedFilter used
% to build the range-tracked signal did not cancel its own internal
% group delay, which showed up as detected range being offset high by
% exactly one chirp-length's worth of range bins (~1500 m here). Fixed
% by adding 'GroupDelaySource','Auto' to the matched filter object,
% which automatically re-aligns the filter output with the true
% round-trip delay. No other logic needed to change for this fix.
%
% Requires:
%   - Phased Array System Toolbox
%   - droneMotion.m in the same folder

clear;
clc;
close all;

%% Constants

c = physconst('LightSpeed');
fc = 5e9;
lambda = c/fc;

%% Radar Parameters

fs = 10e6;
prf = 2e4;
dt = 1/prf;

sweepBW = 5e6;                 % chirp bandwidth -> range resolution
rangeRes = c/(2*sweepBW);
fprintf('Range Resolution           = %.2f m\n',rangeRes);

radarpos = [0;0;0];
radarvel = [0;0;0];

% --- Modeling assumptions (unchanged from previous version) ---
% Radar is STATIONARY and NON-SCANNING: radarpos/radarvel are fixed at
% zero for the whole run, and the array below has no mechanical or
% electronic boresight steering / rotation applied -- it is a fixed
% staring array, not a rotating (PPI-style) radar.
%
% NO atmosphere or wind is modeled: phased.FreeSpace below applies only
% free-space path loss and propagation delay -- no atmospheric
% attenuation, rain/humidity loss, turbulence, or wind. The drone's
% flight path is pure kinematics (piecewise-constant velocity between
% waypoints); there is no gust/turbulence perturbation on its motion.

%% Waypoints (A -> B -> C), each leg >= 100 m, flown in a few seconds

A = [500; 0;   100];
B = [620; 80;  100];
C = [500; 180; 100];

legDurations = [2 2];          % seconds spent flying each leg

waypoints = [A B C];

legDist  = vecnorm(diff(waypoints,1,2));
legSpeed = legDist./legDurations;

fprintf('\nLeg 1 (A->B): distance = %.1f m, duration = %.1f s, speed = %.1f m/s (%.0f km/h)\n', ...
    legDist(1),legDurations(1),legSpeed(1),legSpeed(1)*3.6);
fprintf('Leg 2 (B->C): distance = %.1f m, duration = %.1f s, speed = %.1f m/s (%.0f km/h)\n\n', ...
    legDist(2),legDurations(2),legSpeed(2),legSpeed(2)*3.6);

vel1 = (B-A)/legDurations(1);
vel2 = (C-B)/legDurations(2);

cumT = [0 cumsum(legDurations)];       % [0, tB, tC]

%% Drone Motion (initialized on leg 1; velocity updated at waypoint B)

tgtmotion = phased.Platform( ...
    'InitialPosition',A,...
    'Velocity',vel1);

%% Quadcopter Geometry

rotorRadius = 0.12;

rotorOffset = [...
     0.25   0.25   0
    -0.25   0.25   0
    -0.25  -0.25   0
     0.25  -0.25   0]';

Nrotors = 4;
Nblades = 2;

bladeRate = 150*2*pi;
bladePhase = (0:Nrotors-1)*pi/2;

%% Radar Target

MeanRCS = [0.2 0.01*ones(1,Nrotors*Nblades)];

droneTarget = phased.RadarTarget( ...
    'MeanRCS',MeanRCS,...
    'PropagationSpeed',c,...
    'OperatingFrequency',fc);

%% Waveform (LFM chirp for real range resolution)

wav = phased.LinearFMWaveform(...
    'SampleRate',fs,...
    'PulseWidth',10e-6,...
    'PRF',prf,...
    'SweepBandwidth',sweepBW,...
    'SweepDirection','Up',...
    'SweepInterval','Positive');

%% Antenna (Digital Array)

ura = phased.URA( ...
    'Size',[4 4],...
    'ElementSpacing',[lambda/2 lambda/2]);

Nelements = prod(ura.Size);

tx = phased.Transmitter(...
    'PeakPower',1000,...
    'Gain',20);

rx = phased.ReceiverPreamp(...
    'Gain',20,...
    'NoiseFigure',3);

%% Propagation Channel

channel = phased.FreeSpace( ...
    'PropagationSpeed',c,...
    'OperatingFrequency',fc,...
    'SampleRate',fs,...
    'TwoWayPropagation',true);

%% Radiator / Collector
% NOTE: phased.Collector below already returns one column per array
% element (it combines the multiple *incident scatterer* signals onto
% each element, it does NOT sum across elements). In the previous
% version this per-element data was thrown away with sum(echo,2)
% immediately after. This version keeps it and beamforms explicitly --
% see "Digital Beamformer" section below.

txant = phased.Radiator(...
    'Sensor',ura,...
    'OperatingFrequency',fc,...
    'PropagationSpeed',c);

rxant = phased.Collector(...
    'Sensor',ura,...
    'OperatingFrequency',fc,...
    'PropagationSpeed',c);

%% Digital Beamformer (explicit digital array processing)
% Phase-shift (delay-and-sum) digital beamforming across the Nelements
% receive channels. DirectionSource is set to 'Input port' so the
% steering direction can be updated every pulse -- this per-pulse
% re-steering is the thing an analog/summed array cannot do, and is
% the main practical benefit of going digital.
%
% Simplification/assumption: this simulation steers using the
% scatterer's TRUE look angle (from droneMotion/rangeangle) each pulse,
% i.e. perfect a-priori angle knowledge. A real digital array radar
% would instead estimate that angle (e.g. monopulse or a DOA/MUSIC-type
% estimator run on the same element data) and steer using the estimate.
% That angle-estimation step is not implemented here.

beamformer = phased.PhaseShiftBeamformer( ...
    'SensorArray',ura,...
    'OperatingFrequency',fc,...
    'PropagationSpeed',c,...
    'DirectionSource','Input port');

arrayGain_dB = 10*log10(Nelements);
fprintf('Digital array elements      = %d\n',Nelements);
fprintf('Coherent array gain (approx) = %.1f dB\n\n',arrayGain_dB);

%% Simulation Parameters

NSampPerPulse = round(fs/prf);

totalDuration = cumT(end);
Niter = round(totalDuration*prf);

fprintf('Total simulation time = %.1f s (%d pulses)\n',totalDuration,Niter);
fprintf('Approx memory for rxsig  = %.2f GB\n', ...
    NSampPerPulse*Niter*16/1e9);
fprintf('NOTE: this is a long run. Shorten legDurations above for a quick test.\n\n');

rxsig = complex(zeros(NSampPerPulse,Niter));

bodyPosHistory   = zeros(3,Niter);
bodyRangeHistory = zeros(1,Niter);
bodySpeedHistory = zeros(1,Niter);

currentLeg = 1;

disp('Simulation Started...')

%% Pulse Loop

for m = 1:Niter

    t = (m-1)/prf;

    % Switch to leg-2 velocity once the drone reaches waypoint B
    if currentLeg == 1 && t >= cumT(2)
        tgtmotion.Velocity = vel2;
        currentLeg = 2;
    end

    [scatterPos,...
     scatterVel,...
     scatterAng] = droneMotion(...
        t,...
        dt,...
        tgtmotion,...
        radarpos,...
        rotorOffset,...
        rotorRadius,...
        bladeRate,...
        bladePhase);

    bodyPosHistory(:,m)  = scatterPos(:,1);
    bodyRangeHistory(m)  = norm(scatterPos(:,1)-radarpos);
    bodySpeedHistory(m)  = radialspeed( ...
        scatterPos(:,1),scatterVel(:,1),radarpos,radarvel);

    pulse = wav();
    txsig = tx(pulse);
    txsig = txant(txsig,scatterAng);

    echo = channel(...
        txsig,radarpos,scatterPos,radarvel,scatterVel);

    echo = droneTarget(echo);
    echo = rxant(echo,scatterAng);     % NSampPerPulse x Nelements
    echo = rx(echo);                   % add receiver noise, still per-element

    % --- Digital beamforming (replaces sum(echo,2)) ---
    % Steer toward the body scatterer's true angle this pulse.
    rxsig(:,m) = beamformer(echo,scatterAng(:,1));

    if mod(m,20000) == 0
        fprintf('  %d / %d pulses processed\n',m,Niter);
    end

end

disp('Simulation Complete.')

%% Matched Filter
% FIX: phased.MatchedFilter does not cancel its own internal group
% delay, so the raw output's peak sample index sits
% length(mfcoeff)-1 samples LATER than the true round-trip delay
% (previously this showed up as detected range reading ~1500 m too
% high -- exactly chirp_length_in_samples * c/(2*fs), i.e. 100 samples
% * 15 m/sample). Newer toolbox versions expose a
% 'GroupDelaySource','Auto' property to compensate this automatically,
% but that property isn't available in this MATLAB install, so instead
% we shift the filter output up (earlier in time) by its known group
% delay manually -- the version-agnostic fix from the project summary.
% The Range-Doppler snapshot plots further below were already correct
% because they use phased.RangeDopplerResponse, which handles this
% delay internally -- only this standalone-matched-filter path needed
% the fix.

mfcoeff = getMatchedFilter(wav);

mf = phased.MatchedFilter(...
    'Coefficients',mfcoeff);

mfSignalRaw = mf(rxsig);

groupDelay = length(mfcoeff) - 1;   % samples

mfSignal = zeros(size(mfSignalRaw));
mfSignal(1:end-groupDelay,:) = mfSignalRaw(groupDelay+1:end,:);
% Last groupDelay rows are left as zero (no data available to fill them
% in from -- they correspond to ranges beyond the unambiguous window
% edge after the shift).

%% Track-Before-Micro-Doppler: find the target's range bin every pulse
% Because the drone changes range continuously along A->B->C, a single
% fixed range bin (valid for the straight-line case) no longer applies.
% Instead, find the peak-energy range bin pulse-by-pulse, then build the
% slow-time signal by following that bin -- this is the standard
% range-tracking step needed before micro-Doppler analysis of a
% maneuvering target.

[~,rangeBinTrack] = max(abs(mfSignal),[],1);

rangeGrid = (0:NSampPerPulse-1)'*c/(2*fs);
detectedRangeTrack = rangeGrid(rangeBinTrack);

trackedSignal = zeros(1,Niter);
for m = 1:Niter
    trackedSignal(m) = mfSignal(rangeBinTrack(m),m);
end

timeAxis = (0:Niter-1)/prf;

%% Range Track Plot: verifies the A -> B -> C path in range,
% and verifies the group-delay fix (ground truth and detected range
% should now overlay, not sit ~1500 m apart).

figure;
hold on
plot(timeAxis,bodyRangeHistory,'w-','LineWidth',1.5)
plot(timeAxis,detectedRangeTrack,'c.','MarkerSize',2)
xline(cumT(2),'r--','Waypoint B','LabelVerticalAlignment','bottom')
xlabel('Time (s)')
ylabel('Range (m)')
legend('Ground-truth body range','Matched-filter detected range', ...
    'Location','best')
title('Drone Range Track (Digital Array): A \rightarrow B \rightarrow C')
grid on

%% Radial Speed Track Plot

figure;
plot(timeAxis,bodySpeedHistory,'LineWidth',1.5)
xline(cumT(2),'r--','Waypoint B','LabelVerticalAlignment','bottom')
xlabel('Time (s)')
ylabel('Radial Speed (m/s)')
title('Drone Radial Speed (Digital Array): A \rightarrow B \rightarrow C')
grid on

%% Range-Compensated Micro-Doppler Spectrogram (whole flight)

figure;
pspectrum(trackedSignal,prf,'spectrogram');
title('Drone Micro-Doppler Spectrogram (Digital Array): A \rightarrow B \rightarrow C')

%% Range-Doppler Response

rdresp = phased.RangeDopplerResponse(...
    'PropagationSpeed',c,...
    'SampleRate',fs,...
    'OperatingFrequency',fc,...
    'DopplerFFTLengthSource','Property',...
    'DopplerFFTLength',128,...
    'DopplerOutput','Speed');

%% Snapshot Range-Doppler Maps: mid-leg-1 and mid-leg-2

snapTimes = [legDurations(1)/2, cumT(2)+legDurations(2)/2];
snapLabel = {'Mid-Leg A\rightarrowB','Mid-Leg B\rightarrowC'};

for k = 1:2

    idx0 = round(snapTimes(k)*prf);
    idx0 = max(1,min(idx0,Niter-127));
    idxRange = idx0:idx0+127;

    snapRange = mean(bodyRangeHistory(idxRange));
    snapSpeed = mean(bodySpeedHistory(idxRange));

    figure;
    plotResponse(rdresp,rxsig(:,idxRange),mfcoeff);
    ylim([snapRange-50 snapRange+50])
    hold on
    plot(snapSpeed,snapRange,'ro','MarkerSize',12,'LineWidth',1.5)
    legend('','Expected target location','TextColor','white')
    title(['Drone Range-Doppler Map (Digital Array): ' snapLabel{k}])

end

%% Blade Tip Speed / Micro-Doppler Bandwidth (unchanged physics)

tipSpeed = rotorRadius*bladeRate;
bladeFlashFreq = Nblades*bladeRate/(2*pi);

fprintf('\nBlade Tip Speed                = %.2f m/s\n',tipSpeed);
fprintf('Blade-Flash (Sideband) Spacing = %.1f Hz\n',bladeFlashFreq);

%% 3D Flight Path Plot

figure;
hold on
grid on
axis equal

plot3(0,0,0,'ks','MarkerFaceColor','k','MarkerSize',10)

plot3(bodyPosHistory(1,:),bodyPosHistory(2,:),bodyPosHistory(3,:), ...
    '-','Color',[0.3 0.6 1],'LineWidth',1.5)

wpLabels = {'A','B','C'};
for k = 1:3
    plot3(waypoints(1,k),waypoints(2,k),waypoints(3,k), ...
        'ro','MarkerFaceColor','r','MarkerSize',8)
    text(waypoints(1,k),waypoints(2,k),waypoints(3,k)+15,wpLabels{k}, ...
        'Color','white','FontWeight','bold')
end

xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
legend('Radar','Flight path','Waypoints','Location','best')
title('Drone Flight Path (Digital Array): A \rightarrow B \rightarrow C')
view(3)