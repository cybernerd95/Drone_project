function [scatterPos, scatterVel, scatterAng] = multiDroneMotion( ...
    t, dt, tgtmotion, radarpos, ...
    rotorOffset, rotorRadius, bladeRate, bladePhase)
%MULTIDRONEMOTION Generate quadcopter body and blade-tip scatterers for one drone.
%
% This is functionally identical to droneMotion.m but adapted for
% use in the multi-drone scenario where each drone's motion is computed
% independently via a separate phased.Platform object.
%
% Scatterer #1   : Drone body
% Scatterer #2-9 : Blade tips (2 blades x 4 rotors)
%
% Inputs
%   t             - Current simulation time (s), drives blade phase
%   dt            - Time step for tgtmotion advancement (s)
%   tgtmotion     - phased.Platform object (this drone's body motion)
%   radarpos      - 3x1 radar position
%   rotorOffset   - 3x4 rotor center offsets from body frame
%   rotorRadius   - Blade length (m)
%   bladeRate     - Rotor angular velocity (rad/s)
%   bladePhase    - Initial phase of each rotor (1x4)
%
% Outputs
%   scatterPos    - 3x9 positions
%   scatterVel    - 3x9 velocities
%   scatterAng    - 2x9 look angles ([az;el], deg), from radarpos

%% Advance body position and velocity by one pulse interval
[pos, vel] = tgtmotion(dt);

%% Number of rotors
Nrotors = size(rotorOffset, 2);

%% Total scatterers
Nscatter = 1 + 2*Nrotors;

scatterPos = zeros(3, Nscatter);
scatterVel = zeros(3, Nscatter);

%% Body scatterer (index 1)
scatterPos(:, 1) = pos;
scatterVel(:, 1) = vel;

%% Blade-tip scatterers (indices 2-9)
idx = 2;

for r = 1:Nrotors
    
    % Rotor center (rigid offset from body; body assumed non-rotating)
    center = pos + rotorOffset(:, r);
    
    % Rotor phase angle at time t
    theta = bladeRate*t + bladePhase(r);
    
    % Two blades, 180 degrees apart
    for b = 0:1
        
        phi = theta + b*pi;
        
        % Tip position relative to rotor hub, in the rotor plane (x-y)
        tipOffset = rotorRadius * [ ...
            cos(phi)
            sin(phi)
            0];
        
        tipPos = center + tipOffset;
        
        % Tip velocity = body translation + tangential blade-rotation term
        tipVelRot = rotorRadius * bladeRate * [ ...
            -sin(phi)
            cos(phi)
            0];
        
        tipVel = vel + tipVelRot;
        
        scatterPos(:, idx) = tipPos;
        scatterVel(:, idx) = tipVel;
        
        idx = idx + 1;
        
    end
    
end

%% Look angles from radar to every scatterer
[~, scatterAng] = rangeangle(scatterPos, radarpos);

end
