%{
*
 NAME           : QuinticPolynomial.m
 AUTHOR         : Paweekorn Buasakorn
 DATE           : May 16th 2023
 MODIFIED BY    : Paweekorn Buasakorn
 DESCRIPTION    : QuinticPolynomial(0,100,400,500) in command line
*
%}

function [] = QuinticPolynomial(initialPosition, finalPosition, maxVelocity, maxAcceleration)
    % Parameter settings
    maxVel = maxVelocity; % Maximum velocity (e.g., 500)
    maxAcc = maxAcceleration; % Maximum acceleration (e.g., 400)
    displacement = finalPosition - initialPosition;
    distance = abs(displacement);
    timeToReachFinalPosition = ceil(max(sqrt((40 * sqrt(3) * max(distance) / (3 * maxAcc)) / 2), (15 * max(distance)) / (8 * maxVel)));
    totalTrajectoryTime = timeToReachFinalPosition;

    % Solve coefficients for the R trajectory
    coefficients = [initialPosition(1);
               0;
               0;
               (10 * displacement(1)) / totalTrajectoryTime^3;
               -(15 * displacement(1)) / totalTrajectoryTime^4;
               (6 * displacement(1)) / totalTrajectoryTime^5];

    % Trajectory Evaluation
    time = (0:0.0125:totalTrajectoryTime)';
    
    % R trajectory
    position = coefficients(1) + coefficients(2) * time + coefficients(3) * time.^2 + coefficients(4) * time.^3 + coefficients(5) * time.^4 + coefficients(6) * time.^5;
    velocity = coefficients(2) + 2 * coefficients(3) * time + 3 * coefficients(4) * time.^2 + 4 * coefficients(5) * time.^3 + 5 * coefficients(6) * time.^4;
    acceleration = 2 * coefficients(3) + 6 * coefficients(4) * time + 12 * coefficients(5) * time.^2 + 20 * coefficients(6) * time.^3;
    jerk = 6 * coefficients(4) + 24 * coefficients(5) * time + 60 * coefficients(6) * time.^2;
    snap = 24 * coefficients(5) + 120 * coefficients(6) * time;
    crakcle = 120 * coefficients(6);
    
    % Plot the trajectories
    subplot(6, 1, 1);
    plot(time, position);
    xlabel('Time');
    ylabel('Position');
    
    subplot(6, 1, 2);
    plot(time, velocity);
    xlabel('Time');
    ylabel('Velocity');
    
    subplot(6, 1, 3);
    plot(time, acceleration);
    xlabel('Time');
    ylabel('Acceleration');
    
    subplot(6, 1, 4);
    plot(time, jerk);
    xlabel('Time');
    ylabel('Jerk');
    
    subplot(6, 1, 5);
    plot(time, snap);
    xlabel('Time');
    ylabel('Snap');
    
    subplot(6, 1, 6);
    plot(time, crakcle);
    xlabel('Time');
    ylabel('Crakcle');
end
