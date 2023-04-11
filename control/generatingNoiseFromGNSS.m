


function motionWithNoisy = generatingNoiseFromGNSS(motion,accuracy)
    INS = insSensor("PositionAccuracy",accuracy.AccuracyPos, ...
                    "VelocityAccuracy",accuracy.AccuracyVel, ...
                    "AccelerationAccuracy",accuracy.AccuracyAcc, ...
                    "AngularVelocityAccuracy",accuracy.AccuracyAng);
    measurement = INS(motion);
    positionMeasurement = measurement.Position;
    velocityMeasurement = measurement.Velocity;
    accelerationMeasurement = measurement.Acceleration;
    angularMeasurement = measurement.AngularVelocity;
    motionWithNoisy = [positionMeasurement; 
                       velocityMeasurement;
                       accelerationMeasurement;
                       angularMeasurement];
end

