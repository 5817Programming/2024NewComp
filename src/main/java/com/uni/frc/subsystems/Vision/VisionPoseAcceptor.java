package com.uni.frc.subsystems.Vision;

import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Twist2d;

import edu.wpi.first.wpilibj.DriverStation;


public class VisionPoseAcceptor {
    public boolean shouldAcceptVision(double timestamp, Pose2d visionFieldToVehicle, Twist2d robotVelocity) {


        boolean rotatingTooFast = Math.abs(robotVelocity.dtheta) >= 1.0;
        if (!rotatingTooFast) {
            return true;
        } else {
            return false;
        }
    }

}