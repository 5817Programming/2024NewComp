package com.uni.lib.motion;

import java.security.cert.X509CRL;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.geometry.Twist2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TimeView {

    private PathPlannerTrajectory mTrajectory;
    private double start_t;
    private double end_t;


    public TimeView(PathPlannerTrajectory path){
        this.mTrajectory = path;
        this.start_t = 0;
        this.end_t = path.getTotalTimeSeconds();
    }

    public double first_interpolant(){
        return start_t;
    }
    public double last_interpolant(){
        return end_t;
    }

    public PathPointState sample(double t){
        State state = mTrajectory.sample(t);

        Pose2d pose = new Pose2d(new Translation2d(state.positionMeters), new Rotation2d(state.targetHolonomicRotation));
        double xVel = state.velocityMps * Math.cos(state.heading.getRadians());
        double yVel = state.velocityMps * Math.sin(state.heading.getRadians());
        double headingRate = state.holonomicAngularVelocityRps.get();
        double curvature = state.curvatureRadPerMeter;
        Rotation2d motion_direction = new Rotation2d(state.heading);
        if(DriverStation.getAlliance().get() == Alliance.Red){
            pose = pose.mirrorAboutX(8.25);
            xVel *= -1;
            headingRate *= -1;
            motion_direction = motion_direction.flip();
        }


        return new PathPointState(pose, motion_direction, curvature, state.velocityMps/2, state.accelerationMpsSq/2, t, headingRate/2);
    }

    public PathPlannerTrajectory getTrajectory(){        
        return mTrajectory; 
    }
    
}
