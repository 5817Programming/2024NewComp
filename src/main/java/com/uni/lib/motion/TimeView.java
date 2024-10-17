package com.uni.lib.motion;

import java.security.cert.X509CRL;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.geometry.Twist2d;

import edu.wpi.first.units.Velocity;
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

        Pose2d pose = new Pose2d(new Translation2d(state.positionMeters), new Rotation2d(state.targetHolonomicRotation).inverse());
        double headingRate = 0;
        double velocity = state.velocityMps;
        double curvature = state.curvatureRadPerMeter;
        Rotation2d motion_direction = new Rotation2d(state.heading).flip().inverse();
        if(DriverStation.getAlliance().get() == Alliance.Red){
            pose = pose.mirrorAboutX(8.25);
            headingRate *= -1;
            motion_direction = motion_direction.inverse();
            velocity *= -1;
        }

        return new PathPointState(pose, motion_direction, curvature, velocity, state.accelerationMpsSq, t, headingRate);
    }

    public PathPlannerTrajectory getTrajectory(){        
        return mTrajectory; 
    }
    
}
