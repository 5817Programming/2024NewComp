package com.uni.lib.motion;


import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ChorTimeView {

    private ChoreoTrajectory mTrajectory;
    private double start_t;
    private double end_t;


    public ChorTimeView(ChoreoTrajectory path){
        this.mTrajectory = path;
        this.start_t = 0;
        this.end_t = path.getTotalTime();
    }

    public double first_interpolant(){
        return start_t;
    }
    public double last_interpolant(){
        return end_t;
    }

    public ChorPathPointState sample(double t){
        ChoreoTrajectoryState state = mTrajectory.sample(t);
        if(DriverStation.getAlliance().get().equals(Alliance.Red))
            state = state.flipped();

        Pose2d pose = new Pose2d(state.getPose());
        Translation2d velocity = new Translation2d(state.velocityX,state.velocityY);

        return new ChorPathPointState(pose, velocity, t);
    }

    public ChoreoTrajectory getTrajectory(){        
        return mTrajectory; 
    }
    
}
