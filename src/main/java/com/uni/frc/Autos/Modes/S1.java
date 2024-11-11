package com.uni.frc.Autos.Modes;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.Autos.AutoBase;
import com.uni.frc.Autos.Actions.LambdaAction;
import com.uni.frc.Autos.Actions.TrajectoryAction;
import com.uni.frc.Autos.Actions.WaitAction;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class S1 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = 1;
    PathPlannerPath path = PathPlannerPath.fromPathFile("S1");
    
    

    PathPlannerTrajectory trajectory = addTrajectory(path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation)));
    


    @Override
    public void routine() {
        s.setContinuousShoot(true);
        runAction(new WaitAction(2));
        runAction(new LambdaAction(() -> s.shootState(false)));
        runAction(new WaitAction(7));
        runAction(new TrajectoryAction(trajectory));
    } 
}