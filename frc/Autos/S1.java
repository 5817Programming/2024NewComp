package com.uni.frc.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.Shooter;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.SuperStructure.SuperState;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class S1 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = -60;
    PathPlannerPath path = PathPlannerPath.fromPathFile("S1");
    PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {

        Shooter.getInstance().setPercent(.8);
        s.setState(SuperState.AUTO);
        s.setContinuousShootState(false);
        s.setPivotState(60);
        s.waitState(0.3, false);

        s.shootState(false);
        s.setContinuousShootState(true);

        s.trajectoryState(trajectory, initRotation);
        registerTrajectoryEvents("S1");
    }

}