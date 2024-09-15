package com.uni.frc.Autos.Actions;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.motion.TimeView;
import com.uni.lib.motion.TrajectoryIterator;

public class TrajectoryAction implements Action{

	private SwerveDrive mDrive = null;


	private TrajectoryIterator mTrajectory;
	private boolean mResetGyro;

	public TrajectoryAction(PathPlannerTrajectory path){
		this(path, false);
	}

	public TrajectoryAction(PathPlannerTrajectory path, boolean resetPos){
		this.mTrajectory = new TrajectoryIterator(new TimeView(path));
		this.mResetGyro = resetPos;
		this.mDrive = SwerveDrive.getInstance();
	}

	@Override
	public void start(){
		if(mResetGyro){
			Rotation2d init_rot = mTrajectory.getCurrentState().getPose().getRotation();
			mDrive.resetGryo(init_rot.getDegrees());
		}
		
		mDrive.setTrajectory(mTrajectory);
	}

	@Override
	public boolean isFinished() {
		return mDrive.isTrajectoryFinished();
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
	}
}