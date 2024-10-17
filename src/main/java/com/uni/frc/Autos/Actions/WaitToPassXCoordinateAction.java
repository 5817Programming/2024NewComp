package com.uni.frc.Autos.Actions;

import com.uni.frc.Constants;
import com.uni.frc.subsystems.Swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;

public class WaitToPassXCoordinateAction implements Action {
	double startingXCoordinate;
	double targetXCoordinate;
	SwerveDrive drive;

	public WaitToPassXCoordinateAction(double x) {
		if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
			targetXCoordinate = Constants.FieldConstants.kFieldLength - x;
		} else {
			targetXCoordinate = x;
		}
		drive = SwerveDrive.getInstance();
	}

	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate)
				!= Math.signum(drive.getPose().getTranslation().x() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = drive.getPose().getTranslation().x();
	}

	@Override
	public void update() {}

	@Override
	public void done() {}
}