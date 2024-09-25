package com.uni.frc.Autos.Actions;

import com.uni.frc.subsystems.SuperStructure;
import edu.wpi.first.wpilibj.Timer;

public class WaitForSuperstructureAction implements Action {
	private double mTimeToWait;
	private double mStartTime;

	public WaitForSuperstructureAction(double timeToWait) {
		mTimeToWait = timeToWait;
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - mStartTime >= mTimeToWait || SuperStructure.getInstance().requestsCompleted();
	}


	@Override
	public void start() {
	}

	@Override
	public void update() {}

	@Override
	public void done() {}
}