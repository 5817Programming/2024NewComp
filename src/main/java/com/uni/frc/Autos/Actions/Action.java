package com.uni.frc.Autos.Actions;

public interface Action {
	boolean isFinished();
	void update();
	void done();
	void start();
}
