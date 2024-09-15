package com.uni.frc.Autos.Actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class SeriesAction implements Action {
	private Action mCurrentAction;
	private final ArrayList<Action> mRemainingActions;

	public SeriesAction(List<Action> actions) {
		mRemainingActions = new ArrayList<>(actions.size());
		mRemainingActions.addAll(actions);
		mCurrentAction = null;
	}

	public SeriesAction(Action... actions) {
		this(Arrays.asList(actions));
	}

	@Override
	public void start() {}

	@Override
	public void update() {
		if (mCurrentAction == null) {
			if (mRemainingActions.isEmpty()) {
				return;
			}

			mCurrentAction = mRemainingActions.remove(0);
			mCurrentAction.start();
		}

		mCurrentAction.update();

		if (mCurrentAction.isFinished()) {
			mCurrentAction.done();
			mCurrentAction = null;
		}
	}

	@Override
	public boolean isFinished() {
		return mRemainingActions.isEmpty() && mCurrentAction == null;
	}

	@Override
	public void done() {}
}
