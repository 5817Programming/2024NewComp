package com.uni.lib.motion;

import com.pathplanner.lib.path.PathPlannerTrajectory;


public class TrajectoryIterator {

    protected TimeView mTimeView;
    protected double progress_ = 0.0;
    protected PathPointState currentState_;

    public TrajectoryIterator(TimeView timeView){
        this.mTimeView = timeView;
        this.currentState_ = timeView.sample(timeView.first_interpolant());
        progress_ = timeView.first_interpolant();
   }


   public boolean isDone(){
     return getRemainingProgress() == 0.0;

   }

   public double getRemainingProgress(){
       return Math.max(0.0, mTimeView.last_interpolant() - progress_);
   }

   public PathPointState getCurrentState(){
       return currentState_;
   }

   public PathPointState advance(double additional_progress){

        progress_ = Math.max(mTimeView.first_interpolant(), Math.min(mTimeView.last_interpolant(), progress_ + additional_progress));

        currentState_ = mTimeView.sample(progress_);
        return currentState_;
   }    

   public TimeView getTimeView(){
       return mTimeView;
   }

    public PathPointState preview(double additional_progress){
        final double progress = Math.max(mTimeView.first_interpolant(), Math.min(mTimeView.last_interpolant(), progress_ + additional_progress));

        return mTimeView.sample(progress);
    }       

    public PathPlannerTrajectory trajectory(){
        return mTimeView.getTrajectory();
    }
   

}
