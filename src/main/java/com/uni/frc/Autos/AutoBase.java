package com.uni.frc.Autos;

import java.util.List;
import java.util.ArrayList;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.Autos.Actions.Action;

public abstract class AutoBase{

    protected List<PathPlannerTrajectory> trajectories = new ArrayList<>();

    boolean mActive = false;
    public void start() {
        mActive = true;
        routine();
   }
	public abstract void routine();
    public void stop() {
        mActive = false;
    }

    public void runAction(Action action){
        action.start();
        if(!mActive)
            throw new Error("Action ran while auto is inactive");
        while(mActive && !action.isFinished()){
            action.update();
            
            try{
                Thread.sleep(20);
            }
            catch(InterruptedException e){
                e.printStackTrace();
            }
        } 
        if(!mActive)
            throw new Error("Action Inturupted");
        action.done();
    }


    public PathPlannerTrajectory addTrajectory(PathPlannerTrajectory path){
        trajectories.add(path);
        return path;
    }

    public List<PathPlannerTrajectory> getTrajectory(){
        return trajectories;
    }








}