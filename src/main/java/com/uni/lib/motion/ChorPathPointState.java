package com.uni.lib.motion;


import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.util.Util;

public class ChorPathPointState {
    protected final Pose2d mPose;
    protected final Translation2d mVelocity;
    protected final double mT;

    public ChorPathPointState(){
        mPose = Pose2d.identity();
        mVelocity = Translation2d.identity();
        mT = 0;
    }

    public ChorPathPointState(Pose2d pose, Translation2d velocity,  double t){
        this.mPose = pose;
        this.mVelocity = velocity;
        this.mT = t;
    }

    public Pose2d getPose(){
        return mPose;
    }   

    public ChorPathPointState transformBy(Pose2d transform){
        return new ChorPathPointState(mPose.transformBy(transform),mVelocity,  mT);
    }

    public ChorPathPointState mirror(){
        return new ChorPathPointState(mPose.mirror(), mVelocity.inverse(), mT);
    }

    public Translation2d getVelocity(){        
        return mVelocity;
    }
  
    public Translation2d getTranslation(){
        return mPose.getTranslation();    
    }   

   public double t(){
        return mT;
    }
   
    public ChorPathPointState add(ChorPathPointState other){
        return this.transformBy(other.getPose());
    }
    


    
}
