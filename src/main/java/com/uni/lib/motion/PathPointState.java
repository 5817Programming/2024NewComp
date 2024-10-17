package com.uni.lib.motion;

import java.nio.channels.GatheringByteChannel;
import java.nio.file.Path;
import java.util.Optional;

import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.geometry.Twist2d;
import com.uni.lib.util.Util;

public class PathPointState {
    protected final Pose2d mPose;
    protected final Rotation2d mMotionDirection;
    protected final double mCurvature;
    protected final double mHeading_rate;
    protected final double mVelocity;
    protected final double mAcceleration;
    protected final double mT;

    public PathPointState(){
        mPose = Pose2d.identity();
        mMotionDirection = Rotation2d.identity();
        mCurvature = 0;
        mVelocity = 0;
        mAcceleration = 0;
        mHeading_rate = 0;
        mT = 0;
    }

    public PathPointState(Pose2d pose, Rotation2d motion_direction, double curvature, double velocity, double acceleration, double t, double heading_rate){
        this.mHeading_rate = heading_rate;
        this.mPose = pose;
        this.mMotionDirection = motion_direction;
        this.mCurvature = curvature;
        this.mVelocity = velocity;
        this.mAcceleration = acceleration;
        this.mT = t;
    }

    public Pose2d getPose(){
        return mPose;
    }   

    public PathPointState transformBy(Pose2d transform){
        return new PathPointState(mPose.transformBy(transform), mMotionDirection, mCurvature,mVelocity, mAcceleration, mT, mHeading_rate);
    }

    public PathPointState mirror(){
        return new PathPointState(mPose.mirror().getPose(), mMotionDirection.mirror(), -mCurvature, mVelocity, mAcceleration, mT, mHeading_rate);
    }

    public PathPointState mirrorAboutX(double x){
        return new PathPointState(mPose.mirrorAboutX(x), mMotionDirection.mirrorAboutX(), -mCurvature, mVelocity, mAcceleration, mT, mHeading_rate);
    }

    public PathPointState mirrorAboutY(double y){
        return new PathPointState(mPose.mirrorAboutY(y), mMotionDirection.mirrorAboutY(), -mCurvature, mVelocity, mAcceleration, mT, mHeading_rate);
    }

    public double getmCurvature(){
        return mCurvature;
    }

    public double getVelocity(){        
        return mVelocity;
    }

    public double getAcceleration(){
        return mAcceleration;
    }
   
    public Translation2d getTranslation(){
        return mPose.getTranslation();    
    }   

    public PathPointState interpolate(final PathPointState other, double x) {
        return new PathPointState(getPose().interpolate(other.getPose(), x),
                mMotionDirection.interpolate(other.mMotionDirection, x),
                Util.interpolate(getmCurvature(), other.getmCurvature(), x),
                Util.interpolate(getVelocity(), other.getVelocity(), x),
                Util.interpolate(getVelocity(), other.getVelocity(), x),
                Util.interpolate(mT, other.t(), x),
                Util.interpolate(getHeadingRate(), other.getHeadingRate(), x)
                );
    }

    public double getHeadingRate(){
        return mHeading_rate;
    }
    public double t(){
        return mT;
    }
    public PathPointState rotateBy(Rotation2d rotation){
        return new PathPointState(mPose.rotateBy(rotation), Rotation2d.identity(), mCurvature, mVelocity, mAcceleration, mT, mHeading_rate); 
    }
    
    public PathPointState add(PathPointState other){
        return this.transformBy(other.getPose());
    }
    
    public Rotation2d getCourse(){        
        return mMotionDirection;
    }



    
}
