package com.uni.frc.Autos;

public class AutoExecuter{

    private Thread mThread = null;

    private AutoBase mAuto = null;


    public void setAuto(AutoBase auto){
    this.mAuto = auto;
    }

    public AutoBase getAuto(){
        return this.mAuto;
    }

    public void start(){
        if(mThread == null){
            mThread = new Thread(new Runnable(){
                @Override
                public void run(){
                    if(mAuto != null)
                        mAuto.start();   
                }
            });
            mThread.start();
        }
   }

    public void stop(){
        if(mAuto != null){
          mAuto.stop();
        }
        mThread = null;
    }
} 