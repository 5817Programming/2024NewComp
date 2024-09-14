package com.uni.lib.util;


public class PID2d{
    protected SynchronousPIDF x;
    protected SynchronousPIDF y;

    public PID2d(SynchronousPIDF xPID, SynchronousPIDF yPID){
        x = xPID;
        y = yPID;
    }

    public SynchronousPIDF x(){
        return x;
    }

    public SynchronousPIDF y(){
        return y;
    }


}