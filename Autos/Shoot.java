package com.uni.frc.Autos;



import com.uni.frc.subsystems.Shooter;
import com.uni.frc.subsystems.SuperStructure;
public class Shoot extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();


    @Override
    public void auto() {
        Shooter.getInstance().setPercent(0.8);

        s.setPivotState(0.083-.125);
        s.shootState(false);
    }
}