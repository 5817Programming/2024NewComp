package com.uni.frc.Autos.Modes;



import com.uni.frc.Autos.AutoBase;
import com.uni.frc.Autos.Actions.LambdaAction;
import com.uni.frc.Autos.Actions.WaitAction;
import com.uni.frc.subsystems.Shooter;
import com.uni.frc.subsystems.SuperStructure;
public class Shoot extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();


    @Override
    public void routine() {
        s.setContinuousShoot(true);
        runAction(new WaitAction(2));
        runAction(new LambdaAction(() -> s.shootState(false)));
    }
}