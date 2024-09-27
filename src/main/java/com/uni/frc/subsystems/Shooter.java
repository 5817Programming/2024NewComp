package com.uni.frc.subsystems;



import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.uni.frc.Ports;
import com.uni.frc.Constants.ShooterConstants;
import com.uni.frc.subsystems.Requests.Request;
import com.uni.lib.TalonConfigs;


 public class Shooter extends Subsystem {
   private PeriodicIO mPeriodicIO = new PeriodicIO();
   private TalonFX shooterMotor1 = new TalonFX(Ports.shooter1, "Minivore");
   private TalonFX shooterMotor2 = new TalonFX(Ports.shooter2, "Minivore");
   public State currentState = State.IDLE;
   private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
   private double spinMultiplier = 1;
   public static Shooter instance = null;

   public static Shooter getInstance() {
     if (instance == null)
       instance = new Shooter();
     return instance;
   }

   /** Creates a new shooter. */
   public Shooter() {
     configMotors();
   }

   public enum State{
    PARTIALRAMP(.7),
    SHOOTING(1),
    TRANSFER(0.3),
    REVERSETRANSFER(-.5),
    AMP(.5),
    IDLE(.1);

    double output = 0;
    State(double output){
        this.output = output;
    }
   }

   public void setRamp(double rampTime) {
     shooterMotor1.getConfigurator().refresh(shooterConfig);
     shooterMotor2.getConfigurator().refresh(shooterConfig);

     shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;

     shooterMotor1.getConfigurator().apply(shooterConfig);
     shooterMotor2.getConfigurator().apply(shooterConfig);
   }

   public void configMotors() {
     shooterConfig = TalonConfigs.shooterConfigs();
     shooterMotor1.getConfigurator().apply(shooterConfig);
     shooterMotor2.getConfigurator().apply(shooterConfig);

   }

   public void setPercent(double Percentage) {
     currentState = State.SHOOTING;
     mPeriodicIO.driveDemand = Percentage;
   }


  
   public void setAcceleration(double accelerationDemand){
     mPeriodicIO.accelerationDemand = accelerationDemand;
   }


  

   public Request setPercentRequest(double percentage) {
     return new Request() {

       @Override
       public void act() {

         setPercent(percentage);
       }

     };

   }
   public void conformToState(State state){
    currentState = state;
    setPercent(state.output);
   }
 public Request stateRequest(State state){
  return new Request() {
    @Override
    public void act() {
        setPercent(state.output);
    }
  };
 }
 public Request setAccelerationRequest(double percentage) {
     return new Request() {

       @Override
       public void act() {
         setAcceleration(percentage);
       }

     };

   }



   public void setSpin(double spinMultiplier){
    this.spinMultiplier = spinMultiplier;
   }




   public double getStatorCurrent() {
     return mPeriodicIO.statorCurrent;
   }

   public Request atTargetRequest(){
    return new Request(){
      @Override
      public boolean isFinished() {
          return Math.abs(shooterMotor1.getVelocity().getValueAsDouble())>(70*mPeriodicIO.driveDemand) ;
      }
    };
   }
   public Request atTargetRequest(double percent){
    return new Request(){
      @Override
      public boolean isFinished() {
          return Math.abs(shooterMotor1.getVelocity().getValueAsDouble())>(80*percent);
      }
    };
   }

   @Override
   public void writePeriodicOutputs() {
     mPeriodicIO.drivePosition = shooterMotor1.getPosition().getValueAsDouble();
     mPeriodicIO.velocity = shooterMotor1.getVelocity().getValueAsDouble();
     mPeriodicIO.statorCurrent = (shooterMotor1.getStatorCurrent().getValueAsDouble()+shooterMotor2.getStatorCurrent().getValueAsDouble())/2;
   }

   @Override
   public void readPeriodicInputs() {
      shooterMotor1.setControl(new DutyCycleOut(mPeriodicIO.driveDemand).withEnableFOC(true));
      shooterMotor2.setControl(new DutyCycleOut(-mPeriodicIO.driveDemand*(spinMultiplier)).withEnableFOC(true));
    }

   @Override
   public void outputTelemetry() {
   }

   @Override
   public void stop() {
     setPercentRequest(0);
   }

   public static class PeriodicIO {
     double drivePosition = 0;
     double velocity = 0;
     double statorCurrent = 0;
     double accelerationDemand = 0;
     double driveDemand = 0.0;
   }
 }
