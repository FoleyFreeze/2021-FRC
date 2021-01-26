package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.cals.CannonCals;
import frc.robot.cals.ClimberCals;
import frc.robot.motors.Motor;
import frc.robot.util.Util;

public class CannonClimber extends SubsystemBase{

    public CannonCals shootCals;
    public ClimberCals climbCals;
    private Motor motor;
    private Motor motor2;
    public enum HoodPos{
        LOW, MID1, MID2, HIGH
    }
    private Solenoid hoodSol;
    private Solenoid stopSol;
    private Solenoid camLightsSol;

    private Solenoid shootVsClimb;

    private Solenoid dropFoot;

    HoodPos hCurrPos = HoodPos.LOW;
    public HoodPos hTgtPos = HoodPos.LOW;
    double solRestTime = 0;

    double targetSpeed = 0;

    RobotContainer subsystem;

    public CannonClimber(RobotContainer sub, CannonCals sCals, ClimberCals cCals){
        subsystem = sub;

        shootCals = sCals;
        climbCals = cCals;
        if(sCals.disabled && cCals.disabled) return;

        motor = Motor.initMotor(shootCals.ccMotor);
        motor2 = Motor.initMotor(shootCals.ccMotor2);

        hoodSol = new Solenoid(sCals.hoodSolValue);
        stopSol = new Solenoid(sCals.stopSolValue);
        camLightsSol = new Solenoid(sCals.camLightsSol);
        shootVsClimb = new Solenoid(sCals.ShootVClimbValue);
        dropFoot = new Solenoid(cCals.dropFootValue);
    }
    
    public void setpower(double power){
        if(shootCals.disabled) return;
        motor.setPower(power);
        //motor2.setPower(power);
        Display.put("CCMotorCurrent 0", motor.getCurrent());
        Display.put("CCMotorCurrent 1", motor2.getCurrent());
        Display.put("CC Motor Temp 0", motor.getTemp());
        Display.put("CC Motor Temp 1", motor2.getTemp());
        
        targetSpeed = 0;
    }

    public void setspeed(double speed){
        if(shootCals.disabled) return;
        //motor.setSpeed(speed);
        //motor2.setSpeed(speed);
        motor.setPower(speed / shootCals.falconRpmPerPower);
        targetSpeed = speed;
    }

    public void prime(double distToTgt){
        distToTgt += shootCals.initJogDist;
        if(shootCals.disabled) return;
        if(!climbCals.disabled) shootVsClimb.set(false);
        
        double[] distAxis = shootCals.dist[hTgtPos.ordinal()];
        if(distAxis[0] > distToTgt){
            switch(hTgtPos){
                case LOW:
                    hTgtPos = HoodPos.LOW;
                    break;
                
                case MID1:
                    hTgtPos = HoodPos.LOW;
                    break;

                case MID2:
                    hTgtPos = HoodPos.MID1;
                    break;

                case HIGH:
                    hTgtPos = HoodPos.MID2;
                    break;
            }
        }

        if(distAxis[distAxis.length - 1] < distToTgt){
            switch(hTgtPos){
                case LOW:
                    hTgtPos = HoodPos.MID1;
                    break;

                case MID1:
                    hTgtPos = HoodPos.MID2;
                    break;

                case MID2:
                    hTgtPos = HoodPos.HIGH;
                    break;
                    
                case HIGH:
                    hTgtPos = HoodPos.HIGH;
                    break;
            }
        }
    

        double speed = Util.interpolate(shootCals.rpm[hTgtPos.ordinal()], shootCals.dist[hTgtPos.ordinal()], distToTgt);
        setspeed(speed);
    }

    public boolean ready(){
        if(subsystem.m_transporterCW.launcher.get()){
            return Math.abs(motor.getSpeed() - targetSpeed) < shootCals.allowedRpmHyst;
        } else {
            return Math.abs(motor.getSpeed() - targetSpeed) < shootCals.allowedRpmError;
        }
    }

    public void periodic(){
        if(DriverStation.getInstance().isDisabled()){
            hTgtPos = HoodPos.LOW;
        }

        if(DriverStation.getInstance().isAutonomous()){
            hTgtPos = HoodPos.HIGH;
        }

        if(climbCals.disabled && shootCals.disabled) return;
        
        if(Timer.getFPGATimestamp()>solRestTime){
            switch(hTgtPos){
                case LOW:
                    if(hCurrPos != HoodPos.LOW){
                        hoodSol.set(false);
                        stopSol.set(false);
                        hCurrPos = HoodPos.LOW;
                        solRestTime = Timer.getFPGATimestamp() + shootCals.SOL_RESTTIME;
                    }
                    break;

                case MID1:
                    if(hCurrPos != HoodPos.MID1){
                        if(hCurrPos == HoodPos.LOW){
                            hoodSol.set(true);
                            stopSol.set(true);
                            hCurrPos = HoodPos.MID1;
                            solRestTime = Timer.getFPGATimestamp() + shootCals.SOL_RESTTIME;
                        }else{
                            hoodSol.set(false);
                            stopSol.set(false);
                            hCurrPos = HoodPos.LOW;
                            solRestTime = Timer.getFPGATimestamp() + shootCals.SOL_RESTTIME;
                        }
                    }
                    break;

                case MID2:
                    if(hCurrPos != HoodPos.MID2){
                        if(hCurrPos == HoodPos.HIGH){
                            hoodSol.set(false);
                            stopSol.set(true);
                            hCurrPos = HoodPos.MID2;
                            solRestTime = Timer.getFPGATimestamp() + shootCals.SOL_RESTTIME;
                        }else{
                            hoodSol.set(true);
                            stopSol.set(false);
                            hCurrPos = HoodPos.HIGH;
                            solRestTime = Timer.getFPGATimestamp() + shootCals.SOL_RESTTIME;
                        }
                    }
                    break;

                case HIGH:
                    if(hCurrPos != HoodPos.HIGH){
                        hoodSol.set(true);
                        stopSol.set(false);
                        hCurrPos = HoodPos.HIGH;
                        solRestTime = Timer.getFPGATimestamp() + shootCals.SOL_RESTTIME;
                    }
                    break;
            }
        }
        Display.put("Foot Dropped", dropFoot.get());
        Display.put("CCMotorCurrent 0", motor.getCurrent());
        Display.put("CCMotorCurrent 1", motor2.getCurrent());
        Display.put("CC Motor Temp 0", motor.getTemp());
        Display.put("CC Motor Temp 1", motor2.getTemp());
        Display.put("RPM", motor.getSpeed());
        Display.put("Hood Pos", hCurrPos.toString());
    }

    public void jogUpDn(boolean up){
        if(up){
            shootCals.initJogDist += shootCals.distJog;
        }else{
            shootCals.initJogDist -= shootCals.distJog;
        }
        Display.put("JogUpDn", shootCals.initJogDist);
    }

    public void jogLR(boolean left){
        if(left){
            shootCals.initJogAng -= shootCals.angJog;
        }else{
            shootCals.initJogAng += shootCals.angJog;
        }
        Display.put("JogLR", shootCals.initJogAng);
    }

    //Climber
    public void climbMode(boolean set){
        if(climbCals.disabled && shootCals.disabled) return;
        shootVsClimb.set(set);
    }
    public void dropFoot(boolean on){
        if(climbCals.disabled && shootCals.disabled) return;
        dropFoot.set(on);
    }
    public void setCamLights(boolean on){
        camLightsSol.set(on);
    }
}