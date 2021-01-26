package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.cals.IntakeCals;
import frc.robot.motors.Motor;

public class Intake extends SubsystemBase {

    public Motor spinmotor;
    public Solenoid depSol;
    public IntakeCals mCals;

    public Intake(IntakeCals cals){
        this.mCals = cals;
        if(mCals.disabled) return;
        spinmotor = Motor.initMotor(mCals.spinMotor);
        depSol = new Solenoid(mCals.depSolValue);
    }

    double jamTime;
    boolean prevJammed = false;
    public void periodic(){
        if(mCals.disabled) return;
        Display.put("InMotorCurr", spinmotor.getCurrent());
    }
    
    public void setPower(double power){
        if(mCals.disabled) return;

        /*if(Timer.getFPGATimestamp() > jamTime){
            if(prevJammed) {
                prevJammed = false;
            }
        } 
        if(spinmotor.isJammed() && !prevJammed){
            jamTime = Timer.getFPGATimestamp() + mCals.jamRestTime;
            power = mCals.unjamPwr;
            prevJammed = true;
        }*/
        if(power == mCals.forwardPower){
            spinmotor.setPower(power);
        } else {
            spinmotor.setPower(power);
        }
        
    }
    public void setSpeed(double speed){
        if(mCals.disabled) return;
        spinmotor.setSpeed(speed);
    }
    
    public void dropIntake(boolean activate){
        if(mCals.disabled) return;
        depSol.set(activate);
    }

    public boolean isOut(){
        if(mCals.disabled) return false;
        return depSol.get();
    }
}