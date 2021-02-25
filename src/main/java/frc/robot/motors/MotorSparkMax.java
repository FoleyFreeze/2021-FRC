package frc.robot.motors;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.cals.MotorCal;



public class MotorSparkMax extends Motor{
    
    MotorCal cals;
    CANSparkMax motor;
    CANEncoder encoder;
    
    public MotorSparkMax(MotorCal cal){ 
        cals = cal;
        motor = new CANSparkMax(cal.id, 
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();

        brakeOn = cal.brake;
        if(cal.brake){
            motor.setIdleMode(IdleMode.kBrake);
        }else{
            motor.setIdleMode(IdleMode.kCoast);
        }

        motor.setOpenLoopRampRate(cal.rampRate);

        if(cals.pid){
            CANPIDController pid = motor.getPIDController();
            pid.setP(cal.kP);
            pid.setI(cal.kI);
            pid.setD(cal.kD);
            pid.setFF(cal.kF);
            pid.setDFilter(cal.kDFilt);
            pid.setOutputRange(cal.minPower, cal.maxPower);
            pid.setIZone(cal.ilim);
            if(cal.rampRate != 0){
                motor.setClosedLoopRampRate(cal.rampRate);
            }
        }
        
        motor.setInverted(cal.invert);

        encoder = motor.getEncoder();
        if(cal.resetEnc) encoder.setPosition(0);

        //save settings to the motor controllers
        //motor.burnFlash();
    }

    public void setPower(double power){
        if(power > 1) power = 1;
        else if(power < -1) power = -1;

        if(power > 0) power *= cals.maxPower;
        else if(power < 0) power *= -cals.minPower;

        checkJammed();
        if(isJammed()){
            motor.set(0);
        } else {
            motor.set(power);
        }
    }

    public void setPosition(double position){
        checkJammed();
        if(isJammed()){
            motor.set(0);
        } else {
            motor.getPIDController().setReference(position, 
            ControlType.kPosition);    
        }
    }

    double currentTimer = 0;
    int overCurrentCount = 0;
    public void checkJammed(){
        if(getCurrent() > cals.currentLimit){
            overCurrentCount++;
        } else {
            if(overCurrentCount > 0) overCurrentCount -= cals.overCurrentCountDown;
        }

        if(overCurrentCount > cals.overCurrentCountLimit){
            currentTimer = Timer.getFPGATimestamp() + cals.overCurrentRestTime;
        }

        if(getTemp() > cals.tempLimit){
            currentTimer = Timer.getFPGATimestamp() + cals.overTempRestTime;
        }

        //return Timer.getFPGATimestamp() < currentTimer;
    }

    public boolean isJammed(){
        return Timer.getFPGATimestamp() < currentTimer;
    }

    
    public double getPosition(){
        return encoder.getPosition();
    }

    public void setSpeed(double speed){
        checkJammed();
        if(isJammed()){
            motor.set(0);
        } else {
            motor.getPIDController().setReference(speed, 
            ControlType.kVelocity);
        }
    }

    public double getSpeed(){
        return encoder.getVelocity();
    }

    public double getCurrent(){
        return motor.getOutputCurrent();
    }

    public double getTemp(){
        return motor.getMotorTemperature();
    }

    boolean brakeOn;
    public boolean getBrake(){
        return brakeOn;
    }

    public void setBrake(boolean on){
        if(on){
            motor.setIdleMode(IdleMode.kBrake);
        } else {
            motor.setIdleMode(IdleMode.kCoast);
        }
        brakeOn = on;
    }
}