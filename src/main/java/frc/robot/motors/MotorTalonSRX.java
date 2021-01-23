package frc.robot.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

import frc.robot.cals.MotorCal;



public class MotorTalonSRX extends Motor{
    
    TalonSRX motor;
    MotorCal cal;

    public MotorTalonSRX(MotorCal cal){
        this.cal = cal;
        motor = new TalonSRX(cal.id);

        motor.configFactoryDefault();

        motor.setInverted(cal.invert);
        
        if(cal.brake){
            motor.setNeutralMode(NeutralMode.Brake);
        } else {
            motor.setNeutralMode(NeutralMode.Coast);
        }

        motor.configOpenloopRamp(cal.rampRate);

        if(cal.pid){
            motor.configClosedloopRamp(cal.rampRate);
            motor.config_kF(0, cal.kF);
            motor.config_kP(0, cal.kP);
            motor.config_kI(0, cal.kI);
            motor.config_kD(0, cal.kD);
        }

        if(cal.follow){
            motor.set(ControlMode.Follower, cal.followID);
            return;
        }
    }

    public void setPower(double power){
        if(cal.follow) {
            motor.set(ControlMode.Follower, cal.followID);
            return;
        }

        if(power > cal.maxPower) power = cal.maxPower;
        if(power < cal.minPower) power = cal.minPower;
        motor.set(ControlMode.PercentOutput, power);
    }

    public void setPosition(double position){
        if(cal.follow) {
            motor.set(ControlMode.Follower, cal.followID);
            return;
        }

        motor.set(ControlMode.Position, position);
    }

    public double getPosition(){
        return motor.getSelectedSensorPosition();
    }    
    
    public void setSpeed(double speed){
        if(cal.follow) {
            motor.set(ControlMode.Follower, cal.followID);
            return;
        }
        
        motor.set(ControlMode.Position, speed * 2048/600.0);
    }
    
    public double getCurrent(){
        return motor.getStatorCurrent();
    }

    public double getTemp(){
        return 0;
    }

    @Override
    public double getSpeed() {
        return motor.getSelectedSensorVelocity() * 600/2048.0;
    }

    public boolean isJammed(){
        return false;
    }

    boolean brakeOn;
    @Override
    public boolean getBrake(){
        return brakeOn;
    }

    @Override
    public void setBrake(boolean brake){
        if(brake){
            motor.setNeutralMode(NeutralMode.Brake);
        } else {
            motor.setNeutralMode(NeutralMode.Coast);
        }
        brakeOn = brake;
    }
}