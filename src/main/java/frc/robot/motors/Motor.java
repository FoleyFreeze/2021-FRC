package frc.robot.motors;

import frc.robot.cals.MotorCal;

public abstract class Motor{  

    public static Motor initMotor(MotorCal cal){

        if(cal.id < 0) return new MotorNull();

        switch(cal.type){
            case PWM_TALON:
                return new MotorTalonPWM(cal);

            case SPARK_MAX:
                return new MotorSparkMax(cal);

            case TALON_SRX:
                return new MotorTalonSRX(cal);

            default:
                return null;
        }
    }

    public abstract void setPower(double power);
    public abstract void setPosition(double position);
    public abstract double getPosition();
    public abstract void setSpeed(double speed);
    public abstract double getSpeed();
    public abstract double getCurrent();
    public abstract double getTemp();
    public abstract boolean isJammed();
    public abstract boolean getBrake();
    public abstract void setBrake(boolean brake);
}