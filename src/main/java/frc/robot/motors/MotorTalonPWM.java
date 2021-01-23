package frc.robot.motors;

import edu.wpi.first.wpilibj.Talon;
import frc.robot.cals.MotorCal;


public class MotorTalonPWM extends Motor {

    Talon motor;

    public MotorTalonPWM(MotorCal cal){
        motor = new Talon(cal.id);
    }

    public void setPower(double power){
        motor.set(power);
    }

    public void setPosition(double position){
        throw new UnsupportedOperationException();
    }

    public double getPosition(){
        return 0;
    }

    public void setSpeed(double speed){
        throw new UnsupportedOperationException();
    }

    public double getSpeed(){
        return 0;
    }

    public double getCurrent(){
        return 0;
    }

    public double getTemp(){
        return 0;
    }

    public boolean isJammed(){
        return false;
    }

    @Override
    public boolean getBrake() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void setBrake(boolean brake) {
        // TODO Auto-generated method stub

    }
}