package frc.robot.cals;

import frc.robot.cals.MotorCal.MotorType;

public class IntakeCals extends CalSet {

    public boolean disabled = false;
    public MotorCal spinMotor = new MotorCal(MotorType.SPARK_MAX, 6).currLim(40).currLimCount(20).currLimTime(0.1).currLimCntDn(5);
    //public MotorCal spinMotor = new MotorCal(MotorType.SPARK_MAX, 6).pid(0.0004, 0, 0, 0.0003).ramp(10).limit(0, 1);
    public int depSolValue = 7;
    public double forwardPower = 0.65;//worked at 65
    public double forwardSpeed = 100;//1500;//JNC CHANGE from 500
    public double backwardPower = -0.2;
    public double idxPower = 0.2;
    public double jamRestTime = 0.1;
    public double unjamPwr = -0.0;

    public IntakeCals(){

        switch(type){
            case COMPETITION:

            break;

            case PRACTICE:
                disabled = true;
            break;

            case LASTYEAR:

            break;
        }
    }
}