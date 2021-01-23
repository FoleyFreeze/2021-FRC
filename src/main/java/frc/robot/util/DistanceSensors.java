package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.I2CDistSense.Port;
import frc.robot.util.I2CDistSense.Unit;

public class DistanceSensors{

    public boolean disabled = true;

    public class DistData{
        public double dist;
        public double angle;
        public double timestamp;

        public String toString(){
            return String.format("T: %.0f, D: %.1f, \u03B8: %.0f", Timer.getFPGATimestamp() - timestamp, dist, angle);
        }
    }

    public I2CDistSense rightFront;
    public I2CDistSense rightBack;
    public I2CDistSense rearLeft;
    public I2CDistSense rearRight;

    public double distBetSenseRight = 12;
    public double distBetSenseRear = 12;

    public DistanceSensors(){
        if(disabled) return;

        rightFront = new I2CDistSense(Port.kOnboard);
        rightBack = new I2CDistSense(Port.kOnboard);
        rearLeft = new I2CDistSense(Port.kOnboard);
        rearRight = new I2CDistSense(Port.kOnboard);
    }

    DistData rear = new DistData();

    public DistData getRear(){
        if(disabled) return rear;

        if(Math.abs(rearLeft.getTimestamp() - rearRight.getTimestamp()) <= 0.2){
            rear.timestamp = (rearLeft.getTimestamp() + rearRight.getTimestamp()) 
                / 2;
            rear.dist = (rearLeft.getRange(Unit.kInches) + 
                rearRight.getRange(Unit.kInches)) / 2;
            rear.angle = Math.toDegrees(Math.atan2(rearRight.getRange(Unit.kInches) 
                - rearLeft.getRange(Unit.kInches), distBetSenseRear));
        }
        return rear;
    }

    DistData right = new DistData();

    public DistData getRight(){
        if(disabled) return right;

        if(Math.abs(rightBack.getTimestamp() - rightFront.getTimestamp()) <= 0.2){
            right.timestamp = (rightBack.getTimestamp() + rightFront.getTimestamp()) 
                / 2.0;
            right.dist = (rightBack.getRange(Unit.kInches) + 
                rightFront.getRange(Unit.kInches)) / 2.0;
            right.angle = Math.toDegrees(Math.atan2(rightFront.getRange(Unit.kInches) 
                - rightBack.getRange(Unit.kInches), distBetSenseRight));
        }
        return right;
    }
}