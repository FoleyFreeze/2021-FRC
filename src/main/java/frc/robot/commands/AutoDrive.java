package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.cals.DriverCals;
import frc.robot.util.Util;
import frc.robot.util.Vector;

public class AutoDrive extends CommandBase{

    private RobotContainer m_subsystem;
    private double tgtX;
    private double tgtY;
    private double tgtRot;
    private double deltaX;
    private double deltaY;
    private double errorX;
    private double errorY;
    private double startX;
    private double startY;
    private double errorRot;
    private boolean deltaVsField;
    
    private DriverCals mCals;

    public AutoDrive(RobotContainer subsystem, double deltaX, double deltaY, double angle, boolean deltaVsField){
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_driveStrafe);
        addRequirements(m_subsystem.m_driveRot);
        mCals = m_subsystem.m_drivetrain.k;
        this.deltaX = deltaX;
        this.deltaY = deltaY;
        this.tgtRot = angle;
        this.deltaVsField = deltaVsField;//if true, it's delta
    }

    @Override
    public void initialize(){
        Pose2d pose = m_subsystem.m_drivetrain.drivePos;
        startX = pose.getTranslation().getX();
        startY = pose.getTranslation().getY();
        if(deltaVsField){
            tgtX = startX + deltaX;
            tgtY = startY + deltaY;
        } else {
            tgtX = deltaX;
            tgtY = deltaY;
        }
        SmartDashboard.putNumber("AutoXtgt",tgtX);
        SmartDashboard.putNumber("AutoYtgt",tgtY);
    }

    @Override
    public void execute(){
        Pose2d pose = m_subsystem.m_drivetrain.drivePos;
        double x = pose.getTranslation().getX();
        double y = pose.getTranslation().getY();
        errorX = (tgtX - x);
        errorY = (tgtY - y);
        errorRot = Util.angleDiff(tgtRot, m_subsystem.m_drivetrain.robotAng);

        Vector strafe = Vector.fromXY(errorX* mCals.autoDriveStrafeKp, errorY * mCals.autoDriveStrafeKp);

        double power = mCals.autoDriveMaxPwr;
        double dfsX = x - startX;
        double dfsY = y - startY;
        double distFromStart = Math.sqrt(dfsX*dfsX + dfsY*dfsY);
        double distToTarget = Math.sqrt(errorX*errorX + errorY*errorY);
        double startPwr = ((power - mCals.autoDriveStartPwr)/(mCals.autoDriveStartDist)) * distFromStart 
            + mCals.autoDriveStartPwr;
        double endPwr = ((power - mCals.autoDriveEndPwr)/(mCals.autoDriveEndDist)) * distToTarget + mCals.autoDriveEndPwr;
        power = Util.min(power, startPwr, endPwr);

        m_subsystem.m_drivetrain.drive(strafe, -errorRot * mCals.autoDriveAngKp, 0, 0, true, power);

        SmartDashboard.putNumber("AutoXerr",errorX);
        SmartDashboard.putNumber("AutoYerr",errorY);
        SmartDashboard.putNumber("AutoAngerr",errorRot);
        SmartDashboard.putNumber("DistFromStart",distFromStart);
        SmartDashboard.putNumber("DistToTarget",distToTarget);
        SmartDashboard.putNumber("AutoPower",power);
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.m_drivetrain.drive(new Vector(0, 0), 0, 0, 0, true);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(errorX) < mCals.autoDriveStrafeRange 
            && Math.abs(errorY) < mCals.autoDriveStrafeRange 
            && Math.abs(errorRot) < mCals.autoDriveAngRange;
    }
}