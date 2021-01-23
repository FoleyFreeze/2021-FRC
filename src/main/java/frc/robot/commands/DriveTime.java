package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class DriveTime extends CommandBase{
    double time;
    double x,y,r;
    RobotContainer sub;
    double endTime;

    public DriveTime(double time, RobotContainer subsystem, double x, double y, double r){
        this.time = time;
        sub = subsystem;
        this.x = x;
        this.y = y;
        this.r = r;
    }

    @Override
    public void initialize(){
        endTime = Timer.getFPGATimestamp() + time;
    }

    @Override
    public void execute(){
        sub.m_drivetrain.drive(Vector.fromXY(x, y), r);
    }

    @Override
    public void end(boolean interrupted){
        sub.m_drivetrain.drive(Vector.fromXY(0,0), 0);
    }

    @Override
    public boolean isFinished(){
        return Timer.getFPGATimestamp() > endTime;
    }
}