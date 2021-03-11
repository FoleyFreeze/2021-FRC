package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class BallSave extends CommandBase{
    RobotContainer m_subsystem;

    public BallSave(RobotContainer subsystem){
        m_subsystem = subsystem;
        addRequirements(subsystem.m_driveRot);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
