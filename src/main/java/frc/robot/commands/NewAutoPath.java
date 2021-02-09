package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class NewAutoPath extends CommandBase{
    
    private RobotContainer m_subsystem;   
    
    public NewAutoPath(RobotContainer subsystem){
        m_subsystem = subsystem;
        addRequirements(subsystem.m_drivetrain);
    }   

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.m_drivetrain.drive(new Vector(0, 0), 0, 0, 0, true);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
