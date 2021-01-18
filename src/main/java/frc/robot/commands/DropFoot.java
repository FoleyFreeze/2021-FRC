package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DropFoot extends CommandBase{

    public boolean m_activated;
    private RobotContainer m_subsystem;

    public DropFoot(RobotContainer subsystem, boolean activated){
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_cannonClimber);

        m_activated = activated;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(m_subsystem.m_input.enableBudClimb()) m_subsystem.m_cannonClimber.dropFoot(m_activated);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}