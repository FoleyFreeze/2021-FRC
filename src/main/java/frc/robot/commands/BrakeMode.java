package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class BrakeMode extends CommandBase{

    RobotContainer m_subsystem;
    boolean on;

    public BrakeMode(RobotContainer subsystem, boolean on){
        m_subsystem = subsystem;
        //addRequirements(m_subsystem.m_drivetrain);
        this.on = on;
    }

    @Override
    public void initialize(){
        m_subsystem.m_drivetrain.setBrake(on);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return on == m_subsystem.m_drivetrain.getBrake();
    }
}
