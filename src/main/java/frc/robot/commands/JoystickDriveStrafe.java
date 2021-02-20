package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class JoystickDriveStrafe extends CommandBase {
 
    private RobotContainer m_subsystem;
    private Vector strafePwr;

    public JoystickDriveStrafe(RobotContainer subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_driveStrafe);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        strafePwr = m_subsystem.m_input.getXY();
        m_subsystem.m_drivetrain.driveStrafe(strafePwr,
            m_subsystem.m_drivetrain.k.joyMaxPwr);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
