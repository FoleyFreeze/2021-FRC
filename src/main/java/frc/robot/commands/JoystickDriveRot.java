package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class JoystickDriveRot extends CommandBase {
 
    private RobotContainer m_subsystem;
    private double rotPwr;

    public JoystickDriveRot(RobotContainer subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_driveRot);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        rotPwr = m_subsystem.m_input.getRot();
        m_subsystem.m_drivetrain.driveRot(rotPwr,
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
