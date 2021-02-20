package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class CWMove extends CommandBase{
    
    RobotContainer m_subsystem;
    Vector xy;
    double start;

    public CWMove(RobotContainer subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_driveStrafe);
        addRequirements(m_subsystem.m_driveRot);
        addRequirements(m_subsystem.m_transporterCW);
        xy = Vector.fromXY(0, -1);
        start = m_subsystem.m_drivetrain.drivePos.getTranslation().getY();
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_subsystem.m_drivetrain.drive(xy, 0);
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.m_transporterCW.deployCW(false);
    }

    @Override
    public boolean isFinished(){
        return m_subsystem.m_drivetrain.drivePos.getTranslation().getY() >= start - 3;
    }
}