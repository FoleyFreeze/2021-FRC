package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.cals.CWheelCals;
import frc.robot.subsystems.TransporterCW;

public class CWInit extends CommandBase{
    
    RobotContainer m_subsystem;
    TransporterCW transCW;
    CWheelCals m_cals;

    public CWInit(RobotContainer subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_transporterCW);
        transCW = m_subsystem.m_transporterCW;
        m_cals = transCW.cCals;
    }

    @Override
    public void initialize(){
        transCW.launcher.set(true);
    }

    @Override
    public void execute(){
        //transCW.loadMotor.setSpeed(m_cals.cwInitSpeed);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return (transCW.detectedColor == m_cals.Red || 
            transCW.detectedColor == m_cals.Blue || transCW.detectedColor 
            == m_cals.Green || transCW.detectedColor == m_cals.Yellow);
    }
}