package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.cals.CWheelCals;
import frc.robot.subsystems.TransporterCW;

public class CWStop extends CommandBase{

    RobotContainer m_subsystem;
    TransporterCW transCW;
    CWheelCals m_cals;
    double stopTime;

    public CWStop(RobotContainer subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_transporterCW);
        transCW = m_subsystem.m_transporterCW;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        //transCW.(0.0); //TODO: figure out how to pass this command
        if(transCW.detectedColor != transCW.lastColor) stopTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished(){
        return transCW.detectedColor == transCW.lastColor && 
            Timer.getFPGATimestamp() >= stopTime+m_cals.stopBuffer;
    }
}