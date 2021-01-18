package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.cals.CWheelCals;
import edu.wpi.first.wpilibj.util.Color;

public class CWColor extends CommandBase{

    RobotContainer m_subsystem;
    CWheelCals m_cals;
    char tgtColor = m_subsystem.m_transporterCW.gameData.charAt(0);
    Color detectedColor = m_subsystem.m_transporterCW.detectedColor;

    public CWColor(RobotContainer subsystem, CWheelCals cals){
        m_subsystem = subsystem;
        m_cals = cals;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_subsystem.m_transporterCW.deployCW(true);
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.m_transporterCW.deployCW(false);
    }

    @Override
    public boolean isFinished(){
        
        switch(tgtColor){
            case 'B':
            return detectedColor == m_cals.Red;//We see red, but it the detector sees blue

            case 'G':
            return detectedColor == m_cals.Yellow;//We see yellow, but it the detector sees green

            case 'R':
            return detectedColor == m_cals.Blue;//We see blue, but it the detector sees red

            case 'Y':
            return detectedColor == m_cals.Green;//We see green, but it the detector sees yellow

            default:
            return false;
        }
    }
}