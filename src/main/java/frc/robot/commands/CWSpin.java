package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.cals.CWheelCals;
import frc.robot.subsystems.TransporterCW;

public class CWSpin extends CommandBase{

    private RobotContainer m_subsystem;
    private TransporterCW transCW;
    private CWheelCals m_cals;
    private int wedgeCount;
    private char gameData;

    public CWSpin(RobotContainer subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_transporterCW);
        transCW = m_subsystem.m_transporterCW;
        m_cals = transCW.cCals;
        wedgeCount = 0;
        gameData = DriverStation.getInstance().getGameSpecificMessage().charAt(0);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(m_subsystem.m_input.stage2V3()){
            //transCW.loadMotor.setSpeed(m_cals.rotSpeed);
            if(transCW.detectedColor != transCW.lastColor) wedgeCount++;
        }else{
            //transCW.loadMotor.setSpeed(m_cals.colSpeed);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        if(m_subsystem.m_input.stage2V3()){
            return wedgeCount >= 24;
        }else{
            switch(gameData){       //TODO fix me!!!
                case 'R':
                return transCW.detectedColor == transCW.cCals.Red;

                case 'G':
                return transCW.detectedColor == transCW.cCals.Green;

                case 'B':
                return transCW.detectedColor == transCW.cCals.Blue;

                case 'Y':
                return transCW.detectedColor == transCW.cCals.Yellow;

                default:
                return false;
            }
        }
    }
}