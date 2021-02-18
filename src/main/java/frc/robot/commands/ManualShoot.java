package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CannonClimber.HoodPos;

public class ManualShoot extends CommandBase{

    private RobotContainer m_subsystem;

    public ManualShoot(RobotContainer subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_cannonClimber);
        //not needed anymore as the kicker no longer exists
        //addRequirements(m_subsystem.m_transporterCW);
    }

    @Override
    public void initialize(){
        //m_subsystem.m_transporterCW.enablefire(true);
    }

    @Override
    public void execute(){
        if(m_subsystem.m_input.shift()){
            m_subsystem.m_cannonClimber.setpower(m_subsystem.m_cannonClimber.shootCals.manualPower * -1.0);
        } else{
            //m_subsystem.m_cannonClimber.setpower(m_subsystem.m_cannonClimber.shootCals.manualPower);
            /*m_subsystem.m_cannonClimber.prime(36);
            if(m_subsystem.m_cannonClimber.ready()){
                m_subsystem.m_transporterCW.enablefire(true);
            }*/

            //use jog u/d to set speed
            double speed = m_subsystem.m_cannonClimber.shootCals.initShootSpeed;
            speed += m_subsystem.m_cannonClimber.shootCals.initJogDist * 100;
            m_subsystem.m_cannonClimber.setspeed(speed);
            m_subsystem.m_transporterCW.enablefire(true);
            
            //use jog l/r to set hood height
            double jogAng = m_subsystem.m_cannonClimber.shootCals.initJogAng;
            if(m_subsystem.m_cannonClimber.shootCals.pneumaticHood){
                switch(((int) jogAng) % 4){
                    case 0:
                        m_subsystem.m_cannonClimber.hTgtPos = HoodPos.LOW;
                    break;
                    case 1:
                        m_subsystem.m_cannonClimber.hTgtPos = HoodPos.MID1;
                    break;
                    case 2:
                        m_subsystem.m_cannonClimber.hTgtPos = HoodPos.MID2;
                    break;
                    case 3:
                        m_subsystem.m_cannonClimber.hTgtPos = HoodPos.HIGH;
                    break;
                }
            } else{
                m_subsystem.m_cannonClimber.setScrewHeight(jogAng*0.5);
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.m_cannonClimber.stop();
        m_subsystem.m_transporterCW.enablefire(false);
        m_subsystem.m_cannonClimber.hTgtPos = HoodPos.LOW;
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}