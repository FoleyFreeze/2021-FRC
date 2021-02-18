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
            double speed = m_subsystem.m_cannonClimber.shootCals.initShootSpeed;
            speed += m_subsystem.m_cannonClimber.shootCals.initJogDist * 100;
            m_subsystem.m_cannonClimber.setspeed(speed);
            m_subsystem.m_transporterCW.enablefire(true);
            int jogAng = (int) m_subsystem.m_cannonClimber.shootCals.initJogAng;
            if(jogAng == 0){
                m_subsystem.m_cannonClimber.hTgtPos = HoodPos.LOW;
            } else if(jogAng == 1){
                m_subsystem.m_cannonClimber.hTgtPos = HoodPos.MID1;
            } else if(jogAng == 2){
                m_subsystem.m_cannonClimber.hTgtPos = HoodPos.MID2;
            } else if(jogAng == 3){
                m_subsystem.m_cannonClimber.hTgtPos = HoodPos.HIGH;
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