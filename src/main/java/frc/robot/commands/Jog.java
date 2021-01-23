package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Jog extends CommandBase{

    RobotContainer mSubsystem;
    boolean jogUpDnvsLR;
    boolean jogPlusMinus;

    public Jog(RobotContainer subsystem, boolean jogUpDnvsLR, boolean jogPlusMinus){
        mSubsystem = subsystem;
        this.jogUpDnvsLR = jogUpDnvsLR;
        this.jogPlusMinus = jogPlusMinus;
    }

    @Override
    public void initialize(){
        if(jogUpDnvsLR){
            mSubsystem.m_cannonClimber.jogUpDn(jogPlusMinus);
        } else {
            mSubsystem.m_cannonClimber.jogLR(jogPlusMinus);
        }
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}