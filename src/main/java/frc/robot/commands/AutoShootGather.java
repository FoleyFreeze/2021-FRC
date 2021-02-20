package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoShootGather extends CommandBase{

    RobotContainer m_subsystem;
    AutoShoot autoShoot;
    AutoGather autoGather;

    public AutoShootGather(RobotContainer subsystem){
        m_subsystem = subsystem;
        autoShoot = new AutoShoot(subsystem, true);
        autoGather = new AutoGather(subsystem, true);
        addRequirements(m_subsystem.m_driveStrafe);
        addRequirements(m_subsystem.m_driveRot);
        addRequirements(m_subsystem.m_cannonClimber);
        addRequirements(m_subsystem.m_transporterCW);
        addRequirements(m_subsystem.m_intake);
    }

    @Override
    public void initialize(){
        autoShoot.initialize();
        autoGather.initialize();
    }

    @Override
    public void execute(){
        autoShoot.execute();
        autoGather.execute();
    }

    @Override
    public void end(boolean interrupted){
        autoShoot.end(interrupted);
        autoGather.end(interrupted);
    }
}
