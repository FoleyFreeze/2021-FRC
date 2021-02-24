package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class AutoBall extends SequentialCommandGroup{
    public RobotContainer m_subsystem;

    public AutoBall(RobotContainer subsystem){
        m_subsystem = subsystem;
        addCommands(new AutoGather(m_subsystem)/*, do a thing*/);
    }
}
