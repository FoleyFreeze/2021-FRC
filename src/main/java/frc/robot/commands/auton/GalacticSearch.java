package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoGather;

public class GalacticSearch extends SequentialCommandGroup {

    RobotContainer m_subsystem;

    public GalacticSearch(RobotContainer subsystem){
        m_subsystem = subsystem;

        addCommands(new AutoDrive(subsystem, 0, -30, 0, true).raceWith(new WaitCommand(0.1)), 
                new AutoGather(subsystem, false, false), 
                new AutoDrive(subsystem, 0, 150, 0, true));
    }
}
