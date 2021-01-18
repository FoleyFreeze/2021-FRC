package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class DriveOnly extends SequentialCommandGroup{//back up and spin in a circle
    public DriveOnly(RobotContainer subsystem){
        addCommands(new AutoDrive(subsystem, 0, -24, 0, true));//TODO test me!!!
        addCommands(new AutoDrive(subsystem, 0, 0, 90, true));
        addCommands(new AutoDrive(subsystem, 0, 0, 180, true));
        addCommands(new AutoDrive(subsystem, 0, 0, 270, true));
        addCommands(new AutoDrive(subsystem, 0, 0, 0, true));
    }
}