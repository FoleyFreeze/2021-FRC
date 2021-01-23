package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class DriveAndShoot extends SequentialCommandGroup{//TODO Test me!!!
    public DriveAndShoot(RobotContainer subsystem){
        addCommands(new AutoDrive(subsystem, 0, -24, 0, false));
        addCommands(new AutoShoot(subsystem));
    }
}