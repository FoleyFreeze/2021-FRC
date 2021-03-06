package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDrive;

public class AutonSquare extends SequentialCommandGroup{

    public AutonSquare(RobotContainer subsystem){
        addCommands(
            new AutoDrive(subsystem, 24, 0, 0, false),
            new AutoDrive(subsystem, 24, 24, 0, false),
            new AutoDrive(subsystem, 0, 24, 0, false),
            new AutoDrive(subsystem, 0, 0, 0, false)
        );
    }
}