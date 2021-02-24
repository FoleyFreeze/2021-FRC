package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.Circle;
import frc.robot.util.Waypoint;

public class AutonAwardPath extends SequentialCommandGroup{
    public AutonAwardPath(RobotContainer subsystem){
        addCommands(new ParallelCommandGroup(new AutoShoot(subsystem), new AutoArcDrive(subsystem, new Circle(new Waypoint(0, 0, 0), 
            new Waypoint(0, 0, 0)), false, subsystem.m_drivetrain.k.autoDriveMaxPwr)), 
        new AutoDrive(subsystem, 0, 0, 0, true),
        new AutoGather(subsystem, false, true),
        new AutoDrive(subsystem, 0, 0, 0, true));
    }
}
