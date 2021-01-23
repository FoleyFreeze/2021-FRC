package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoSC_Test extends SequentialCommandGroup{

    RobotContainer subsystem;

    public AutoSC_Test(RobotContainer subsystem){
        this.subsystem = subsystem;
        addCommands(
            new AutoDrive(subsystem, 180-86, 176, 0, false),
            new AutoDrive(subsystem, 180-32, 176, 0, false),
            new AutoDrive(subsystem, 180-32, 120, 0, false),
            new AutoDrive(subsystem, 180-120, 220, 0, false),
            new AutoDrive(subsystem, 180-150, 180, 0, false),
            new AutoDrive(subsystem, 180-120, 140, 0, false),
            new AutoDrive(subsystem, 180-25, 300, 0, false),
            new AutoDrive(subsystem, 180-86, 324, 0, false),
            new AutoDrive(subsystem, 180-86, 42, 0, false)
        );
    }

    @Override
    public void initialize(){
        subsystem.m_drivetrain.setStartPosition(180-86, 42);
        super.initialize();
    }

    @Override
    public void end(boolean interrupted){
        subsystem.m_drivetrain.setBrake(true);
    }
}