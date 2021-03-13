package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoArcDrive;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoGather;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.JoystickDriveRot;
import frc.robot.commands.JoystickDriveStrafe;
import frc.robot.util.Circle;
import frc.robot.util.Waypoint;


public class AutonAwardPath extends SequentialCommandGroup{

    public RobotContainer m_subsystem;

    public AutonAwardPath(RobotContainer subsystem){
        m_subsystem = subsystem;
        
        //move and shoot path
        /*
        for(int i = 0; i < 1; i++){
            addCommands(
                new ParallelCommandGroup(new AutoShoot(subsystem), new AutoArcDrive(subsystem, new Circle(new Waypoint(150, 185, 0), 
                    new Waypoint(90, 195, 0)), false, subsystem.m_drivetrain.k.autoDriveMaxPwr)), 
                new AutoDrive(subsystem, 150, 330, 0, true),
                new AutoGather(subsystem, false, true),
                new AutoDrive(subsystem, 30, 185, 0, true)
            );
        }
        */

        //stationary shoot path
        Waypoint circleCenter = new Waypoint(90,75,0);
        Waypoint shootPos2 = new Waypoint(70,121,0);
        Waypoint shootPos3 = new Waypoint(70,121,0);
        addCommands(
            //gather 3 balls first
            new AutoGather(subsystem, false, false).raceWith(new JoystickDriveRot(subsystem)),
            new AutoGather(subsystem, false, false).raceWith(new JoystickDriveRot(subsystem)),
            new AutoGather(subsystem, false, false).raceWith(new JoystickDriveRot(subsystem)),
            
            new AutoDrive(subsystem, 90, 125, 0, false), //ensure robot somewhat faces goal
            new AutoShoot(subsystem).raceWith(new JoystickDriveStrafe(subsystem)), 
            new AutoArcDrive(subsystem, new Circle(new Waypoint(140, 75, 0), circleCenter), false, subsystem.m_drivetrain.k.autoDriveMaxPwr), 
            new AutoDrive(subsystem, 130, -10, 12, false),
            new AutoGather(subsystem, false, true).raceWith(new JoystickDriveRot(subsystem)),
            new AutoDrive(subsystem, 130, -10, 18, false),
            new AutoGather(subsystem, false, true).raceWith(new JoystickDriveRot(subsystem)),
            new AutoDrive(subsystem, 130, -10, 18, false),
            new AutoGather(subsystem, false, true).raceWith(new JoystickDriveRot(subsystem)),
            //new AutoDrive(subsystem, 40, 75, 0, false)
            new AutoDrive(subsystem, 30, 120, 0, false) //shoot from left location
        );
        addCommands(
            //new AutoArcDrive(subsystem, new Circle(shootPos2, circleCenter), false, subsystem.m_drivetrain.k.autoDriveMaxPwr),    
            new AutoShoot(subsystem).raceWith(new JoystickDriveStrafe(subsystem)), 
            new AutoArcDrive(subsystem, new Circle(new Waypoint(140, 75, 0), circleCenter), false, subsystem.m_drivetrain.k.autoDriveMaxPwr), 
            new AutoDrive(subsystem, 130, -10, 12, false),
            new AutoGather(subsystem, false, true).raceWith(new JoystickDriveRot(subsystem)),
            new AutoDrive(subsystem, 130, -10, 18, false),
            new AutoGather(subsystem, false, true).raceWith(new JoystickDriveRot(subsystem)),
            new AutoDrive(subsystem, 130, -10, 18, false),
            new AutoGather(subsystem, false, true).raceWith(new JoystickDriveRot(subsystem)),
            //new AutoDrive(subsystem, 40, 75, 0, false)
            new AutoDrive(subsystem, 110, 120, 0, false) //shoot from right location
        );
        addCommands(
            //new AutoArcDrive(subsystem, new Circle(shootPos3, circleCenter), false, subsystem.m_drivetrain.k.autoDriveMaxPwr),     
            new AutoShoot(subsystem).raceWith(new JoystickDriveStrafe(subsystem)), 
            new AutoDrive(subsystem, circleCenter.x, 5, -135, false)
        );
        
        
    }

    @Override
    public void initialize(){
        super.initialize();
        m_subsystem.m_drivetrain.setStartPosition(90, 125);
    }

    @Override
    public void execute(){
        super.execute();
    }
}
