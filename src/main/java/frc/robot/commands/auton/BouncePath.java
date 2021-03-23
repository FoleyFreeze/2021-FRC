package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.BrakeMode;
import frc.robot.commands.NewAutoPath;

public class BouncePath extends SequentialCommandGroup {

    public RobotContainer m_subsystem;

    public BouncePath(RobotContainer subsystem){
        m_subsystem = subsystem;
    }

    @Override
    public void initialize(){
        addCommands(
            new NewAutoPath(m_subsystem, "BouncePath1.txt"),
            new BrakeMode(m_subsystem, true).alongWith(new WaitCommand(0.1)),
            new NewAutoPath(m_subsystem, "BouncePath2.txt").alongWith(new BrakeMode(m_subsystem, false)),
            new BrakeMode(m_subsystem, true).alongWith(new WaitCommand(0.1)),
            new NewAutoPath(m_subsystem, "BouncePath3.txt").alongWith(new BrakeMode(m_subsystem, false)),
            new BrakeMode(m_subsystem, true).alongWith(new WaitCommand(0.1)),
            new NewAutoPath(m_subsystem, "BouncePath4.txt").alongWith(new BrakeMode(m_subsystem, false))
        );
        super.initialize();
    }
}

