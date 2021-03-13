package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.CWInit;
import frc.robot.commands.CWMove;
import frc.robot.commands.CWSpin;
import frc.robot.commands.CWStop;

public class CWCombo extends SequentialCommandGroup{
    
    RobotContainer m_subsystem;

    public CWCombo(RobotContainer subsystem){
        m_subsystem = subsystem;

        addCommands(new CWInit(m_subsystem));
        addCommands(new CWSpin(m_subsystem));
        addCommands(new CWStop(m_subsystem));
        addCommands(new CWMove(m_subsystem));
    }
}