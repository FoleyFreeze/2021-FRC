package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class JoystickDrive extends CommandBase{
    private RobotContainer m_subsystem;

    public JoystickDrive(RobotContainer subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_drivetrain);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute(){
        Vector strafe = m_subsystem.m_input.getXY();
        double rot = m_subsystem.m_input.getRot();
        double pausePwr = m_subsystem.m_drivetrain.k.pausePwrPne;
        m_subsystem.m_pneumatics.pauseReq(strafe.r > pausePwr 
                || Math.abs(rot) > pausePwr);
        
        m_subsystem.m_drivetrain.drive(strafe, 
            rot, m_subsystem.m_drivetrain.k.DRV_XROBOTCENT, 
            m_subsystem.m_drivetrain.k.DRV_YROBOTCENT, 
            m_subsystem.m_input.fieldOrient());
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}