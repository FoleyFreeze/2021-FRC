package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision.VisionData;
import frc.robot.util.Util;
import frc.robot.util.Vector;

public class AutoGather extends CommandBase {

    private RobotContainer m_subsystem;
    private boolean auton;

    public AutoGather(RobotContainer subsystem){
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_intake);
        addRequirements(m_subsystem.m_drivetrain);
    }

    @Override
    public void initialize(){
        auton = DriverStation.getInstance().isAutonomous();
        m_subsystem.m_vision.NTEnablePiBall(true);
        m_subsystem.m_intake.dropIntake(true);
    }

    @Override
    public void execute(){
        double rot;
        Vector strafe;
        if(m_subsystem.m_input.enableBallCam() && m_subsystem.m_vision.hasBallImage()){//robot has control

            VisionData ballData = m_subsystem.m_vision.ballData.getFirst();

            strafe = Vector.fromXY(0, ballData.dist * m_subsystem.m_drivetrain.k.autoBallDistKp);
            boolean fieldOrient = m_subsystem.m_input.fieldOrient();
            if(fieldOrient){
                strafe.theta -= m_subsystem.m_drivetrain.robotAng;
            }

            double robotAngleDiff = Util.angleDiff(m_subsystem.m_drivetrain.robotAng, ballData.robotangle);
            rot = Util.angleDiff(ballData.angle, robotAngleDiff) * m_subsystem.m_drivetrain.k.autoBallAngKp;

        }else{//driver has control
            strafe = m_subsystem.m_input.getXY();
            rot = m_subsystem.m_input.getRot();
        }
        m_subsystem.m_drivetrain.drive(strafe, rot, 0, 0, m_subsystem.m_input.fieldOrient());
        
        if(m_subsystem.m_transporterCW.ballnumber >= 5 && !m_subsystem.m_input.shift()){
            m_subsystem.m_intake.dropIntake(false);
            m_subsystem.m_intake.setPower(m_subsystem.m_intake.mCals.backwardPower);
        } else if(m_subsystem.m_transporterCW.isIndexing()){
            m_subsystem.m_intake.setPower(m_subsystem.m_intake.mCals.idxPower);
        } else {
            m_subsystem.m_intake.setPower(m_subsystem.m_intake.mCals.forwardPower);
        }

        m_subsystem.m_transporterCW.gatherIndex();
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.m_intake.dropIntake(false);
        m_subsystem.m_intake.setPower(0);
        m_subsystem.m_vision.NTEnablePiBall(false);
    }

    @Override
    public boolean isFinished(){
        if(auton){
            return m_subsystem.m_transporterCW.ballnumber >= 5;
        }
        return false;
    }
}