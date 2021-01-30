package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        prevBotAngle = m_subsystem.m_drivetrain.robotAng;
        prevPose = m_subsystem.m_drivetrain.drivePos;
    }

    private double prevBotAngle;
    private Pose2d prevPose;

    @Override
    public void execute(){
        double rot,maxPower;
        Vector strafe;
        Pose2d botPos = m_subsystem.m_drivetrain.drivePos;
        double dXError = botPos.getX()-prevPose.getX();
        double dYError = botPos.getY()-prevPose.getY();
        if(/*m_subsystem.m_input.enableBallCam() &&*/ m_subsystem.m_vision.hasBallImage()){//robot has control

            VisionData ballData = m_subsystem.m_vision.ballData.getFirst();

            double distError = ballData.dist-m_subsystem.m_drivetrain.k.autoBallGthDist;
            double dDistError = Math.sqrt((dXError*dXError)+(dYError*dYError))/ .020/*m_subsystem.dt*/;//might need to set to .020

            strafe = Vector.fromXY(-distError * m_subsystem.m_drivetrain.k.autoBallDistKp 
                                     - dDistError*m_subsystem.m_drivetrain.k.autoBallDistKd, 0);
            prevPose=botPos;
            boolean fieldOrient = m_subsystem.m_input.fieldOrient();
            if(fieldOrient){
                strafe.theta -= m_subsystem.m_drivetrain.robotAng;
            }

            double botAngle = m_subsystem.m_drivetrain.robotAng;
            double robotAngleDiff = Util.angleDiff(botAngle, ballData.robotangle);
            double rotError = Util.angleDiff(ballData.angle, robotAngleDiff);
            double dRotError = Util.angleDiff(botAngle, prevBotAngle) / 0.020 /*m_subsystem.dt*/;
            /*if(Math.abs(dError) > m_subsystem.m_drivetrain.k.autoBallMaxD){
                dError = m_subsystem.m_drivetrain.k.autoBallMaxD * Math.signum(dError);
            }*/
            SmartDashboard.putNumber("Rotspeed",dRotError);
            prevBotAngle = botAngle;
            rot = rotError * m_subsystem.m_drivetrain.k.autoBallAngKp + dRotError * m_subsystem.m_drivetrain.k.autoBallAngKd;
            

            maxPower = m_subsystem.m_drivetrain.k.autoBallMaxPwr;
        }else{//driver has control
            strafe = m_subsystem.m_input.getXY();
            rot = m_subsystem.m_input.getRot();
            maxPower = 1;
            prevBotAngle = m_subsystem.m_drivetrain.robotAng;
            prevPose = botPos;
        }
        m_subsystem.m_drivetrain.drive(strafe, rot, 0, 0, m_subsystem.m_input.fieldOrient(),maxPower);
        
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