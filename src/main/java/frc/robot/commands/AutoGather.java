package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision.VisionData;
import frc.robot.util.Util;
import frc.robot.util.Vector;

public class AutoGather extends CommandBase {

    private RobotContainer m_subsystem;
    private boolean auton;
    public boolean masked;
    public boolean kpOverride;
    public Vector strafe;
    public double maxPower;
    int ballCount;

    public AutoGather(RobotContainer subsystem, boolean masked, boolean kpOverride){
        m_subsystem = subsystem;
        this.masked = masked;
        if(!masked){
            addRequirements(m_subsystem.m_intake);
            addRequirements(m_subsystem.m_driveStrafe);
        }
        //this.kpOverride = kpOverride;
        this.kpOverride = true;
    }

    public AutoGather(RobotContainer subsystem){
        this(subsystem, false, false);
    }

    @Override
    public void initialize(){
        auton = DriverStation.getInstance().isAutonomous();
        m_subsystem.m_vision.NTEnablePiBall(true);
        m_subsystem.m_intake.dropIntake(true);

        prevBotAngle = m_subsystem.m_drivetrain.robotAng;
        prevPose = m_subsystem.m_drivetrain.drivePos;

        ballCount = m_subsystem.m_transporterCW.ballnumber;
        prevBallPos = null; //ensure we forget old ball data
    }

    private double prevBotAngle;
    private Pose2d prevPose;
    Pose2d ballIntersect;

    @Override
    public void execute(){
        double rot;
        Pose2d botPos = m_subsystem.m_drivetrain.drivePos;
        double dXError = botPos.getX()-prevPose.getX();
        double dYError = botPos.getY()-prevPose.getY();
        if(m_subsystem.m_input.enableBallCam() && m_subsystem.m_vision.hasBallImage()){//robot has control

            VisionData ballData = m_subsystem.m_vision.ballData.getFirst();

            double distError = ballData.dist-m_subsystem.m_drivetrain.k.autoBallGthDist;
            double dDistError = Math.sqrt((dXError*dXError)+(dYError*dYError))/ .020/*m_subsystem.dt*/;//might need to set to .020

            // strafe = Vector.fromXY(-distError * m_subsystem.m_drivetrain.k.autoBallDistKp 
            //                        - dDistError*m_subsystem.m_drivetrain.k.autoBallDistKd, 0);
            double botAngle = m_subsystem.m_drivetrain.robotAng;
            double robotAngleDiff = Util.angleDiff(botAngle, ballData.robotangle);
            double rotError = Util.angleDiff(ballData.angle, robotAngleDiff);
            
            double x = ballData.dist * Math.sin(Math.toRadians(rotError));
            double y = ballData.dist * Math.cos(Math.toRadians(rotError));
            
            if(!kpOverride){
                double xFactor = Math.max(2.15 - ballData.dist/45.0, 0);
                if(Math.abs(x) < 2){//constant is in inches
                    x = 2 * Math.signum(x);
                }
                y -= Math.abs(xFactor*x);

                double kp = m_subsystem.m_drivetrain.k.autoBallDistKp;
                strafe = Vector.fromXY(-y*kp, x*kp);
            } else{

                if(distError > 48){
                    Vector out = ballPredict(ballData);
                    SmartDashboard.putNumber("BallIntercept", out.r);
                    ballIntersect = new Pose2d(botPos.getX() + out.getX(), botPos.getY() + out.getY(), new Rotation2d());

                    strafe = new Vector(0,0);
                } else {
                    if(ballIntersect != null){
                        //PID to it
                        double kp = 0.3 / 12.0;
                        double kd = 0.0;

                        double errorX = botPos.getX() - ballIntersect.getX();
                        double errorY = botPos.getY() - ballIntersect.getY();

                        double dt = m_subsystem.m_drivetrain.dt;
                        double dErrorX = (botPos.getX() - prevPose.getX()) / dt;
                        double dErrorY = (botPos.getY() - prevPose.getY()) / dt;

                        double pX = errorX * kp - dErrorX * kd;
                        double pY = errorY * kp - dErrorY * kd;
                        strafe = Vector.fromXY(pX, pY);

                        //treat as field oriented even when not
                        if(!m_subsystem.m_input.fieldOrient()){
                            strafe.theta -= m_subsystem.m_drivetrain.robotAng;
                        }

                    } else {
                        strafe = new Vector(0,0);
                    }
                }
            }

            prevPose=botPos;
            boolean fieldOrient = m_subsystem.m_input.fieldOrient();
            if(fieldOrient){
                strafe.theta -= m_subsystem.m_drivetrain.robotAng;
            }

            //if we are still gathering/indexing the first ball
            //dont drive into the second (but maybe still rotate)
            /*if(m_subsystem.m_transporterCW.isIndexing()){
                strafe.r = 0;
            }*/

            /*
            double botAngle = m_subsystem.m_drivetrain.robotAng;
            double robotAngleDiff = Util.angleDiff(botAngle, ballData.robotangle);
            double rotError = Util.angleDiff(ballData.angle, robotAngleDiff);
            double dRotError = Util.angleDiff(botAngle, prevBotAngle) / 0.020 /*m_subsystem.dt*/;
            /*if(Math.abs(dError) > m_subsystem.m_drivetrain.k.autoBallMaxD){
                dError = m_subsystem.m_drivetrain.k.autoBallMaxD * Math.signum(dError);
            }*/
            /*SmartDashboard.putNumber("Rotspeed",dRotError);
            prevBotAngle = botAngle;
            rot = rotError * m_subsystem.m_drivetrain.k.autoBallAngKp + dRotError * m_subsystem.m_drivetrain.k.autoBallAngKd;*/
            
            maxPower = m_subsystem.m_drivetrain.k.autoBallMaxPwr;
        }else{//driver has control
            strafe = m_subsystem.m_input.getXY();
            maxPower = 1;
            prevBotAngle = m_subsystem.m_drivetrain.robotAng;
            prevPose = botPos;
        }
        rot = m_subsystem.m_input.getRot();
        m_subsystem.m_drivetrain.driveStrafe(strafe, maxPower);
        
        if(!masked && m_subsystem.m_transporterCW.ballnumber >= m_subsystem.m_transporterCW.tCals.maxBallCt && !m_subsystem.m_input.shift()){//limiting balls in tn to maximum
            m_subsystem.m_intake.dropIntake(false);
            m_subsystem.m_intake.setPower(m_subsystem.m_intake.mCals.backwardPower);
        } else if(m_subsystem.m_transporterCW.isIndexing()){//spin intake backwards when spinning tn
            m_subsystem.m_intake.setPower(m_subsystem.m_intake.mCals.idxPower);
        } else {
            m_subsystem.m_intake.setPower(m_subsystem.m_intake.mCals.forwardPower);
        }

        //only rotate the revolver if there is space for it, or if we are shooting
        if(masked || m_subsystem.m_transporterCW.ballnumber < m_subsystem.m_transporterCW.tCals.maxBallCt){
            m_subsystem.m_transporterCW.gatherIndex();
        }
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
            if(kpOverride) return m_subsystem.m_transporterCW.ballnumber > ballCount; //for skills challenge only
            else return m_subsystem.m_transporterCW.ballnumber >= m_subsystem.m_transporterCW.tCals.maxBallCt;
        }
        return false;
    }

    Vector prevBallPos;
    double dx;
    double dy;
    double m;
    public Vector ballPredict(VisionData image){
        double r = image.dist/Math.cos(Math.toRadians(image.angle));
        Vector ballPos = new Vector(r, Math.toRadians(image.angle));
        Vector output;
        if(prevBallPos != null){
            Vector movement = Vector.subtract(ballPos, prevBallPos);
            dx = dx*m + (1-m)*movement.getX();
            dy = dy*m + (1-m)*movement.getY();
            output = new Vector((image.dist - m_subsystem.m_drivetrain.k.autoBallGthDist)/dy * dx, Math.toRadians(m_subsystem.m_drivetrain.robotAng));
            m = 0.5;
        } else{
            output = new Vector(0, 0);
            m = 0;
        }
        prevBallPos = ballPos;
        return output;
    }
}