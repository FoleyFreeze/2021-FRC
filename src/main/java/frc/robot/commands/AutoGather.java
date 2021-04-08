package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

    public AutoGather(RobotContainer subsystem, Vector[] initBallPos) {
        this(subsystem,false,false);
        this.initBallPos = initBallPos;
    }

    public AutoGather(RobotContainer subsystem, boolean masked, boolean kpOverride){
        m_subsystem = subsystem;
        this.masked = masked;
        if(!masked){
            addRequirements(m_subsystem.m_intake);
            addRequirements(m_subsystem.m_driveStrafe);
        }
        this.kpOverride = kpOverride;
        //this.kpOverride = true;
        this.initBallPos = null;
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
        idx = 0; //for galactic search
        rightBall = false;
        oldImage =null;
    }
    
    private Vector[] initBallPos;

    private double prevBotAngle;
    private Pose2d prevPose;
    Pose2d ballIntersect;
    int idx = 0;
    boolean rightBall;
    VisionData oldImage;

    double[] distThresh = {30,30,30};

    @Override
    public void execute(){
        double rot;
        Pose2d botPos = m_subsystem.m_drivetrain.drivePos;
        double botAngle = m_subsystem.m_drivetrain.robotAng;
        double dXError = botPos.getX()-prevPose.getX();
        double dYError = botPos.getY()-prevPose.getY();
        boolean fieldOrient = false;
        VisionData ballData = null;
        if(m_subsystem.m_vision.ballData.size() != 0) ballData = m_subsystem.m_vision.ballData.getFirst();


        if(auton && initBallPos != null){
            if(idx<3 && m_subsystem.m_vision.hasBallImage()){
                if(idx < m_subsystem.m_transporterCW.ballnumber) idx = m_subsystem.m_transporterCW.ballnumber;

                Vector ballCoords = ballData.location[0].camToFieldVec(botAngle, Vector.fromPose(botPos));
                double dist = Vector.subtract(initBallPos[idx], ballCoords).r;

                //is the ball we see the next ball we should gather
                SmartDashboard.putNumber("GSDist", dist);
                if(Math.abs(dist) <= distThresh[idx]){
                    rightBall = true;
                }else{
                    //if we just lost it, that means we probably gathered it
                    if(ballCoords != null){
                        if(rightBall && oldImage.location[0].r < 30){
                            idx++;
                        }
                    }
                    rightBall = false;
                }

                oldImage = ballData;
                //System.out.println(rightBall + " " + initBallPos[idx] + " " + ballCoords  + " " + oldImage.location[0].r);
            }else{
                //no ball in the current image
                rightBall = false;
            }
        } else {
            rightBall = true;
        }

        if((m_subsystem.m_input.enableBallCam() || auton) && m_subsystem.m_vision.hasBallImage() 
                && rightBall){/*m_subsystem.m_vision.ballData.getFirst().location[0].r <= 48.0)*///robot has control

            fieldOrient = false;

            

            double distError = ballData.location[0].r-m_subsystem.m_drivetrain.k.autoBallGthDist;
            double dDistError = Math.sqrt((dXError*dXError)+(dYError*dYError))/ .020/*m_subsystem.dt*/;//might need to set to .020

            // strafe = Vector.fromXY(-distError * m_subsystem.m_drivetrain.k.autoBallDistKp 
            //                        - dDistError*m_subsystem.m_drivetrain.k.autoBallDistKd, 0);
            
            double robotAngleDiff = Util.angleDiff(botAngle, ballData.robotangle);
            double rotError = Util.angleDiff(Math.toDegrees(ballData.location[0].theta), robotAngleDiff);
            
            double x = ballData.location[0].r * Math.sin(Math.toRadians(rotError));
            double y = ballData.location[0].r * Math.cos(Math.toRadians(rotError));
            
            if(true /*!m_subsystem.m_input.stage2V3()*/){
                double xFactor = Math.max(2.15 - ballData.location[0].r/45.0, 0);
                if(Math.abs(x) < 2){//constant is in inches
                    x = 2 * Math.signum(x);
                }
                y -= Math.abs(xFactor*x);

                double kp = m_subsystem.m_drivetrain.k.autoBallDistKp;
                strafe = Vector.fromXY(-y*kp, x*kp);
            } else{

                double errorX = 0;
                double errorY = 0;
                if(ballIntersect != null){
                    errorX = ballIntersect.getX() - botPos.getX();
                    errorY = ballIntersect.getY() - botPos.getY();
                }

                if(distError > 80){
                    strafe = new Vector(0,0);
                    prevBallPos = null;
                    ballIntersect = null;
                } else if(distError > 40){
                    Vector out = ballPredict(ballData);
                    SmartDashboard.putNumber("BallIntercept", out.r);
                    //System.out.println("Y: " + out.r);
                    ballIntersect = new Pose2d(botPos.getX() + out.getX(), botPos.getY() + out.getY(), new Rotation2d());

                    strafe = new Vector(0,0);
                } else if (distError > 10 && (Math.abs(errorX) > 1 || Math.abs(errorY) > 1)) {
                    if(ballIntersect != null){
                        //PID to it
                        double kp = 0.6 / 12.0;
                        double kd = 0.0;

                        double dt = m_subsystem.m_drivetrain.dt;
                        double dErrorX = (botPos.getX() - prevPose.getX()) / dt;
                        double dErrorY = (botPos.getY() - prevPose.getY()) / dt;

                        double pX = errorX * kp - dErrorX * kd;
                        double pY = errorY * kp - dErrorY * kd;
                        strafe = Vector.fromXY(pX, pY);
                        //strafe = new Vector(0,0);

                        if(!m_subsystem.m_input.fieldOrient()){
                            strafe.theta -= Math.toRadians(m_subsystem.m_drivetrain.robotAng);
                        }

                    } else {
                        strafe = new Vector(0,0);
                    }
                } else {
                    if(prevBallPos != null){
                        //System.out.println(errorX + "," + errorY);
                        prevBallPos = null;
                        ballIntersect = null;
                    }

                    //"normal" ball gathering
                    double xFactor = Math.max(2.15 - ballData.location[0].r/45.0, 0);
                    if(Math.abs(x) < 2){//constant is in inches
                        x = 2 * Math.signum(x);
                    }
                    y -= Math.abs(xFactor*x);

                    double kp = m_subsystem.m_drivetrain.k.autoBallDistKp;
                    strafe = Vector.fromXY(-y*kp, x*kp);
                }
            }

            prevPose=botPos;
            /*boolean fieldOrient = m_subsystem.m_input.fieldOrient();
            if(fieldOrient){
                strafe.theta -= m_subsystem.m_drivetrain.robotAng;
            }*/

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
        }else{
            if(auton && initBallPos != null){
                //GALACTIC SEARCH PATTERN
                /*set strafe to go towards the next ball
                1) get vector to next ball via NextBallPosition - RobotPosition
                2) subtract some y offset to target just before the next ball so that the gatherer can get it
                3) normalize that vector, and use it as the strafe command
                */
                fieldOrient = true;
                if(idx < 3){
                    //index is defined above
                    double x = initBallPos[idx].getX() - botPos.getX();
                    double y = initBallPos[idx].getY() - botPos.getY();
                    //a hack to force the robot to see the next ball
                    //requires the robot be zero'd 90deg to the balls/field
                    Vector v = Vector.fromXY(x+60, y/*-64*/);//Offset for robot width and gatherer
                    //Vector offset = new Vector(64, Math.toRadians(botAngle + 90));//subtract the offset in the direction the robot is facing
                    //v.subtract(offset);
                    SmartDashboard.putString("GS_Vec", String.format("%.1f| %s", Timer.getFPGATimestamp(), v.toStringXY()));
                    v.threshNorm();
                    strafe = v;
                } else {
                    strafe = new Vector(1, Math.PI/2);
                }
                maxPower = m_subsystem.m_drivetrain.k.autoDriveMaxPwr;
                prevBotAngle = m_subsystem.m_drivetrain.robotAng;
                prevPose = botPos;

            } else {//driver has control
                fieldOrient = m_subsystem.m_input.fieldOrient();
                strafe = m_subsystem.m_input.getXY();
                maxPower = 1;
                prevBotAngle = m_subsystem.m_drivetrain.robotAng;
                prevPose = botPos;
            }
        }
        rot = m_subsystem.m_input.getRot();
        m_subsystem.m_drivetrain.driveStrafe(strafe, maxPower, fieldOrient);
        
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
            if(kpOverride) return m_subsystem.m_transporterCW.ballnumber > ballCount; //for other skills challenges
            else return m_subsystem.m_transporterCW.ballnumber >= m_subsystem.m_transporterCW.tCals.maxBallCt-1 || idx == 3; //for galactic search
        }
        return false;
    }

    Vector prevBallPos;
    double dx;
    double dy;
    double m;
    public Vector ballPredict(VisionData image){
        double r = image.location[0].r/Math.cos(image.location[0].theta);
        Vector ballPos = new Vector(r, image.location[0].theta);

        Vector output;

        if(prevBallPos != null){
            Vector movement = Vector.subtract(ballPos, prevBallPos);
            if(Math.abs(movement.r) >= 4) { //if it has moved a signficant distance
                //System.out.println(ballPos.toStringXY());
                dx = dx*m + (1-m)*movement.getX();
                dy = dy*m + (1-m)*movement.getY();
                prevBallPos = ballPos;
                m = 0.5;
            }
            //force robot to overshoot so we always gather w/ leading edge
            double r2/*d2*/ = (ballPos.getX() - m_subsystem.m_drivetrain.k.autoBallGthDist)/(-dx) * dy + ballPos.getY();
            if(Math.signum(r2) != Math.signum(ballPos.getY())){
                r2 += Math.signum(r2) * 0;
            }
            output = new Vector(r2, Math.toRadians(m_subsystem.m_drivetrain.robotAng) + Math.PI/2);
        } else{
            output = new Vector(0, 0);
            m = 0;
            prevBallPos = ballPos;
        }
        
        return output;
    }
}