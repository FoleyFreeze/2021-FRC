package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.cals.CannonCals;
import frc.robot.subsystems.CannonClimber.HoodPos;
import frc.robot.subsystems.Vision.VisionData;
import frc.robot.util.Util;
import frc.robot.util.Vector;

public class AutoShoot extends CommandBase{

    private RobotContainer m_subsystem;
    private CannonCals m_cals;
    public boolean auton;
    public double shootFinTime;
    double prevRobotAngle;
    double prevTime;
    double sumError;
    public boolean masked;
    public double rot;
    public double centX = 0;
    public double centY = 0;
    public boolean shootReady;
    
    public AutoShoot(RobotContainer subsystem, boolean masked){
        m_subsystem = subsystem;
        this.masked = masked;
        m_cals = m_subsystem.m_cannonClimber.shootCals;

        if(!masked){
            addRequirements(m_subsystem.m_driveRot);
            addRequirements(m_subsystem.m_cannonClimber);
            addRequirements(m_subsystem.m_transporterCW);
        }
    }

    public AutoShoot(RobotContainer subsystem){
        this(subsystem, false);
    }

    @Override
    public void initialize(){
        if(m_subsystem.m_input.cam()){
            m_subsystem.m_cannonClimber.setCamLights(true);
            m_subsystem.m_vision.NTEnablePiTgt(true);
        }
        
        auton = DriverStation.getInstance().isAutonomous();
        prevRobotAngle = m_subsystem.m_drivetrain.robotAng;
        prevTime = Timer.getFPGATimestamp();
        sumError = 0;
        prevReady = false;
    }

    boolean prevReady;
    @Override
    public void execute(){
        double time = Timer.getFPGATimestamp();
        double error;
        boolean aligned = false;
        double dist;
        if(/* &&*/ m_subsystem.m_input.cam()){
            double botAngle = m_subsystem.m_drivetrain.robotAng;
            Vector toTarget = new Vector(0,0);
            VisionData image = null;
            boolean hasImage = m_subsystem.m_vision.hasTargetImage();
            if(hasImage){
                image = m_subsystem.m_vision.targetData.getFirst();
                /*
                double robotAngleDiff = Util.angleDiff(botAngle, image.robotangle);
                double rotError = Util.angleDiff(image.angle + m_cals.initJogAng, robotAngleDiff);
                */

                double jog = m_subsystem.m_cannonClimber.shootCals.initJogAng;
                double distH = image.location[0].r / Math.cos(image.location[0].theta);
                toTarget = new Vector(distH, Math.toRadians(image.robotangle - Math.toDegrees(image.location[0].theta) + jog));
                //toTarget = applyLatencyOffset(toTarget, image);
                toTarget = apply3ptProjection(toTarget); //assumes that robot angle is zerod to the target
                //toTarget = applyMovementCompensation(toTarget, m_subsystem.m_drivetrain.recentVelocity);
            } else {
                toTarget = new Vector(0, Math.toRadians(botAngle));
            }

            //error = rotError;
            //dist = image.dist;
            error = Util.angleDiff(Math.toDegrees(toTarget.theta), botAngle);
            dist = toTarget.r;
            
            double deltaAngle = Util.angleDiff(botAngle, prevRobotAngle);

            double dt = (time - prevTime);
            double d = deltaAngle / dt;
            sumError += error * dt;
            //SmartDashboard.putNumber("SumI",sumError);
            if(Math.abs(sumError) > m_cals.maxI){
                sumError = m_cals.maxI * Math.signum(sumError);
            }
            if(Math.abs(error) > m_cals.iZone){
                sumError = 0;
            }
            if(Math.signum(error) != Math.signum(sumError)){
                sumError = 0;
            }
            rot = error * m_cals.kPDrive - d * m_cals.kDDrive + sumError * m_cals.kIDrive;

            if(rot > m_cals.maxRot) rot = m_cals.maxRot;
            else if(rot < -m_cals.maxRot) rot = -m_cals.maxRot;

            centX = m_cals.shootCentX;
            centY = m_cals.shootCentY;

            SmartDashboard.putNumber("AngError", error);
            if(hasImage && Math.abs(error) <= m_cals.tolerance){
                m_subsystem.m_drivetrain.resetFieldPos(image);
            }

        } else if(DriverStation.getInstance().isAutonomous()){
            rot = 0;
            error = 0;
            dist = m_cals.autonDist;
        }else {
            rot = m_subsystem.m_input.getRot();
            error = 0;
            if(m_subsystem.m_input.layup()){
                dist = m_cals.layupDist;
            } else dist = m_cals.trenchDist;
        }
        if(Math.abs(error) <= m_cals.tolerance) aligned = true;//make dependent on dist
        
        m_subsystem.m_drivetrain.driveRot(rot, m_subsystem.m_drivetrain.k.autoBallMaxPwr);

        if(!masked){
            
            boolean ready = m_subsystem.m_cannonClimber.ready();
            m_subsystem.m_cannonClimber.prime(dist);

            /*
            //this is a hack to assist with the power port challenge //FIXME: remove me
            if(dist > 165) {
                dist = 0;
                ready = false;
            }
            */

            //ensure we are within rpm target 2x in a row
            if(prevReady && ready && dist != 0 && aligned && m_subsystem.m_transporterCW.ballnumber > 0){
                m_subsystem.m_transporterCW.shootAll();
                shootFinTime = Timer.getFPGATimestamp() + m_subsystem.m_cannonClimber.shootCals.shootTime;
            } 

            prevReady = ready;
        }else{
            if(m_subsystem.m_transporterCW.ballnumber >= m_subsystem.m_transporterCW.tCals.maxBallCt){
                //Once this starts up, it will not stop priming, which is theoretically fine
                m_subsystem.m_cannonClimber.prime(dist);
                m_subsystem.m_transporterCW.enablefire(true);
            }
        }

        prevRobotAngle = m_subsystem.m_drivetrain.robotAng;
        prevTime = time;
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.m_cannonClimber.stop();
        m_subsystem.m_cannonClimber.setCamLights(false);
        m_subsystem.m_vision.NTEnablePiTgt(false);
        m_subsystem.m_cannonClimber.hTgtPos = HoodPos.LOW;
        m_subsystem.m_transporterCW.enablefire(false);
        m_subsystem.m_transporterCW.stoprot();
    }

    @Override
    public boolean isFinished(){
        //SmartDashboard.putNumber("shootFinTime",shootFinTime - Timer.getFPGATimestamp());
        if(auton) return Timer.getFPGATimestamp() >= shootFinTime && m_subsystem.m_transporterCW.ballnumber == 0;
        return false;
    }

    private Vector apply3ptProjection(Vector v){
        //if we are doing 3 pointers
        if(m_subsystem.m_input.twoVThreePt()){
            //System.out.println(v.toString());
            final double xOffset = 29;//29.25;
            double angTgt = v.theta;
            double hypSin = v.r * Math.sin(angTgt);
            double hypCos = v.r * Math.cos(angTgt) + xOffset; //additional distance between 2pt and 3pt (in)
    
            double dist = Math.sqrt(hypSin*hypSin + hypCos*hypCos);
            double ang = Math.atan2(hypSin,hypCos);

            Vector out = new Vector(dist-xOffset, ang);
            //System.out.println(out.toString());
            return out;
        } else {
            return v;
        }
    }

    private Vector applyMovementCompensation(Vector tgt, Vector vel){
        CannonCals cc = m_subsystem.m_cannonClimber.shootCals;
        if(cc.enableVelOffset){
            double time = Util.interpolate(cc.flightTime, cc.screwDist, tgt.r);
            Vector displacement = new Vector(-vel.r * time, vel.theta);
            Vector out = Vector.add(tgt,displacement);
            SmartDashboard.putNumber("VelOffset",Math.toDegrees(Util.angleDiffRad(out.theta, tgt.theta)));
            
            return out;
        } else{
            return tgt;
        }
    }

    private Vector applyLatencyOffset(Vector tgt, VisionData img){
        CannonCals cc = m_subsystem.m_cannonClimber.shootCals;
        if(cc.enableLatOffset){
            
        }
        return tgt;
    }
    
}