package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.cals.CannonCals;
import frc.robot.subsystems.CannonClimber.HoodPos;
import frc.robot.subsystems.Vision.VisionData;
import frc.robot.util.Util;

public class AutoShoot extends CommandBase{

    private RobotContainer m_subsystem;
    private CannonCals m_cals;
    public double rotCam = 0.0;
    public boolean auton;
    public double shootFinTime;
    double prevRobotAngle;
    double prevTime;
    public boolean masked;
    public double rot;
    public double centX = 0;
    public double centY = 0;
    
    public AutoShoot(RobotContainer subsystem, boolean masked){
        m_subsystem = subsystem;
        this.masked = masked;
        m_cals = m_subsystem.m_cannonClimber.shootCals;

        if(!masked){
            addRequirements(m_subsystem.m_drivetrain);
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
        prevTime = 0;
    }

    @Override
    public void execute(){
        double time = Timer.getFPGATimestamp();
        double error;
        boolean aligned = false;
        double dist;
        if(m_subsystem.m_vision.hasTargetImage() && m_subsystem.m_input.cam()){
            VisionData image = m_subsystem.m_vision.targetData.getFirst();

            double camAng = image.angle + m_cals.initJogAng;
            double camAngError = Util.angleDiff(m_subsystem.m_drivetrain.robotAng, camAng);
            double angDelta = Util.angleDiff(image.robotangle, m_subsystem.m_drivetrain.robotAng);
            rotCam = Util.angleDiff(camAngError, angDelta);
            error = rotCam;
            dist = image.dist;

            //if we are doing 3 pointers
            if(m_subsystem.m_input.twoVThree()){
                double angTgt = rotCam - m_subsystem.m_drivetrain.robotAng;
                double hypSin = dist * Math.sin(angTgt);
                double hypCos = dist * Math.cos(angTgt) + 29.25;

                dist = Math.sqrt(hypSin*hypSin + hypCos*hypCos);
                error = Math.atan(hypSin/hypCos);
            }

            double robotAngle = m_subsystem.m_drivetrain.robotAng;
            
            double deltaAngle = Util.angleDiff(robotAngle, prevRobotAngle);

            double d = (deltaAngle)/(time - prevTime);
            rot = error * m_cals.kPDrive - d * m_cals.kDDrive;

            if(rot > m_cals.maxRot) rot = m_cals.maxRot;
            else if(rot < -m_cals.maxRot) rot = -m_cals.maxRot;

            centX = m_cals.shootCentX;
            centY = m_cals.shootCentY;
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
        
        if(!masked){
            m_subsystem.m_drivetrain.drive(m_subsystem.m_input.getXY(), rot, centX, centY, 
            m_subsystem.m_input.fieldOrient());
        }
        
        m_subsystem.m_cannonClimber.prime(dist);

        if(m_subsystem.m_cannonClimber.ready() && aligned && m_subsystem.m_transporterCW.ballnumber > 0){
            m_subsystem.m_transporterCW.shootAll();
            m_subsystem.m_transporterCW.enablefire(true);
            shootFinTime = Timer.getFPGATimestamp() + m_subsystem.m_cannonClimber.shootCals.shootTime;
        } 
        else m_subsystem.m_transporterCW.stoprot();

        prevRobotAngle = m_subsystem.m_drivetrain.robotAng;
        prevTime = time;
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.m_cannonClimber.setpower(0);
        m_subsystem.m_cannonClimber.setCamLights(false);
        m_subsystem.m_vision.NTEnablePiTgt(false);
        m_subsystem.m_cannonClimber.hTgtPos = HoodPos.LOW;
        m_subsystem.m_transporterCW.enablefire(false);
    }

    @Override
    public boolean isFinished(){
        if(auton) return Timer.getFPGATimestamp() >= shootFinTime && m_subsystem.m_transporterCW.ballnumber == 0;
        return false;
    }
}