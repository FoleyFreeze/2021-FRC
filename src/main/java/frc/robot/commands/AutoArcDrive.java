package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.cals.DriverCals;
import frc.robot.util.Circle;
import frc.robot.util.Util;
import frc.robot.util.Vector;

public class AutoArcDrive extends CommandBase{
    
    RobotContainer m_subsystem;
    ArrayList<Circle> tgtCirc;
    Pose2d botPos;
    int idx = 1;
    DriverCals k;
    boolean stop;
    double maxPwr;

    public AutoArcDrive(RobotContainer subsystem, String path, boolean stop, double maxPwr){
        tgtCirc = Circle.fromFile(path);
        m_subsystem = subsystem;
        addRequirements(m_subsystem.m_driveStrafe);
        botPos = m_subsystem.m_drivetrain.drivePos;
        k = m_subsystem.m_drivetrain.k;
        this.stop = stop;
        this.maxPwr = maxPwr;
    }

    public AutoArcDrive(RobotContainer subsystem, Circle circle, boolean stop, double maxPwr){
        m_subsystem = subsystem;
        tgtCirc = new ArrayList<>();
        tgtCirc.add(circle);
        addRequirements(m_subsystem.m_driveStrafe);
        botPos = m_subsystem.m_drivetrain.drivePos;
        k = m_subsystem.m_drivetrain.k;
        this.stop = stop;
        this.maxPwr = maxPwr;
    }

    double prevSign;
    double circAng;

    @Override
    public void initialize(){
        if(tgtCirc.isEmpty()) return;
        prevSign = 0;
        circAng = Math.PI/2;
    }

    boolean tgtReached = false;

    @Override
    public void execute(){
        Circle tgt = tgtCirc.get(idx);
        botPos = m_subsystem.m_drivetrain.drivePos;
        Vector tan;

        tan = new Vector(1, tgt.tangentAngle(botPos.getX(), botPos.getY(), circAng));
        Vector radial = Vector.fromXY(tgt.getCentX()-botPos.getX(), tgt.getCentY()-botPos.getY());

        //finishes segment when error less than cal and sign change
        double angError = Util.angleDiffRad(radial.theta+Math.PI, tgt.angleofEnd());
        tgtReached = Math.abs(angError) < k.minAngDiffAuto && prevSign != Math.signum(angError);
        prevSign = Math.signum(angError);

        //apply correction for being too far/close from the center
        radial.r -= tgt.radius;
        radial.r*= k.circKp;
        //apply correction depending on how fast we are moving
        radial.r += k.lookAheadCurve*(k.autoDriveMaxPwr/tgt.radius);

        tan.add(radial);

        tan.r = k.autoDriveMaxPwr;
        if(tgtReached){//If the target is close enough
            idx++;
            prevSign = 0;
        } else{//If the target isn't reached or close enough
            circAng = tan.theta;    
            m_subsystem.m_drivetrain.drive(tan, 0);
        }
    }

    @Override
    public void end(boolean interrupted){
        if(stop){
            m_subsystem.m_drivetrain.drive(new Vector(0,0), 0);
        }
    }

    @Override
    public boolean isFinished(){
        return tgtCirc.size() <= idx;
    }
}
