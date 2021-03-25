package frc.robot.commands;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.cals.DriverCals;
import frc.robot.util.Circle;
import frc.robot.util.Util;
import frc.robot.util.Vector;

public class NewAutoPath extends CommandBase{

    private int idx;
    public Pose2d botPos;
    public double prevDriveAngle;
    ArrayList<Circle> path;
    double dist;
    double prevDist;
    double prevSign;
    boolean tgtReached;
    DriverCals k;
    
    private RobotContainer m_subsystem;   
    
    public NewAutoPath(RobotContainer subsystem, String fileName){
        m_subsystem = subsystem;
        addRequirements(subsystem.m_driveStrafe);
        addRequirements(subsystem.m_driveRot);
        path = Circle.fromFile(fileName);
        System.out.println(fileName + " length = " + path.size());
        k = m_subsystem.m_drivetrain.k;
    }   

    @Override
    public void initialize(){
        if(path == null) return;
        idx = 1;
        m_subsystem.m_drivetrain.setStartPosition(path.get(0).getX(), path.get(0).getY());

        prevDist = 9999;
        prevSign = 0;
        prevDriveAngle = Math.PI/2;
    }

    @Override
    public void execute(){
        Circle tgt = path.get(idx);
        botPos = m_subsystem.m_drivetrain.drivePos;
        Vector tan;

        if(tgt.radius > 0){//circle
            tan = new Vector(1, tgt.tangentAngle(botPos.getX(), botPos.getY(), path.get(idx-1).end));
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
            
        } else{//line
            double dX = botPos.getX()-tgt.getX();
            double dY = botPos.getY()-tgt.getY();
            dist = Math.sqrt(dX*dX+dY*dY);
            //finishes segment when error less than cal and sign change
            tgtReached = dist < k.minDistAutoCirc && dist > prevDist;
            prevDist = dist;

            tan = Vector.fromXY(tgt.getX() - botPos.getX(), tgt.getY() - botPos.getY());
        }

        tan.r = k.autoDriveMaxPwr;

        System.out.println("Idx: " + idx);
        System.out.println(tan.toString());

        if(tgtReached){//If the target is close enough
            idx++;
            prevDist = 9999;
            prevSign = 0;
        } else{//If the target isn't reached or close enough
            prevDriveAngle = tan.theta;    
            m_subsystem.m_drivetrain.drive(tan, 0, true);//force field oriented
        }
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.m_drivetrain.drive(new Vector(0, 0), 0, 0, 0, true);
        m_subsystem.m_drivetrain.setBrake(true);
    }

    @Override
    public boolean isFinished(){
        return path.size() <= idx;
    }
}
