package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoGather;
import frc.robot.commands.DriveTime;
import frc.robot.commands.JoystickDriveRot;
import frc.robot.subsystems.Vision.VisionData;
import frc.robot.util.Vector;

public class GalacticSearch extends SequentialCommandGroup {

    RobotContainer m_subsystem;
    Vector[] ballVector = new Vector[3];

    public GalacticSearch(RobotContainer subsystem){
        m_subsystem = subsystem;

        addCommands(/*new DriveTime(0.1, subsystem, 1, 0, 0).raceWith(new WaitCommand(0.1)),*/
                new AutoGather(subsystem, ballVector).raceWith(new JoystickDriveRot(subsystem)), 
                new DriveTime(0.3, subsystem, 0, 0.5, 0));
    }

    @Override
    public void initialize(){
        m_subsystem.m_transporterCW.clearBalls();
        super.initialize();
    }

    public void updateBallVector(){
        //save the most recent image with 3 balls
        if(m_subsystem.m_vision.hasBallImage()) {
            Pose2d pos = m_subsystem.m_drivetrain.drivePos;
            if(pos == null) return;
            double angle = m_subsystem.m_drivetrain.robotAng;

            Vector[] tempVectors = new Vector[3];
            boolean allgood = true;
            VisionData initialImage = m_subsystem.m_vision.ballData.getFirst();
            Vector robot = Vector.fromPose(pos);
            for(int i = 0; i < tempVectors.length; i++){
                double r = initialImage.location[i].r/Math.cos(initialImage.location[i].theta);
                double theta = Math.PI - initialImage.location[i].theta + Math.toRadians(angle);
                Vector ball = new Vector(r, theta);
                ball.add(robot);

                //System.out.println("Ball" + i + ": " + ball.toStringXY());
                tempVectors[i] = ball;

                //if some data is bad, dont use it
                if(ball.r < 3) {
                    allgood = false;
                }
            }

            if(allgood){
                for(int i=0; i<ballVector.length; i++){
                    ballVector[i] = tempVectors[i];
                }
            }
        }
    }
}
