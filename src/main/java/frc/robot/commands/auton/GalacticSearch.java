package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
                new DriveTime(6.0, subsystem, -1, 0, 0));
    }

    @Override
    public void initialize(){
        m_subsystem.m_transporterCW.clearBalls();
        super.initialize();
    }

    Vector[][] routes = {{Vector.fromXY(-30,0), Vector.fromXY(-90,30), Vector.fromXY(-120,-60)},  //redA
                            {Vector.fromXY(-30,0), Vector.fromXY(-90,60), Vector.fromXY(-150,0)},    //redB
                            {Vector.fromXY(-150,0), Vector.fromXY(-180,-90), Vector.fromXY(-240,-60)},//blueA
                            {Vector.fromXY(-150,0), Vector.fromXY(-210,-60), Vector.fromXY(-270,0)}}; //blueB
    String[] routenames = {"RedA","RedB","BlueA","BlueB"};    
    boolean hardcodedDists = true;
    
    public void updateBallVector(){
        boolean allgood = false;
        //save the most recent image with 3 balls
        if(m_subsystem.m_vision.hasBallImage()) {
            Pose2d pos = m_subsystem.m_drivetrain.drivePos;
            if(pos == null) return;
            double angle = m_subsystem.m_drivetrain.robotAng;

            Vector[] tempVectors = new Vector[3];
            allgood = true;
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
                if(ball.r < 3 || ball.r > 300) {
                    allgood = false;
                }
            }

            if(allgood){
                for(int i=0; i<ballVector.length; i++){
                    ballVector[i] = tempVectors[i];
                }
                SmartDashboard.putString("GSearchCoords", String.format("%.0f| %s : %s : %s", Timer.getFPGATimestamp(), ballVector[0].toStringXY(), ballVector[1].toStringXY(), ballVector[2].toStringXY()));
            }
        }


        if(hardcodedDists && allgood){
            double minError = 1e9;
            int minIdx = 0;

            //loop through all routes
            for(int idx = 0; idx < routes.length; idx++){
                double sum = 0;
                //sum error of each ball on the route
                for(int i = 0; i<routes[idx].length; i++){
                    sum += Math.abs(Vector.subtract(ballVector[i], routes[idx][i]).r);
                }

                //save the route with the smallest error
                if(sum < minError){
                    minError = sum;
                    minIdx = idx;
                }
            }

            SmartDashboard.putString("GSRoute",String.format("%s %.0f",routenames[minIdx],minError));
            for(int i=0;i<ballVector.length;i++){
                ballVector[i] = routes[minIdx][i];
            }
        }
    }
}
