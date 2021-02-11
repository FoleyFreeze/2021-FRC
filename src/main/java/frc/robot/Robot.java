/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.cals.CalSet;
import frc.robot.commands.AutoPath;
import frc.robot.commands.NewAutoPath;
import frc.robot.subsystems.Display;
import frc.robot.util.Waypoint;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    Display.init();
    CalSet.identifyBot();
    
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    double time = Timer.getFPGATimestamp();
    m_robotContainer.dt = time - lastTime;
    Display.put("DT", m_robotContainer.dt);
    lastTime = time;
    
    CommandScheduler.getInstance().run();
  }

  double lastTime = 0;

  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule(false);
    }

    m_robotContainer.m_transporterCW.autonInit();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if(m_robotContainer.m_drivetrain.getBrake()){
      m_robotContainer.m_drivetrain.setBrake(false);
    }

    //sets cals based on connected controller (flysky vs xbox) 
    m_robotContainer.m_input.onInit();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit(){

    /*
    NewAutoPath nap = new NewAutoPath(m_robotContainer, "path1.txt");
    nap.initialize();

    nap.botPos = new Pose2d(0, 0, new Rotation2d());
    nap.robotAngle = 0;
    nap.execute();

    nap.botPos = new Pose2d(47, 0, new Rotation2d());
    nap.execute();

    //should finish first index
    nap.botPos = new Pose2d(50, 0, new Rotation2d());
    nap.execute();

    nap.botPos = new Pose2d(84, 30, new Rotation2d());
    nap.robotAngle = 90;
    nap.execute();

    nap.botPos = new Pose2d(22, 44, new Rotation2d());
    nap.robotAngle = 240;
    nap.execute();

    nap.botPos = new Pose2d(20, 41, new Rotation2d());
    nap.execute();

    nap.botPos = new Pose2d(0.5, 0.5, new Rotation2d());
    nap.execute();

    nap.botPos = new Pose2d(-1, -1, new Rotation2d());
    nap.execute();

    System.out.println(nap.isFinished());
    System.exit(0);
    */
    
    //testing for the auto path spline follower 
    /*Waypoint[] path = Waypoint.fromFile("path1.txt");
    AutoPath pathTest = new AutoPath(m_robotContainer, path, 36);
    pathTest.useFake = true;
    
    pathTest.fakeX = path[0].x;
    pathTest.fakeY = path[0].y;
    pathTest.fakeTheta = path[0].theta;
    pathTest.initialize();

    pathTest.execute();
    System.out.println(pathTest.tgtPt);

    pathTest.fakeX = 94;
    pathTest.fakeY = 139;
    pathTest.fakeTheta = path[0].theta;
    pathTest.execute();
    System.out.println(pathTest.tgtPt);

    pathTest.fakeX = 94;
    pathTest.fakeY = 141;
    pathTest.fakeTheta = path[0].theta;
    pathTest.execute();
    System.out.println(pathTest.tgtPt);

    pathTest.fakeX = 100;
    pathTest.fakeY = 165;
    pathTest.fakeTheta = path[0].theta;
    pathTest.execute();
    System.out.println(pathTest.tgtPt);

    pathTest.fakeX = 125;
    pathTest.fakeY = 170;
    pathTest.fakeTheta = path[0].theta;
    pathTest.execute();
    System.out.println(pathTest.tgtPt);

    pathTest.fakeX = 140;
    pathTest.fakeY = 168;
    pathTest.fakeTheta = path[0].theta;
    pathTest.execute();
    System.out.println(pathTest.tgtPt);

    System.exit(0);
    */
  }

}
