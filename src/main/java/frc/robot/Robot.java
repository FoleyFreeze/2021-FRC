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
import frc.robot.commands.AutoArcDrive;
import frc.robot.commands.AutoPath;
import frc.robot.commands.auton.AutonAwardPath;
import frc.robot.commands.NewAutoPath;
import frc.robot.subsystems.Display;
import frc.robot.util.Circle;
import frc.robot.util.Waypoint;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_subsystem;

  @Override
  public void robotInit() {
    Display.init();
    CalSet.identifyBot();
    SmartDashboard.putNumber("DistError",0);
    m_subsystem = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    double time = Timer.getFPGATimestamp();
    m_subsystem.dt = time - lastTime;
    Display.put("DT", m_subsystem.dt);
    lastTime = time;
    
    CommandScheduler.getInstance().run();

    Display.put("Selected Auton" , m_subsystem.autonChooser.getSelected().getName());
  }

  double lastTime = 0;

  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {
    m_subsystem.galacticSearch.updateBallVector();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_subsystem.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule(false);
    }

    m_subsystem.m_transporterCW.autonInit();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if(m_subsystem.m_drivetrain.getBrake()){
      m_subsystem.m_drivetrain.setBrake(false);
    }

    //sets cals based on connected controller (flysky vs xbox) 
    m_subsystem.m_input.onInit();
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
    NewAutoPath nap = new NewAutoPath(m_robotContainer, "CircleTest.txt");
    nap.initialize();

    nap.botPos = new Pose2d(0, 0, new Rotation2d());
    //nap.robotAngle = 0;
    nap.execute();

    nap.botPos = new Pose2d(30, 60+60*Math.sqrt(Math.PI)/2, new Rotation2d());
    nap.execute();

    //should finish first index
    nap.botPos = new Pose2d(60, 120, new Rotation2d());
    nap.execute();

    nap.botPos = new Pose2d(120, 60, new Rotation2d());
    //nap.robotAngle = 90;
    nap.execute();

    nap.botPos = new Pose2d(0, 0, new Rotation2d());
    //nap.robotAngle = 240;
    nap.execute();

    nap.botPos = new Pose2d(20, 41, new Rotation2d());
    nap.execute();

    nap.botPos = new Pose2d(0.5, 0.5, new Rotation2d());
    nap.execute();

    nap.botPos = new Pose2d(-1, -1, new Rotation2d());
    nap.execute();

    System.out.println(nap.isFinished());
    System.exit(0);*/
    
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

    /*AutoArcDrive aap = new AutoArcDrive(m_subsystem, new Circle(new Waypoint(150, 185, 0), new Waypoint(90, 185, 0)), false, 1);

    aap.initialize();

    m_subsystem.m_drivetrain.drivePos = new Pose2d(90, 125, new Rotation2d());
    aap.execute();

    m_subsystem.m_drivetrain.drivePos = new Pose2d(150-Math.sqrt(2)/2*60, 185+Math.sqrt(2)/2*60, new Rotation2d());
    aap.execute();

    m_subsystem.m_drivetrain.drivePos = new Pose2d(150, 185, new Rotation2d());
    aap.execute();
    */
  }

}
