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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.cals.CalSet;
import frc.robot.commands.AutoPath;
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
    CommandScheduler.getInstance().run();
    double time = Timer.getFPGATimestamp();
    double dt = time - lastTime;
    Display.put("DT", dt);
    lastTime = time;
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
