/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.cals.CWheelCals;
import frc.robot.cals.CannonCals;
import frc.robot.cals.ClimberCals;
import frc.robot.cals.DriverCals;
import frc.robot.cals.ElectroKendro;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.JoystickDriveRot;
import frc.robot.commands.JoystickDriveStrafe;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualRevolve;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.NewAutoPath;
import frc.robot.commands.AutoTrench;
import frc.robot.commands.AutonAwardPath;
import frc.robot.subsystems.*;
import frc.robot.util.Waypoint;
import frc.robot.cals.IntakeCals;
import frc.robot.cals.PneumaticsCals;
import frc.robot.cals.TransporterCals;
import frc.robot.cals.VisionCals;
import frc.robot.commands.ZeroReset;
import frc.robot.commands.AutoTrench.Orientation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoGather;
import frc.robot.commands.AutoPath;
import frc.robot.commands.AutoSC_Test;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootGather;
import frc.robot.commands.AutonSquare;
import frc.robot.commands.CWCombo;
import frc.robot.commands.Climb;
import frc.robot.commands.CountCmd;
import frc.robot.commands.DriveTime;
import frc.robot.commands.Jog;


public class RobotContainer {
  public PneumaticsCals pCals = new PneumaticsCals();

  public final Drivetrain m_drivetrain = new Drivetrain(new DriverCals(), this);
  public final Intake m_intake = new Intake(new IntakeCals());
  public final Inputs m_input = new Inputs(new ElectroKendro());
  public final CannonClimber m_cannonClimber = new CannonClimber(this, new CannonCals(), new ClimberCals());
  public final Display m_display = new Display();
  public final Pneumatics m_pneumatics = new Pneumatics(pCals);
  public final TransporterCW m_transporterCW = new TransporterCW(new TransporterCals(pCals), new CWheelCals(), this);
  public final Vision m_vision = new Vision(new VisionCals(), this);
  public final DriveRot m_driveRot = new DriveRot();
  public final DriveStrafe m_driveStrafe = new DriveStrafe();

  public SendableChooser<CommandBase> autonChooser;

  public double dt;
  
  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(m_drivetrain, m_intake, 
    m_cannonClimber, m_pneumatics, m_transporterCW, m_driveRot, m_driveStrafe);
    
    //m_drivetrain.setDefaultCommand(new JoystickDrive(this));
    m_driveRot.setDefaultCommand(new JoystickDriveRot(this));
    m_driveStrafe.setDefaultCommand(new JoystickDriveStrafe(this));

    configureButtonBindings();

    autonChooser = new SendableChooser<>();
    autonChooser.addOption("DriveOnly", new AutoDrive(this, 0, 12, 0, true));
    //autonChooser.setDefaultOption("DriveAndShoot", new SequentialCommandGroup(new AutoShoot(this),new AutoDrive(this,0,-48,90,true)));
    autonChooser.setDefaultOption("DriveAndShoot", new SequentialCommandGroup(new AutoShoot(this),new DriveTime(3, this, 0, -0.4, 0)));
    
    autonChooser.addOption("AutoSquare", new AutonSquare(this));
    autonChooser.addOption("AutoPath", new NewAutoPath(this, "CircleTest.txt"));
    autonChooser.addOption("AutonAward", new AutonAwardPath(this));
    SmartDashboard.putData(autonChooser);
  }

  
  private void configureButtonBindings() {
    m_input.shoot.and(m_input.layupTrigger.negate()).and(m_input.gather.negate()).whileActiveOnce(new AutoShoot(this));
    m_input.shoot.and(m_input.layupTrigger).and(m_input.gather.negate()).whileActiveOnce(/*new SequentialCommandGroup(new AutoDrive(this, 0, -24, 0, true),*/ new AutoShoot(this));
    m_input.angleReset.whileActiveOnce(new ZeroReset(this));
    m_input.angleReset.whileActiveOnce(new CountCmd(this));
    m_input.climbUp.and(m_input.shift.negate()).whileActiveOnce(new Climb(this, ClimberCals.upPower));
    m_input.climbDn.and(m_input.shift.negate()).whileActiveOnce(new Climb(this, ClimberCals.dnPower));
    m_input.climbUp.and(m_input.shift).whileActiveOnce(new Climb(this, ClimberCals.shiftUpClimb));
    m_input.climbDn.and(m_input.shift).whileActiveOnce(new Climb(this, ClimberCals.shiftDnClimb));
    m_input.manualIntake.whileActiveOnce(new ManualIntake(this));
    m_input.manualShoot.whileActiveOnce(new ManualShoot(this));
    m_input.revolve.whileActiveOnce(new ManualRevolve(this));
    m_input.jogUp.whileActiveOnce(new Jog(this, true, true));
    m_input.jogDn.whileActiveOnce(new Jog(this, true, false));
    m_input.jogL.whileActiveOnce(new Jog(this, false, true));
    m_input.jogR.whileActiveOnce(new Jog(this, false, false));
    m_input.autoTrench.whileActiveOnce(new AutoTrench(this, Orientation.AUTO));
    m_input.gather.and(m_input.shoot.negate()).whileActiveOnce(new AutoGather(this));
    m_input.shoot.and(m_input.gather).whileActiveOnce(new AutoShootGather(this));
    //m_input.cwActivate.whileActiveOnce(new CWCombo(this));
    //TODO: enable when the CW works
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
