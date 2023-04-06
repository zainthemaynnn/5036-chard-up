// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Blinkin.BlinkinColor;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.drive.rampEnabled = false;
    m_robotContainer.drive.encReset();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.drive.rampEnabled = true;
    m_robotContainer.drive.setIdleMode(IdleMode.kCoast);
    m_robotContainer.arm.drivePower(0.);
    m_robotContainer.enableDrive();
    m_robotContainer.leds.addColor(NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAliance").getBoolean(false) ? BlinkinColor.RED_ALLIANCE : BlinkinColor.BLUE_ALLIANCE);
  }

  private boolean endgameEventTriggered = false;

  @Override
  public void teleopPeriodic() {
    if (Timer.getMatchTime() <= 30.0 && endgameEventTriggered == false) {
      m_robotContainer.leds.addColor(NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAliance").getBoolean(false) ? BlinkinColor.RED_ALLIANCE_END : BlinkinColor.BLUE_ALLIANCE_END);
      endgameEventTriggered = true;
      m_robotContainer.leds.display();
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
