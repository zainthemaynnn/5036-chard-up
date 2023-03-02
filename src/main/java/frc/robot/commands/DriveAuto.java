// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAuto implements Command {
  private static final double P = 0., I = 0., D = 0.;
  private static final Constraints CONSTRAINTS = new Constraints(0., 0.);
  private ProfiledPIDController pidController;
  private Drive drive;

  public DriveAuto(Drive drive, State goal) {
    pidController = new ProfiledPIDController(P, I, D, CONSTRAINTS);
    this.drive = drive;
    pidController.setGoal(goal);
  }

  public DriveAuto(Drive drive, double goal) {
    this(drive, new State(goal, 0.));
  }

  @Override
  public void execute() {
      drive.arcadeDrive(pidController.calculate(drive.centerDist()), 0.);
  }

  @Override
  public boolean isFinished() {
    return pidController.atGoal();
  }

  @Override
  public Set<Subsystem> getRequirements() {
      return Set.of(drive);
  }
}
