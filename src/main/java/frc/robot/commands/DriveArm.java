// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveArm implements Command {
  private Arm arm;
  private ProfiledPIDController pidController =
    new ProfiledPIDController(
      0., 0, 0.,
      new TrapezoidProfile.Constraints(0., 0.)
    ); // TODO
  private final ArmFeedforward armFeedforward =
    new ArmFeedforward(0., 0., 0.); // TODO

  public DriveArm(Arm arm, State goal) {
    this.arm = arm;
    pidController.setGoal(goal);
  }

  public DriveArm(Arm arm, double goal) {
    this(arm, new State(goal, 0.));
  }

  @Override
  public void execute() {
      var goal = pidController.getGoal();
      arm.driveVolts(
        pidController.calculate(arm.position()) +
          armFeedforward.calculate(goal.position, goal.velocity)
      );
  }

  @Override
  public boolean isFinished() {
    return pidController.atGoal();
  }

  @Override
  public Set<Subsystem> getRequirements() {
      return Set.of(arm);
  }
}
