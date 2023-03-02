// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.hid.Logitech;
import frc.hid.PS4Controller;
import frc.hid.Logitech.Axis;
import frc.hid.Logitech.Button;
import frc.math.VelocityTuner;
import frc.robot.commands.ArmIn;
import frc.robot.commands.ArmOut;
import frc.robot.commands.BLESSEDAuto;
import frc.robot.commands.DriveArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private Vision cams = new Vision();
  //private PowerDistribution pdp;

  private Logitech
    controller = new Logitech(RobotMap.Gamepad.DRIVER.port()),
    controllerToo = new Logitech(RobotMap.Gamepad.OPERATOR.port());

  private final CANSparkMax
      l1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_LEFT.id(), MotorType.kBrushless),
      l2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_LEFT.id(), MotorType.kBrushless),
      r1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_RIGHT.id(), MotorType.kBrushless),
      r2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_RIGHT.id(), MotorType.kBrushless),
      armController = new CANSparkMax(RobotMap.CAN.ARM.id(), MotorType.kBrushless),
      intakeController = new CANSparkMax(RobotMap.CAN.INTAKE.id(), MotorType.kBrushless);
  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  private Drive drive = new Drive(l1, l2, r1, r2, l1.getEncoder(), r1.getEncoder(), gyro);
  private Arm arm = new Arm(armController);
  private Intake intake = new Intake(intakeController);

  private VelocityTuner tuner = new VelocityTuner(drive::centerDist, drive::centerVel,
      (in) -> drive.arcadeDrive(in, 0));

  public RobotContainer() {
    configureBindings();
  }

  private double dir = 1;
  private enum ArmState {
    UP,
    DOWN,
  }

  private ArmState armState = ArmState.DOWN;

  private DriveArm
    driveArmRest = new DriveArm(arm, 0.),
    driveArmMid = new DriveArm(arm, 0.), // TODO
    driveArmHigh = new DriveArm(arm, 0.); // TODO

  private Command
    armIn = new ArmIn(arm),
    armOut = new ArmOut(arm);

  private Command
    blessedAuto = new BLESSEDAuto(drive);

  private Command
    intake1 = new StartEndCommand(
      () -> intake.drivePower(-.40),
      () -> intake.drivePower(0.),
      intake
    ),
    intake2 = new StartEndCommand(
      () -> intake.drivePower(.40),
      () -> intake.drivePower(0.),
      intake
    ),
    outtake1 = new StartEndCommand(
      () -> intake.drivePower(-1),
      () -> intake.drivePower(0.),
      intake
    ),
    outtake2 = new StartEndCommand(
      () -> intake.drivePower(1),
      () -> intake.drivePower(0.),
      intake
    );

  private void configureBindings() {
    drive.setDefaultCommand(new RunCommand(() -> {
      drive.arcadeDrive(controller.getAxisValue(Axis.LEFT_Y), controller.getAxisValue(Axis.RIGHT_X));
    }, drive));

    /*controller.getButton(Button.YELLOW).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(Units.feetToMeters(1.0*dir));
    }, drive));

    controller.getButton(Button.RED).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(Units.feetToMeters(2.0*dir));
    }, drive));

    controller.getButton(Button.GREEN).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(Units.feetToMeters(3.0*dir));
    }, drive));

    controller.getButton(Button.BLUE).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(Units.feetToMeters(4.0*dir));
    }, drive));*/

    controller.getButton(Logitech.Button.YELLOW).whileTrue(new StartEndCommand(
      () -> arm.drivePower(.25),
      () -> arm.drivePower(0.),
      arm
    ));

    controller.getButton(Logitech.Button.RED).whileTrue(new StartEndCommand(
      () -> arm.drivePower(-.25),
      () -> arm.drivePower(0.),
      arm
    ));

    arm.setDefaultCommand(new RunCommand(() -> arm.drivePower(-.05), arm));

    controller.getButton(Logitech.Button.L1).whileTrue(outtake1);
    controller.getAxis(Logitech.Axis.L2).whileTrue(intake2);
    controller.getButton(Logitech.Button.R1).whileTrue(outtake2);
    controller.getAxis(Logitech.Axis.R2).whileTrue(intake1);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
