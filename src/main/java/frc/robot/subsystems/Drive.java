// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.ui.Dashboard;

public class Drive implements Subsystem {
  private static final double TRACK_WIDTH = 24.0;
  private static final double WHEEL_RADIUS = Units.inchesToMeters(3.0);
  private static final double GEAR_RATIO = 8.45;
  private static final double k = Units.inchesToMeters((double) 2*Math.PI*WHEEL_RADIUS / GEAR_RATIO);

  private CANSparkMax l1, l2, r1, r2;
  private RelativeEncoder encL, encR;
  private AHRS gyro;

  private DifferentialDriveKinematics kinematics;
  private ChassisSpeeds chassisSpeeds;
  private DifferentialDrivePoseEstimator estimator;
  private Pose2d pose;

  private Dashboard dashboard = new Dashboard("Drive");

  public Drive(
    CANSparkMax l1,
    CANSparkMax l2,
    CANSparkMax r1,
    CANSparkMax r2,
    RelativeEncoder encL,
    RelativeEncoder encR,
    AHRS gyro
  ) {
    this.pose = new Pose2d();
    this.kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(TRACK_WIDTH));
    //this.estimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), 0, 0, pose);

    this.l1 = l1;
    this.l2 = l2;
    this.r1 = r1;
    this.r2 = r2;
    this.encL = encL;
    this.encR = encR;
    this.gyro = gyro;

    encL.setPosition(0.0);
    encR.setPosition(0.0);
    encL.setPositionConversionFactor(k);
    encR.setPositionConversionFactor(k);
    encL.setVelocityConversionFactor(k);
    encR.setVelocityConversionFactor(k);

    l2.follow(l1);
    r2.follow(r1);

    dashboard
      .add(gyro, "gyro");
  }

  public void arcadeDrive(double throttle, double wheel) {
    l1.set(throttle + wheel);
    r1.set(throttle - wheel);
  }

  public double centerDist() {
    return (encL.getPosition() + encR.getPosition()) / 2;
  }

  public double centerVel() {
    return (encL.getVelocity() + encR.getVelocity()) / 2;
  }

  public double angle() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("enc", centerDist());
      SmartDashboard.putNumber("encV", centerVel());
      SmartDashboard.putNumber("ppr", encL.getCountsPerRevolution());
  }
}
