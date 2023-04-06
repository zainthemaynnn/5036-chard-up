// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

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
  private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);
  private static final double GEAR_RATIO = 8.45;
  private static final double k = (double) 2*Math.PI*WHEEL_RADIUS_METERS / GEAR_RATIO;

  private CANSparkMax l1, l2, r1, r2;
  private List<CANSparkMax> motors;
  private RelativeEncoder encL, encR;
  private AHRS gyro;

  private DifferentialDriveKinematics kinematics;
  private ChassisSpeeds chassisSpeeds;
  private DifferentialDrivePoseEstimator estimator;
  private Pose2d pose;

  private Dashboard dashboard = new Dashboard("Drive");

  private double pre_gyro = 0.;

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
    this.motors = List.of(l1, l2, r1, r2);
    this.encL = encL;
    this.encR = encR;
    this.gyro = gyro;

    encL.setPosition(0.0);
    encR.setPosition(0.0);
    encL.setPositionConversionFactor(k);
    encR.setPositionConversionFactor(k);
    encL.setVelocityConversionFactor(k);
    encR.setVelocityConversionFactor(k);

    l1.enableVoltageCompensation(12.0);
    l2.enableVoltageCompensation(12.0);
    r1.enableVoltageCompensation(12.0);
    r2.enableVoltageCompensation(12.0);

    l2.follow(l1);
    r2.follow(r1);

    gyro.resetDisplacement();

    dashboard
      .add(gyro, "gyro");
  }

  public void encReset() {
    encL.setPosition(0.0);
    encR.setPosition(0.0);
  }

  public double rampRateAcc = 10.0;
  public double rampRateDec = 10.0;
  public boolean rampEnabled = true;
  public double rampRateExp = 2.0;
  private static final double EXPONENTIAL_RAMP_NEGATE_THRESHOLD_PERCENT = .05;

  private double toRampedExp(double actual, double pow) {
    if (rampEnabled) {
      double res;
      double absActual = Math.abs(actual), absPow = Math.abs(pow);
      
      // magnitude
      if (Math.signum(pow) == Math.signum(actual)) {
        if (absActual < absPow) {
          res = absActual * Math.pow(rampRateExp, 0.020);
        } else if (absActual > absPow) {
          res = absActual * Math.pow(1/rampRateExp, 0.020);
        } else {
          res = absActual;
        }
      } else {
        if (absActual > absPow) {
          res = absActual * Math.pow(rampRateExp, 0.020);
        } else if (absActual < absPow) {
          res = absActual * Math.pow(1/rampRateExp, 0.020);
        } else {
          res = absActual;
        }
      }

      // direction
      if (res <= EXPONENTIAL_RAMP_NEGATE_THRESHOLD_PERCENT) {
        if (pow == 0) {
          res = 0;
        } else {
          res = Math.copySign(EXPONENTIAL_RAMP_NEGATE_THRESHOLD_PERCENT, pow);
        }
      } else {
        res = Math.copySign(res, actual);
      }

      // align power
      //if ((pow < res && res < actual) || (actual < res && res < pow)) {
      //  res = pow;
      //}

      System.out.println(res);

      return res;
    } else {
      return pow;
    }
  }

  private double toRamped(double actual, double pow) {
    double rampRate = Math.abs(actual) > Math.abs(pow) ? rampRateAcc : rampRateDec;
    if (rampEnabled) {
      if (pow > actual) {
        return actual + Math.min(rampRate, Math.abs(pow - actual)) * 0.020;
      } else if (pow < actual) {
        return actual - Math.min(rampRate, Math.abs(pow - actual)) * 0.020;
      } else {
        return pow;
      }
    } else {
      return pow;
    }
  }

  public void arcadeDrive(double throttle, double wheel) {
    l1.set(toRamped(l1.get(), throttle + wheel));
    r1.set(toRamped(r1.get(), throttle - wheel));
  }

  public double centerDistMeters() {
    return (encL.getPosition() + encR.getPosition()) / 2;
  }

  public double centerVelMeters() {
    return (encL.getVelocity() + encR.getVelocity()) / 2;
  }

  public double angle() {
    return Math.toRadians(gyro.getAngle());
  }

  public double angleVel() {
    var res = (gyro.getAngle() - pre_gyro) / 0.020;
    pre_gyro = angle();
    return res;
  }

  public double pitch() {
    return Math.toRadians(gyro.getPitch());
  }

  public void setIdleMode(IdleMode mode) {
    motors.forEach(m -> m.setIdleMode(mode));
  }

  public void setRampRate(double rate) {
    motors.forEach(m -> m.setOpenLoopRampRate(rate));
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("enc", centerDistMeters());
      SmartDashboard.putNumber("encV", centerVelMeters());
      SmartDashboard.putNumber("ppr", encL.getCountsPerRevolution());
  }
}
