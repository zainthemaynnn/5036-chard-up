// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.hid.Logitech;
import frc.math.VelocityTuner;
import frc.robot.commands.Align;
import frc.robot.commands.Balance;
import frc.robot.commands.BalanceNoTaxi;
import frc.robot.commands.Taxi;
import frc.robot.commands.DriveArm;
import frc.robot.commands.DriveAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Blinkin.BlinkinColor;

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
  private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  public Drive drive = new Drive(l1, l2, r1, r2, l1.getEncoder(), r1.getEncoder(), gyro);
  public Arm arm = new Arm(armController);
  public Intake intake = new Intake(intakeController);
  public Blinkin leds = new Blinkin(RobotMap.PWM.BLINKIN.port());

  private VelocityTuner tuner = new VelocityTuner(drive::angle, drive::angleVel,
      (in) -> drive.arcadeDrive(0., in));

  public RobotContainer() {
    configureBindings();
  }

  private double dir = 1;
  private enum ArmState {
    UP,
    DOWN,
    MID,
  }

  private enum IntakeState {
    CUBE_IN,
    CONE_IN,
    EMPTY,
  }

  private ArmState armState = ArmState.DOWN;

  private IntakeState intakeState = IntakeState.EMPTY;

  private DriveArm
    driveArmRest = new DriveArm(arm, 0.),
    driveArmMid = new DriveArm(arm, 0.), // TODO
    driveArmHigh = new DriveArm(arm, 0.); // TODO

  private Command
    armOut = new StartEndCommand(
      () -> {
        arm.drivePower(.40);
        armState = ArmState.UP;
      },
      () -> arm.drivePower(0.),
      arm
    ),
    armIn = new StartEndCommand(
      () -> {
        arm.drivePower(-.40);
        armState = ArmState.DOWN;
      },
      () -> arm.drivePower(0.),
      arm
    );

  private Command
    coneIn = new StartEndCommand(
      () -> intake.drivePower(-controllerToo.getAxisValue(Logitech.Axis.R2), false),
      () -> intake.drivePower(0., false),
      intake
    ),

    cubeIn = new StartEndCommand(
      () -> intake.drivePower(controllerToo.getAxisValue(Logitech.Axis.L2), true),
      () -> intake.drivePower(0., false),
      intake
    ),
    cubeOut = new StartEndCommand(
      () -> { intake.drivePower(-1, false); intakeState = IntakeState.EMPTY; },
      () -> intake.drivePower(0., false),
      intake
    ),
    coneOut = new StartEndCommand(
      () -> { intake.drivePower(1, false); intakeState = IntakeState.EMPTY; },
      () -> intake.drivePower(0., false),
      intake
    );

  private ShuffleboardTab control = Shuffleboard.getTab("Control");
  private SimpleWidget
    bridgeTarget = control.add("bridge target", 0.),
    bridgeDelay = control.add("bridge delay", 0.);

  private Command autoCommand = new InstantCommand();
  private enum AutoState {
    NOTHING,
    BRIDGE,
    DROPKICK,
    NOTAXI,
  }
  private SendableChooser<AutoState> autoSelector = new SendableChooser<>();

  private Align
    alignRight = new Align(drive, cams, 1),
    alignLeft = new Align(drive, cams, -1);

  public void enableDrive() {
    drive.setDefaultCommand(new RunCommand(() -> {
      boolean locked = controller.getAxisValue(Logitech.Axis.R2) > .75;
      double pow = locked ? .25 : (.25 +  (.75 - controller.getAxisValue(Logitech.Axis.R2)));
      drive.setIdleMode(locked ? IdleMode.kBrake : IdleMode.kBrake);
      drive.rampEnabled = !(locked);
      drive.arcadeDrive(
        controller.getAxisValue(Logitech.Axis.LEFT_Y) * pow,
        controller.getAxisValue(Logitech.Axis.RIGHT_X) * pow * .40
      );
    }, drive));
  }
  

  private Pickup pickup = Pickup.CUBE;

  private double hardstopUpTicks = 0;
  private double hardstopDownTicks = 0;
  private double topAngle = 0.0;
  private boolean topAngleRecorded = false;

  private void configureBindings() {
    control.add(autoSelector);
    autoSelector.setDefaultOption("Dropkick", AutoState.DROPKICK);
    leds.addColor(BlinkinColor.I_WANT_CUBE);
    leds.display();

    /*controller.getButton(Logitech.Button.YELLOW).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(Units.feetToMeters(1.0*dir));
    }, drive));

    controller.getButton(Logitech.Button.RED).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(Units.feetToMeters(2.0*dir));
    }, drive));

    controller.getButton(Logitech.Button.GREEN).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(Units.feetToMeters(3.0*dir));
    }, drive));

    controller.getButton(Logitech.Button.BLUE).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(Units.feetToMeters(4.0*dir));
    }, drive));*/

    /*controller.getButton(Logitech.Button.YELLOW).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(45);
    }, drive));

    controller.getButton(Logitech.Button.RED).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(90);
    }, drive));

    controller.getButton(Logitech.Button.GREEN).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(135);
    }, drive));

    controller.getButton(Logitech.Button.BLUE).whileTrue(new RunCommand(() -> {
      tuner.driveVelocity(180);
    }, drive));*/

    controller.getButton(Logitech.Button.L1).onTrue(new InstantCommand(() -> dir *= -1));

    //controllerToo.getButton(Logitech.Button.BLUE).toggleOnTrue(new InstantCommand(() -> armState = ArmState.UP));
    //controllerToo.getButton(Logitech.Button.GREEN).toggleOnTrue(new InstantCommand(() -> armState = ArmState.MID));
    //controllerToo.getButton(Logitech.Button.RED).toggleOnTrue(new InstantCommand(() -> armState = ArmState.DOWN));
    controllerToo.getButton(Logitech.Button.BLUE).whileTrue(new RunCommand(() -> { arm.drivePower(.30); armState = ArmState.UP; }, arm));
    controllerToo.getButton(Logitech.Button.RED).whileTrue(new RunCommand(() -> { arm.drivePower(-.30); armState = ArmState.DOWN; }, arm));

    controllerToo.getButton(Logitech.Button.L1).whileTrue(cubeOut);
    //controllerToo.getAxis(Logitech.Axis.L2).whileTrue(cubeIn);
    controllerToo.getButton(Logitech.Button.R1).whileTrue(coneOut);
    //controllerToo.getAxis(Logitech.Axis.R2).whileTrue(coneIn);
    /*controllerToo.getButton(Logitech.Button.YELLOW).toggleOnTrue(new InstantCommand(() -> {
      if (pickup == Pickup.CUBE) {
        if (leds.containsColor(BlinkinColor.I_WANT_CUBE)) {
          leds.removeColor(BlinkinColor.I_WANT_CUBE);
        }
        leds.addColor(BlinkinColor.I_WANT_CONE);
        pickup = Pickup.CONE;
      } else {
        if (leds.containsColor(BlinkinColor.I_WANT_CONE)) {
          leds.removeColor(BlinkinColor.I_WANT_CONE);
        }
        leds.addColor(BlinkinColor.I_WANT_CUBE);
        pickup = Pickup.CUBE;
      }
      leds.display();
    }));*/

    controllerToo.getButton(Logitech.Button.YELLOW).toggleOnTrue(new InstantCommand(() -> {
      /*if (leds.containsColor(BlinkinColor.I_WANT_CUBE)) {
        leds.removeColor(BlinkinColor.I_WANT_CUBE);
      }
      if (leds.containsColor(BlinkinColor.I_WANT_CONE)) {
        leds.removeColor(BlinkinColor.I_WANT_CONE);
      }
      leds.display();*/
      //arm.reset();
    }));

    controller.getButton(Logitech.Button.BLUE).whileTrue(alignLeft);
    controller.getButton(Logitech.Button.RED).whileTrue(alignRight);

    arm.setDefaultCommand(new RunCommand(() -> {
      /*switch (armState) {
        case UP:
          if (Math.abs(arm.velocity()) < Math.toRadians(3.0) && ++hardstopUpTicks >= 5) {
            arm.drivePower(.05);
            arm.setPosition(Math.toRadians(161.0));
          } else {
            System.out.println("up");
            arm.drivePower(.30);
            hardstopUpTicks = 0;
          }
        case DOWN:
          if (Math.abs(arm.velocity()) < Math.toRadians(3.0) && ++hardstopDownTicks >= 5) {
            arm.drivePower(-.05);
            arm.setPosition(0.0);
          } else {
            arm.drivePower(-.30);
            hardstopDownTicks = 0;
          }
          break;
        case MID:
          var pow = -(arm.position() - Math.toRadians(38.0)) * 0.5;
          arm.drivePower(Math.min(Math.abs(pow), .30) * Math.signum(pow));
          System.out.println(Math.min(Math.abs(pow), .30) * Math.signum(pow));
          break;
      }*/
      arm.drivePower(armState == ArmState.UP ? .05 : 0.);
  }, arm));

    intake.setDefaultCommand(new RunCommand(() -> {
      var l2 = controllerToo.getAxisValue(Logitech.Axis.L2);
      var r2 = controllerToo.getAxisValue(Logitech.Axis.R2);
      if (l2 > .02) {
        intake.drivePower(l2*.5, true);
        intakeState = IntakeState.CUBE_IN;
      } else if (r2 > .02) {
        intake.drivePower(-1., false);
        intakeState = IntakeState.CONE_IN;
      } else {
        intake.drivePower(intakeState == IntakeState.CONE_IN ? -1 : 0., false);
      }
    }, intake));

    autoSelector.addOption("Nothing", AutoState.NOTHING);
    autoSelector.addOption("Bridge", AutoState.BRIDGE);
    autoSelector.addOption("Dropkick", AutoState.DROPKICK);
    autoSelector.addOption("NoTaxi", AutoState.NOTAXI);

    control.addDouble("pitch", () -> Math.toDegrees(drive.pitch()));
    control.addDouble("angle", drive::angle);
    control.addDouble("enc", () -> drive.centerDistMeters());
    control.addDouble("arm", () -> Math.toDegrees(arm.position()));
    control.addDouble("armv", () -> Math.toDegrees(arm.velocity()));

    control.addBoolean("has", () -> cams.feed().getEntry("tv").getBoolean(false));
    control.addDouble("x", () -> cams.poseTagRelative().getX());
    control.addDouble("y", () -> cams.poseTagRelative().getZ());
  }

  public Command getAutonomousCommand() {
    SequentialCommandGroup cmd;
    switch (autoSelector.getSelected()) {
      case NOTHING:
        cmd = new SequentialCommandGroup(
          /*new InstantCommand(() -> drive.setIdleMode(IdleMode.kBrake)),
          new InstantCommand(() -> arm.drivePower(.30)),
          new WaitCommand(1.0),
          new InstantCommand(() -> arm.drivePower(0.)),
          new InstantCommand(() -> intake.drivePower(-1, false)),
          new WaitCommand(3.0),
          new InstantCommand(() -> intake.drivePower(0., false)),
          new InstantCommand(() -> arm.drivePower(-.30)),
          new WaitCommand(1.0),
          new InstantCommand(() -> arm.drivePower(0.))*/
          new Taxi(drive, leds, 0.)
        );
        cmd.addRequirements(arm, intake);
        return cmd;
      case BRIDGE:
        cmd = new SequentialCommandGroup(
          new InstantCommand(() -> drive.setIdleMode(IdleMode.kBrake)),
          new InstantCommand(() -> arm.drivePower(.30)),
          new WaitCommand(1.0),
          new InstantCommand(() -> arm.drivePower(0.)),
          new InstantCommand(() -> drive.arcadeDrive(.15, 0.)),
          new WaitCommand(1.0),
          new InstantCommand(() -> drive.arcadeDrive(0., 0.)),
          new InstantCommand(() -> intake.drivePower(-1, false)),
          new WaitCommand(2.0),
          new InstantCommand(() -> intake.drivePower(0., false)),
          new InstantCommand(() -> arm.drivePower(-.30)),
          new WaitCommand(1.0),
          new InstantCommand(() -> arm.drivePower(0.)),
          new WaitCommand(bridgeDelay.getEntry("bridge delay").getDouble(0.)),
          new Taxi(drive, leds, .20)
        );
        cmd.addRequirements(arm, intake, drive);
        return cmd;
      case DROPKICK:
        cmd = new SequentialCommandGroup(
          new InstantCommand(() -> drive.setIdleMode(IdleMode.kBrake)),
          new InstantCommand(() -> drive.arcadeDrive(0., 0.)),
          new InstantCommand(() -> arm.drivePower(.40)),
          new WaitCommand(1.0),
          new InstantCommand(() -> arm.drivePower(0.)),
          new InstantCommand(() -> drive.arcadeDrive(.15, 0.)),
          new WaitCommand(1.0),
          new InstantCommand(() -> drive.arcadeDrive(0., 0.)),
          new InstantCommand(() -> intake.drivePower(-1, false)),
          new WaitCommand(2.0),
          new InstantCommand(() -> intake.drivePower(0., false)),
          new InstantCommand(() -> arm.drivePower(-.40)),
          new WaitCommand(1.0),
          new WaitCommand(bridgeDelay.getEntry("bridge delay").getDouble(0.)),
          new Balance(drive, leds, .20)
          //new WaitCommand(bridgeDelay.getEntry("bridge delay").getDouble(0.)),
          //new BLESSEDAuto(drive, -60.0)
        );
        cmd.addRequirements(arm, intake, drive);
        return cmd;
      case NOTAXI:
        cmd = new SequentialCommandGroup(
          new InstantCommand(() -> drive.setIdleMode(IdleMode.kBrake)),
          new InstantCommand(() -> drive.arcadeDrive(0., 0.)),
          new InstantCommand(() -> arm.drivePower(.40)),
          new WaitCommand(1.0),
          new InstantCommand(() -> arm.drivePower(0.)),
          new InstantCommand(() -> drive.arcadeDrive(.15, 0.)),
          new WaitCommand(1.0),
          new InstantCommand(() -> drive.arcadeDrive(0., 0.)),
          new InstantCommand(() -> intake.drivePower(-1, false)),
          new WaitCommand(2.0),
          new InstantCommand(() -> intake.drivePower(0., false)),
          new InstantCommand(() -> arm.drivePower(-.40)),
          new WaitCommand(1.0),
          new WaitCommand(bridgeDelay.getEntry("bridge delay").getDouble(0.)),
          new BalanceNoTaxi(drive, leds, .20)
          //new WaitCommand(bridgeDelay.getEntry("bridge delay").getDouble(0.)),
          //new BLESSEDAuto(drive, -60.0)
        );
        cmd.addRequirements(arm, intake, drive);
        return cmd;
      default:
        return new PrintCommand("unrecognized option");
    }
  }
}
