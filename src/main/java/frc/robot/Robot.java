// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final double KICK_MOTOR_TURN_TO_GEARBOX_TURN = 1.0 / 3.0;
  private static final double GEARBOX_TURN_TO_DEGREES = 360;

  // Mecanum Drive
  private CANSparkMax frontLeft;
  private CANSparkMax rearLeft;
  private CANSparkMax frontRight;
  private CANSparkMax rearRight;
  private MecanumDrive drive;
  private Joystick joystick;

  // Arm
  private Joystick mechJoystick = new Joystick(1);
  private CANSparkMax armLeft = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax armRight = new CANSparkMax(6, MotorType.kBrushless);
  private MotorControllerGroup arm;

  // Telescoping
  private CANSparkMax Telescope = new CANSparkMax(7, MotorType.kBrushless);

  // Wrist & Claw
  private CANSparkMax wrist = new CANSparkMax(8, MotorType.kBrushless);
  private CANSparkMax claw = new CANSparkMax(9, MotorType.kBrushless);

  // Kickwheel
  private CANSparkMax kickmotor = new CANSparkMax(10, MotorType.kBrushless);
  private RelativeEncoder kickMotorEncoder;
  private SparkMaxPIDController kickMotorController;
  private CANSparkMax kickWHEEL = new CANSparkMax(11, MotorType.kBrushless);

  private double kickMotorAngleDegrees = 0.0;
  private boolean isKickMotorDown = true;

  private Timer autoTimer;

  private SendableChooser<String> autoChooser = new SendableChooser<>();
  private String chosenAuto = null;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    autoChooser.setDefaultOption("Run Auto", "auto");
    autoChooser.addOption("No Auto", null);

    SmartDashboard.putData(autoChooser);

    CameraServer.startAutomaticCapture();
    joystick = new Joystick(0);
    mechJoystick = new Joystick(1);

    // Drive Train
    frontLeft = new CANSparkMax(1, MotorType.kBrushless);
    frontLeft.restoreFactoryDefaults();
    frontLeft.setInverted(false);
    frontLeft.burnFlash();

    rearLeft = new CANSparkMax(3, MotorType.kBrushless);
    rearLeft.restoreFactoryDefaults();
    rearLeft.setInverted(false);
    rearLeft.burnFlash();

    frontRight = new CANSparkMax(2, MotorType.kBrushless);
    frontRight.restoreFactoryDefaults();
    frontRight.setInverted(true);
    frontRight.burnFlash();

    rearRight = new CANSparkMax(4, MotorType.kBrushless);
    rearRight.restoreFactoryDefaults();
    rearRight.setInverted(true);
    rearRight.burnFlash();

    drive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    // Arm movement
    armLeft.restoreFactoryDefaults();
    armLeft.setInverted(false);
    armLeft.setIdleMode(IdleMode.kBrake);
    armLeft.burnFlash();

    armRight.restoreFactoryDefaults();
    armRight.setInverted(false);
    armRight.setIdleMode(IdleMode.kBrake);
    armRight.burnFlash();

    arm = new MotorControllerGroup(armLeft, armRight);
    arm.setInverted(true);

    // Telescoping
    Telescope.restoreFactoryDefaults();
    Telescope.setInverted(true);
    Telescope.setIdleMode(IdleMode.kBrake);
    Telescope.burnFlash();

    // Wrist
    wrist.restoreFactoryDefaults();
    wrist.setInverted(false);
    wrist.setIdleMode(IdleMode.kBrake);
    wrist.burnFlash();

    // Claw
    claw.restoreFactoryDefaults();
    claw.setInverted(true);
    claw.setIdleMode(IdleMode.kBrake);
    claw.burnFlash();

    // Kickmotor
    kickmotor.restoreFactoryDefaults();
    kickmotor.setInverted(false);
    kickmotor.setIdleMode(IdleMode.kBrake);

    kickMotorEncoder = kickmotor.getEncoder();
    kickMotorEncoder.setPosition(0);
    kickMotorEncoder.setPositionConversionFactor(KICK_MOTOR_TURN_TO_GEARBOX_TURN * GEARBOX_TURN_TO_DEGREES);

    kickMotorController = kickmotor.getPIDController();
    //kickMotorController.setP(0.1);
    kickMotorController.setP(0.05);
    kickMotorController.setI(0.0001);
    //kickMotorController.setI(0);
    //kickMotorController.setD(1);
    kickMotorController.setD(0.0);
    kickMotorController.setIZone(0);
    kickMotorController.setFF(0);
    kickMotorController.setOutputRange(-1, 1);
    kickmotor.burnFlash();

    kickWHEEL.restoreFactoryDefaults();
    kickWHEEL.setInverted(false);
    kickWHEEL.setIdleMode(IdleMode.kBrake);
    kickWHEEL.burnFlash();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    kickMotorEncoder.setPosition(0);
    kickMotorAngleDegrees = -20.0;
    isKickMotorDown = false;
    kickMotorController.setReference(kickMotorAngleDegrees, com.revrobotics.CANSparkMax.ControlType.kPosition);

    autoTimer = new Timer();
    autoTimer.reset();
    autoTimer.start();

    chosenAuto = autoChooser.getSelected();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // No auto? Skip.
    if (chosenAuto == null) {
      drive.driveCartesian(0, 0, 0);
      return;
    }

    double time = autoTimer.get();
    if (time < 0.5) {
      // Go back
      drive.driveCartesian(-0.3, 0, 0);
    } else if (time < 1) {
      drive.driveCartesian(0, 0, 0);
    } else if (time < 3) {
      // Go forward
      drive.driveCartesian(0.25, 0, 0);
    } else {
      drive.driveCartesian(0, 0, 0);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double robotX = getRobotForwardSpeed();
    double robotY = getRobotRightSpeed();
    double robotRot = getRobotRotationSpeed();
    drive.driveCartesian(robotX, robotY, robotRot);

    if (joystick.getRawButtonReleased(3)) {
      kickMotorAngleDegrees = 0.0;
      isKickMotorDown = true;
    } else if (joystick.getRawButtonReleased(4)) {
      kickMotorAngleDegrees = -20.0;
      isKickMotorDown = false;
    }

    kickMotorController.setReference(kickMotorAngleDegrees, com.revrobotics.CANSparkMax.ControlType.kPosition);

    if (isKickMotorDown) {
      kickWHEEL.set(frontLeft.get() * 0.418);
    } else {
      kickWHEEL.set(0.0);
    }

    double armspeed = mechJoystick.getRawAxis(5);
    if (armspeed < 0) {
      armspeed *= 0.15;
    } else {
      armspeed *= 0.25;
    }
    arm.set(armspeed);

    double rightTrigger = mechJoystick.getRawAxis(3);
    double leftTrigger = mechJoystick.getRawAxis(2);
    if (leftTrigger > 0.1) {
      claw.set(-leftTrigger * .4);
    } else if (rightTrigger > 0.1) {
      claw.set(rightTrigger * .4);
    } else {
      claw.set(0);
    }

    wrist.set(mechJoystick.getRawAxis(1) * 0.2);

    if (mechJoystick.getPOV() == 0) {
      Telescope.set(.25);
    } else if (mechJoystick.getPOV() == 180) {
      Telescope.set(-0.25);
    } else {
      Telescope.set(0);
    }

  }

  private double getRobotRightSpeed() {

    if (joystick.getTrigger()){
      return joystick.getX() * 0.1;
    }
      else {
        return joystick.getX();
      }
  }

  private double getRobotForwardSpeed() {
    if (joystick.getTrigger()){
      return -joystick.getY() * 0.1;
    }
    else {
      return -joystick.getY();
    }
    
  }

  private double getRobotRotationSpeed() {
    double value = joystick.getZ();
    if (Math.abs(value) < 0.1) {
      return 0.0;
    }
    if (joystick.getTrigger()){
      return Math.pow(value, 3) * .1;
    }
    else {
      return Math.pow(value, 3) * .25;
    }
    

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
