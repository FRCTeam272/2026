// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.PIDSettings;
import frc.lib.utils.SparkMAXContainer;
import frc.lib.utils.TalonFxContainer;
import frc.lib.utils.TrimPot;
import frc.robot.Constants;
import frc.robot.sub_containers.DriveBaseContainer;

public class Shooter extends SubsystemBase {
  public Translation2d targetPose = DriverStation.isDSAttached()
      && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
          ? new Translation2d(12.5, 4) // if red allaince
          : new Translation2d(3.5, 4); // if blue alliance

  /** Creates a new Shooter. */
  TalonFxContainer flywheel;
  TalonFxContainer flywheelFollower;
  SparkMAXContainer hood;

  final int FLYWHEEL_LOCATION = 4;
  final int FLYWHEEL_FOLLOWER_LOCATION = 6;
  final int HOOD_LOCATION = 5;
  int speedThreshold = 200;
  int angleThreshold = 2;

  public boolean useAutoFlywheel = true;

  public TrimPot hoodTrim = new TrimPot("HoodTrim");
  public TrimPot flywheelTrim = new TrimPot("FlywheelTrim");

  public double targetVelocity = Constants.SHOOTER_AUTO_PID_SETTINGS.target_velocity;
  public double targetHood = 0;

  // Cache last PID values to prevent constant re-configuration
  private double lastP, lastI, lastD, lastV, lastA;

  private DriveBaseContainer driveBase;

  public void setDriveBase(DriveBaseContainer driveBaseContainer) {
    this.driveBase = driveBaseContainer;
  }

  public void updateTarget() {
    var allaince = DriverStation.getAlliance();
    if (allaince.isEmpty())
      return;
    this.targetPose = allaince.get().equals(DriverStation.Alliance.Red)
        ? new Translation2d(12.5, 4) // if red allaince
        : new Translation2d(3.5, 4); // if blue alliance
  }

  public Shooter() {
    flywheel = new TalonFxContainer(FLYWHEEL_LOCATION, true);
    flywheelFollower = new TalonFxContainer(FLYWHEEL_FOLLOWER_LOCATION, true);

    for (var motor : new TalonFxContainer[] { flywheel, flywheelFollower }) {

      motor.assignPIDSettings(Constants.SHOOTER_LOW_PID_SETTINGS, 0);
      motor.assignPIDSettings(Constants.SHOOTER_MID_PID_SETTINGS, 1);
      motor.assignPIDSettings(Constants.SHOOTER_HIGH_PID_SETTINGS, 2);

      motor.setBreakMode(false);
      motor.configurator.Audio.BeepOnConfig = false;
      motor.motor.getVelocity().setUpdateFrequency(20);
      motor.applyConfig();
    }

    flywheelFollower.setupAsFollowerMotor(flywheel, false);

    hood = new SparkMAXContainer(HOOD_LOCATION);
    hood.assignPIDValues(.2, 0, 0);
    hood.setCurrentLimit(40);

    lastP = Constants.SHOOTER_AUTO_PID_SETTINGS.kI;
    lastI = Constants.SHOOTER_AUTO_PID_SETTINGS.kP;
    lastD = Constants.SHOOTER_AUTO_PID_SETTINGS.kD;
    lastV = Constants.SHOOTER_AUTO_PID_SETTINGS.kV;
    lastA = Constants.SHOOTER_AUTO_PID_SETTINGS.kA;

    this.driveBase = driveBase;
    this.UpdatePID(Constants.SHOOTER_AUTO_PID_SETTINGS);

    this.setupSmartDashboard();
    this.forceSync();
  }

  boolean firstIteration = true;

  private void setupSmartDashboard() {
    SmartDashboard.putBoolean("Shooter/UseAutoFlywheel", useAutoFlywheel);
    SmartDashboard.putNumber("FlyWheel/TargetVelocity", 5200);
    SmartDashboard.putNumber("Hood/Target", 4.8);
    PIDSettings settings = Constants.SHOOTER_AUTO_PID_SETTINGS;
    if (!firstIteration) {
      switch (flywheel.currentSlot) {
        case 0:
          settings = Constants.SHOOTER_LOW_PID_SETTINGS;
          break;
        case 1:
          settings = Constants.SHOOTER_MID_PID_SETTINGS;
          break;
        default:
          settings = Constants.SHOOTER_HIGH_PID_SETTINGS;
          break;
      }
    }
    this.firstIteration = false;
    setupSmartDashboard(settings);
    SmartDashboard.putNumber("Shooter/AutoFlywheelStage", 0);
  }

  private void setupSmartDashboard(PIDSettings pidConstants) {
    SmartDashboard.putNumber("Shooter/P", pidConstants.kP);
    SmartDashboard.putNumber("Shooter/I", pidConstants.kI);
    SmartDashboard.putNumber("Shooter/D", pidConstants.kD);
    SmartDashboard.putNumber("Shooter/kV", pidConstants.kV);
    SmartDashboard.putNumber("Shooter/kA", pidConstants.kA);
  }

  public boolean TrueStop() {
    flywheel.motor.set(0);
    return true;
  }

  public boolean SpinWheel(double target_velocity) {
    if (target_velocity == 0)
      return TrueStop();
    target_velocity = -(Math.abs(target_velocity) + flywheelTrim.adjusterValue);
    return flywheel.setVelocity(target_velocity, 250);
  }

  public boolean AdjustHood(double target_angle) {
    return hood.goToPostion(target_angle + hoodTrim.adjusterValue, angleThreshold);
    // hood.motor.set(.2 * Math.signum(target_angle));
    // return true;
  }

  public boolean AdjustHoodIncremental(double value) {
    var target = hood.getPosition() + value;
    if (target >= 0)
      driveHood();
    if (target < -5.5)
      target = -5.5;
    return hood.goToPostion(target, 0);
  }

  final TalonFxContainer[] flywheels = new TalonFxContainer[] { flywheel, flywheelFollower };

  public void UpdatePID(PIDSettings settings) {
    for (var motor : flywheels) {
      if (motor == null)
        continue;
      dynamicPID(settings.kP, settings.kI, settings.kD, motor);
      dynamicFeedForward(settings.kV, settings.kA, motor);
    }
  }

  private void dynamicPID(double kP, double kI, double kD, TalonFxContainer motor) {
    if (motor == null)
      return;
    motor.assignPIDValues(kP, kI, kD, motor.currentSlot);
  }

  private void dynamicFeedForward(double kV, double kA, TalonFxContainer motor) {
    if (motor == null)
      return;
    motor.assignFF(0, kV, kA, 0, motor.currentSlot);
  }

  public static boolean isBetween(double value, double min, double max, double threshold) {
    // Ensure min is actually the smaller number
    double realMin = Math.min(min, max) - threshold;
    double realMax = Math.max(min, max) + threshold;

    return value >= realMin && value <= realMax;
  }

  private boolean isShooterUpToSpeed() {
    return isBetween(this.flywheel.getVelocity(), targetVelocity, targetVelocity, speedThreshold);
  }

  private boolean isHoodCorrect() {
    return Math.abs(this.hood.getPosition()) < 1;
  }

  double lastTargetVelocity = targetVelocity;
  double hoodTarget = 0;
  int debounceTime = 150;
  int debounceCount = 0;

  public void forceSync() {
    this.debounceCount = this.debounceTime;
  }

  boolean lock_hood = false;

  public void driveHood() {
    lock_hood = true;
    this.hood.motor.set(.3);
  }

  /**
   * Calculates the distance from the target (hub) using the robot's current pose
   * and the known location of the hub.
   * This can be used to adjust the flywheel speed and hood angle dynamically
   * based on how far the robot is from the target.
   *
   * @return The distance from the target in inches.
   */
  // TODO: do we subtract 8 inches for the intake comp
  private double getDistanceFromTarget() {
    updateTarget();
    var ourDistance = driveBase.getPose();
    var distance = ourDistance.getTranslation().getDistance(targetPose);
    return (distance * 39.37); // convert to inches
  }

  public void autoFlywheel() {
    SmartDashboard.putBoolean("Shooter/UseAutoFlywheel", useAutoFlywheel);
    if (!useAutoFlywheel) {
      return;
    }

    // get the distance

    double distance = Math.abs(getDistanceFromTarget());
    SmartDashboard.putNumber("Shooter/Distance", distance);
    var stage = 0;
    PIDSettings settings;
    // reference that to the table
    if (distance <= 24) {
      this.targetVelocity = 3600;
      this.hoodTarget = (distance * .276) - 2.70; // multiple by negitive one cause its in the negitive space
      settings = Constants.SHOOTER_LOW_PID_SETTINGS;
    } else if (distance <= 70) {
      this.targetVelocity = 4150;
      this.hoodTarget = (distance * .088) - 0.875; // multiple by negitive one cause its in the negitive space
      settings = Constants.SHOOTER_4150_PID_SETTINGS;
      stage = 1;
    } else if (distance <= 110) {
      this.targetVelocity = 4700;
      this.hoodTarget = (distance * .079) - 3.55;
      settings = Constants.SHOOTER_4700_PID_SETTINGS;
      stage = 2;
    } else {
      this.targetVelocity = 5500;
      this.hoodTarget = (distance * .087) - 5.05;
      settings = Constants.SHOOTER_5500_PID_SETTINGS;
      stage = 3;
    }

    if (hoodTarget > 0) {
      hoodTarget *= -1;
    }

    if (hoodTarget < -7.3) {
      hoodTarget = -7.3;
    }

    // force PID to update on smartdashboard
    for (var motor : flywheels) {
      if (motor == null)
        continue;
      dynamicPID(settings.kP, settings.kI, settings.kD, motor);
      dynamicFeedForward(settings.kV, settings.kA, motor);
    }
    SmartDashboard.putNumber("Shooter/P", settings.kP);
    SmartDashboard.putNumber("Shooter/I", settings.kI);
    SmartDashboard.putNumber("Shooter/D", settings.kD);
    SmartDashboard.putNumber("Shooter/kV", settings.kV);
    SmartDashboard.putNumber("Shooter/kA", settings.kA);
    // force targets to update on smartdashboard
    SmartDashboard.putNumber("FlyWheel/TargetVelocity", targetVelocity);
    SmartDashboard.putNumber("Hood/Target", hoodTarget);
    SmartDashboard.putNumber("Shooter/AutoFlywheelStage", stage);
    // update PID
    forceSync();
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("FlyWheel/CurrentVelocity/Main", flywheel.getVelocity());
    SmartDashboard.putNumber("FlyWheel/CurrentVelocity/Follower", flywheelFollower.getVelocity());
    if (DriverStation.isAutonomous())
      return;
    SmartDashboard.putNumber("Shooter/Distance", getDistanceFromTarget());
    // TODO: REMOVE
    SmartDashboard.putNumber("FlyWheel/CurrentVelocity/Main", flywheel.getVelocity());
    SmartDashboard.putBoolean("Ready/Flywheel Up To Speed", this.isShooterUpToSpeed());
    // debounceCount = debounceTime;
    // if (DriverStation.isTestEnabled()) debounceCount = debounceTime;
    if (debounceCount == debounceTime) {
      debounceCount = 0;

      targetVelocity = SmartDashboard.getNumber("FlyWheel/TargetVelocity", targetVelocity);
      SmartDashboard.putNumber("FlyWheel/CurrentVelocity/Main", flywheel.getVelocity());
      SmartDashboard.putNumber("FlyWheel/CurrentVelocity/Follower", flywheelFollower.getVelocity());
      hood.reportMotor("ShooterHood");
      hoodTarget = SmartDashboard.getNumber("Hood/Target", hoodTarget);
      if (lock_hood) {
        lock_hood = false;
      } else {
        if (hoodTarget > 0)
          hoodTarget *= -1;
        hood.goToPostion(hoodTarget);
      }

      final double p = SmartDashboard.getNumber("Shooter/P", lastP);
      final double i = SmartDashboard.getNumber("Shooter/I", lastI);
      final double d = SmartDashboard.getNumber("Shooter/D", lastD);
      final double v = SmartDashboard.getNumber("Shooter/kV", lastV);
      final double a = SmartDashboard.getNumber("Shooter/kA", lastA);

      // ONLY configure if values have changed
      if (p != lastP || i != lastI || d != lastD) {
        dynamicPID(p, i, d, flywheel);
        dynamicPID(p, i, d, flywheelFollower);
        lastP = p;
        lastI = i;
        lastD = d;
      }

      if (v != lastV || a != lastA) {
        dynamicFeedForward(v, a, flywheel);
        dynamicFeedForward(v, a, flywheelFollower);
        lastV = v;
        lastA = a;
      }

      flywheel.getPID("Shooter/PID_Actual/");
      flywheel.reportMotor("ShooterVals");

      flywheel.getPID("Shooter/PID_Actual/");
      flywheel.reportMotor("ShooterVals");

      SmartDashboard.putBoolean("Shooter/UseAutoFlywheel", useAutoFlywheel);
      SmartDashboard.putBoolean("Ready/Can Trech (Hood)", this.isHoodCorrect());
    }
    debounceCount++;

  }
}
