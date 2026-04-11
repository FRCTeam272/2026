package frc.robot;

import java.util.function.DoubleSupplier;

import frc.lib.utils.PIDSettings;
import frc.lib.utils.ShooterProfile;

public class Constants {
    public static final PIDSettings SHOOTER_LOW_PID_SETTINGS = new PIDSettings(
        0.00001, 0.0, 0.0,
        0.00195, 0.00001, 3600, .25 // 1
    );

    public static final PIDSettings SHOOTER_MID_PID_SETTINGS = new PIDSettings(
        0.00025, 0.0, 0.0,
        0.00195, 0.00001,  4700, 1.5 // 4.8
    );

    public static final PIDSettings SHOOTER_HIGH_PID_SETTINGS = new PIDSettings(
        0.0009, 0, 0,
        0.00195, 0.00001, 5500, 5.5 // 6
    );

    public static final PIDSettings SHOOTER_4150_PID_SETTINGS = new PIDSettings( // note this is the same as LOW
        0.00001, 0.0, 0.0,
        0.00195, 0.00001, 4150, 0
    );

    public static final PIDSettings SHOOTER_4700_PID_SETTINGS = new PIDSettings(
        0.00025, 0.0, 0.0,
        0.00195, 0.00001, 4700, 0
    );

    public static final PIDSettings SHOOTER_AUTO_PID_SETTINGS = SHOOTER_MID_PID_SETTINGS;
    // new PIDSettings( // note this is the same as LOW
    //     .0005, 0.0, 0.0,
    //     0.00195, 0.00001, 5100, 0
    // );

    public static final PIDSettings SHOOTER_5500_PID_SETTINGS = new PIDSettings( // note this is the same as LOW
        0.0009, 0.0, 0.0,
        0.00195, 0.00001, 5500, 0
    );

    public static final PIDSettings SHOOTER_5200_PID_SETTINGS = new PIDSettings( // note this is the same as LOW
        0.0009, 0.0, 0.0,
        0.00195, 0.00001, 5200, 0
    );

    public static final PIDSettings SHOOTER_4900_PID_SETTINGS = new PIDSettings( // note this is the same as LOW
        0.0004, 0.0, 0.0,
        0.00195, 0.00001, 4900, 0
    );

    public static final PIDSettings[] autoFlywheelSettings = new PIDSettings[] {
        SHOOTER_4150_PID_SETTINGS,
        SHOOTER_4700_PID_SETTINGS,
        SHOOTER_AUTO_PID_SETTINGS,
        SHOOTER_5500_PID_SETTINGS
    };

    public static final PIDSettings CONVEYOR_PID_SETTINGS = new PIDSettings(
        0.001, 0.0, 0.0
    );

    public static final PIDSettings REGULATOR_PID_SETTINGS = new PIDSettings(
        0.001, 0.0, 0.0
    );

    public static final PIDSettings CLIMBER_PID_SETTINGS = new PIDSettings(
        0.15, 0.0, 0.0
    );

    public static final ShooterProfile _3600_SHOOTER_PROFILE = new ShooterProfile(
        Constants.SHOOTER_LOW_PID_SETTINGS,
        (distance) -> {
            if(distance < 20) return .25;
            return 0.4253 * distance - 10.116;
        },        
        0, // min distance
        35 // max distance
    );

    public static final ShooterProfile _4150_SHOOTER_PROFILE = new ShooterProfile(
        Constants.SHOOTER_4150_PID_SETTINGS,
        (distance) -> {
            return 0.1346 * distance - 3.5577;
        },        
        30, // min distance
        65 // max distance
    );

    public static final ShooterProfile _4700_SHOOTER_PROFILE = new ShooterProfile(
        Constants.SHOOTER_4700_PID_SETTINGS,
        (distance) -> {
            return 0.1249 * distance - 7.2002;
        },        
        58, // min distance
        98 // max distance
    );

    public static final ShooterProfile _4900_SHOOTER_PROFILE = new ShooterProfile(
        Constants.SHOOTER_4900_PID_SETTINGS,
        (distance) -> {
            return 0.1007 * distance - 8.4142;
        },        
        90, // min distance
        135 // max distance
    );

    public static final ShooterProfile _5200_SHOOTER_PROFILE = new ShooterProfile(
        Constants.SHOOTER_5200_PID_SETTINGS,
        (distance) -> {
            return 0.0833 * distance - 7.5;
        },        
        120, // min distance
        150 // max distance
    );

    public static final ShooterProfile[] SHOOTER_PROFILES = new ShooterProfile[] {
        _3600_SHOOTER_PROFILE,
        _4150_SHOOTER_PROFILE,
        _4700_SHOOTER_PROFILE,
        _4900_SHOOTER_PROFILE,
        _5200_SHOOTER_PROFILE
    };


}
