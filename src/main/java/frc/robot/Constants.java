package frc.robot;

import frc.lib.utils.PIDSettings;

public class Constants {
    public static final PIDSettings SHOOTER_LOW_PID_SETTINGS = new PIDSettings(
        0.00001, 0.0, 0.0,
        0.00195, 0.00001, 3900, 0
    );

    public static final PIDSettings SHOOTER_MID_PID_SETTINGS = new PIDSettings(
        0.0003, 0.0, 0.0,
        0.00195, 0.00001,  4700, 4
    );

    public static final PIDSettings SHOOTER_HIGH_PID_SETTINGS = new PIDSettings(
        0.0009, 0, 0,
        0.00195, 0.00001, 5500, 6
    );

    public static final PIDSettings SHOOTER_4150_PID_SETTINGS = new PIDSettings( // note this is the same as LOW
        0.00001, 0.0, 0.0,
        0.00195, 0.00001, 4150, 0
    );

    public static final PIDSettings SHOOTER_4700_PID_SETTINGS = new PIDSettings(
        0.00025, 0.0, 0.0,
        0.00195, 0.00001, 4700, 0)

    public static final PIDSettings SHOOTER_5200_PID_SETTINGS = new PIDSettings( // note this is the same as LOW
        0.0005, 0.0, 0.0,
        0.00195, 0.00001, 5200, 0
    );

    public static final PIDSettings SHOOTER_5500_PID_SETTINGS = new PIDSettings( // note this is the same as LOW
        0.0009, 0.0, 0.0,
        0.00195, 0.00001, 5500, 0
    );

    public static final PIDSettings[] autoFlywheelSettings = new PIDSettings[] {
        SHOOTER_4150_PID_SETTINGS,
        SHOOTER_4700_PID_SETTINGS,
        SHOOTER_5200_PID_SETTINGS,
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
}
