package frc.robot;

import frc.lib.utils.PIDSettings;

public class Constants {
    public static final PIDSettings SHOOTER_PID_SETTINGS = new PIDSettings(
        0.00003, 0.0, 0.0,
        0.00195, 0
    );

    public static final PIDSettings CONVEYOR_PID_SETTINGS = new PIDSettings(
        0.001, 0.0, 0.0
    );

    public static final PIDSettings REGULATOR_PID_SETTINGS = new PIDSettings(
        0.001, 0.0, 0.0
    );

    public static final PIDSettings CLIMBER_PID_SETTINGS = new PIDSettings(
        0.01, 0.0, 0.0
    );
}
