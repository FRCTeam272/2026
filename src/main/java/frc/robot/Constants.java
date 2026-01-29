package frc.robot;

import frc.lib.utils.PIDSettings;

public class Constants {
    public static final PIDSettings SHOOTER_PID_SETTINGS = new PIDSettings(
        0.0002, 0.0, 0.0,
        0.1, .02
    );

    public static final PIDSettings CONVEYOR_PID_SETTINGS = new PIDSettings(
        0.001, 0.0, 0.0
    );

    public static final PIDSettings REGULATOR_PID_SETTINGS = new PIDSettings(
        0.001, 0.0, 0.0
    );
}
