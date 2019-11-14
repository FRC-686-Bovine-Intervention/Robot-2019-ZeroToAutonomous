package frc.robot;

import frc.robot.lib.util.ConstantsBase;

public class Constants extends ConstantsBase {
    public static double kLoopDt = 0.01;
    public static double kDriveWatchdogTimerThreshold = 0.500;
    public static int kTalonTimeoutMs = 5; // ms
    public static int kTalonPidIdx = 0; // 0 for non-cascaded PIDs, 1 for cascaded PIDs
}