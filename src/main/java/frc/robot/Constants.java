package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Operation {
        public static final int kDriverControllerPort = 0;
    }

    public static final class Drive {
        public static final double kSpeed = 0.3;
    }

    public static final class Motor {
        public static final int SMART_CURRENT_LIMIT = 30;
        public static final double POS_FACTOR = 1.0 / 13.5;
        public static final double VEL_FACTOR = POS_FACTOR / 60.0;
    }

    public static final class Auto {
        public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(13.5) / 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kPXController = 1.;
        public static final double kPYController = 1.;
        public static final double kPThetaController = 1.5;
    }
}
