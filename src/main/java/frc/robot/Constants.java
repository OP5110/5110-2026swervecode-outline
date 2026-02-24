package frc.robot;

public final class Constants {

    private Constants() {}

    public static final class CanBus {
        public static final String RIO = "rio";

        private CanBus() {}
    }

    public static final class CanIds {
        public static final int INTAKE_ROLLER = 20;
        public static final int INTAKE_ARM = 21;
        public static final int CONVEYOR = 22;
        public static final int SHOOTER_LOADER = 23;
        public static final int SHOOTER_FLYWHEEL = 24;
        public static final int CLIMBER_MOTOR = 30;

        public static final int PIGEON2 = 13;

        public static final int FRONT_LEFT_DRIVE = 5;
        public static final int FRONT_LEFT_STEER = 4;
        public static final int FRONT_LEFT_ENCODER = 6;

        public static final int FRONT_RIGHT_DRIVE = 8;
        public static final int FRONT_RIGHT_STEER = 7;
        public static final int FRONT_RIGHT_ENCODER = 9;

        public static final int BACK_LEFT_DRIVE = 2;
        public static final int BACK_LEFT_STEER = 1;
        public static final int BACK_LEFT_ENCODER = 3;

        public static final int BACK_RIGHT_DRIVE = 11;
        public static final int BACK_RIGHT_STEER = 10;
        public static final int BACK_RIGHT_ENCODER = 12;

        private CanIds() {}
    }

    public static final class PwmPorts {
        public static final int BLINKIN = 1;
        public static final int CLIMBER_LATCH_SERVO = 2;

        private PwmPorts() {}
    }

    public static final class Climber {
        public static final double LATCH_ENGAGED_POSITION = 0.85;
        public static final double LATCH_DISENGAGED_POSITION = 0.15;
        public static final double LATCH_RELEASE_DELAY_SEC = 0.20;

        private Climber() {}
    }

    public static final class Vision {
        public static final class Front {
            public static final String LIMELIGHT_NAME = "limelight";

            private Front() {}
        }

        public static final class Climb {
            public static final String LIMELIGHT_NAME = "limelight-climb";
            public static final int BLUE_PIPELINE_INDEX = 0;
            public static final int RED_PIPELINE_INDEX = 1;

            private Climb() {}
        }

        private Vision() {}
    }
}
