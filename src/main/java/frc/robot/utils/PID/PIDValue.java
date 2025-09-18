package frc.robot.utils.PID;

public class PIDValue {
        public double kP;
        public double kI;
        public double kD;

        public PIDValue(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }