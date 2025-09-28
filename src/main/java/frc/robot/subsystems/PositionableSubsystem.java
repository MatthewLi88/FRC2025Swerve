package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class PositionableSubsystem extends SubsystemBase {
    private double currentSpeed = 0;
    private double maxSpeed = 1.0;

    private SparkAbsoluteEncoder aEncoder;
    private RelativeEncoder rEncoder;
    private PIDController pid;

    private final String name = getClass().getSimpleName().replace("Subsystem", "");
    private final String ABS_KEY = name + "_ABS";
    private final String REL_KEY = name + "_REL";
    private final String SPEED_KEY = name + " SPEED";
    private final String PIDKP_KEY = name + "_KP";
    private final String PIDKI_KEY = name + "_KI";
    private final String PIDKD_KEY = name + "_KD";

    private double currentKP = 0.0005;
    private double currentKI = 0;
    private double currentKD = 0;

    private DoubleLogEntry plogger, slogger;

    abstract void move(double speed);

    public abstract void stop();

    protected void init(SparkMax motor) {
        // Shuffleboard PID tab
        ShuffleboardTab pidTab = Shuffleboard.getTab("PID");
        pidTab.add(PIDKP_KEY, currentKP);
        pidTab.add(PIDKI_KEY, currentKI);
        pidTab.add(PIDKD_KEY, currentKD);

        rEncoder = motor.getEncoder();
        aEncoder = motor.getAbsoluteEncoder();

        pid = new PIDController(currentKP, currentKI, currentKD);

        DataLog log = DataLogManager.getLog();
        plogger = new DoubleLogEntry(log, name + "Pos");
        slogger = new DoubleLogEntry(log, name + "Spd");

        // Simulation

        showPositionOnDashboard();
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setPIDValues(double kP, double kI, double kD) {
        currentKP = kP;
        currentKI = kI;
        currentKD = kD;
        if (pid != null) {
            pid.setPID(kP, kI, kD);
        }
    }

    public void updatePIDValues() {
        double kPVal = SmartDashboard.getNumber(PIDKP_KEY, currentKP);
        double kIVal = SmartDashboard.getNumber(PIDKI_KEY, currentKI);
        double kDVal = SmartDashboard.getNumber(PIDKD_KEY, currentKD);
        if (kPVal != currentKP || kIVal != currentKI || kDVal != currentKD) {
            setPIDValues(kPVal, kIVal, kDVal);
        }
    }

    public void showPositionOnDashboard() {
        SmartDashboard.putNumber(ABS_KEY, getAbsPosition());
        SmartDashboard.putNumber(REL_KEY, getRelPosition());
        SmartDashboard.putNumber(SPEED_KEY, currentSpeed);
        logInfo();
    }

    private double getAbsPosition() {
        return aEncoder.getPosition();
    }

    private double getRelPosition() {
        return rEncoder.getPosition();
    }

    public long getPosition() {
        return Math.round(rEncoder.getPosition() * 1000);
    }

    protected void setCurrentSpeed(double speed) {
        currentSpeed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);
    }

    protected double getCurrentSpeed() {
        return currentSpeed;
    }

    public void logInfo() {
        if (plogger != null) plogger.append(getPosition());
        if (slogger != null) slogger.append(currentSpeed);
    }

    public Command moveCommand(DoubleSupplier speedSupplier) {
        return run(() -> {
            move(speedSupplier.getAsDouble());
            showPositionOnDashboard();
        });
    }

    public Command moveUp() {
        return run(() -> move(0.1));
    }

    public Command moveDown() {
        return run(() -> move(-0.1));
    }
}
