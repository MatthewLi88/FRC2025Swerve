package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * WristSubsystem controls the wrist joint of the robot arm.
 */
public class WristSubsystem extends PositionableSubsystem {
    private final SparkMax wrist;
    private static WristSubsystem self;

    private static final double LOWEST_POSITION = -400;
    private static final double HIGHEST_POSITION = -5;

    private WristSubsystem() {
        wrist = new SparkMax(Constants.ElevatorConstants.wristFrontCanId, MotorType.kBrushless);
        
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        // Set motor to brake mode and apply current limit
        wristConfig.idleMode(com.revrobotics.spark.config.SparkMaxConfig.IdleMode.kBrake);
        wristConfig.smartCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT_A);

        // Initialize parent PositionableSubsystem
        super.init(wrist);
        super.setMaxSpeed(Constants.IntakeConstants.WRIST_MAX_SPEED);

        // Optional: set absolute encoder usage
        // super.hasAbsEncoder(true);

        showPositionOnDashboard();
    }

    /** Singleton access */
    public static WristSubsystem getInstance() {
        if (self == null) self = new WristSubsystem();
        return self;
    }

    @Override
    public void move(double speed) {
        wrist.set(restrictSpeedForMinMax(speed));
    }

    @Override
    public void stop() {
        move(0);
    }

    /** Restrict movement speed based on min/max positions */
    public double restrictSpeedForMinMax(double speed) {
        double curPosition = getPosition();

        if (speed > 0 && curPosition >= HIGHEST_POSITION) return 0;
        if (speed < 0 && curPosition <= LOWEST_POSITION) return 0;

        return speed;
    }

    @Override
    public long getPosition() {
        return super.getPosition();
    }
}
