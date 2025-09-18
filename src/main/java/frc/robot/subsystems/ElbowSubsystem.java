package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * ElbowSubsystem controls the elbow joint of the robot arm.
 * Uses two SparkMax motors (front and back), with optional position safety limits.
 */
public class ElbowSubsystem extends PositionableSubsystem {
    private final SparkMax elbowf, elbowb;
    private static ElbowSubsystem self;

    private static final boolean APPLY_SAFETY = false;
    private static final int MotorDirectionForUP = 1;
    private static final double Lowest_Elbow_Position = -400;
    private static final double Highest_Elbow_Position = -5;

    private ElbowSubsystem() {
        // Initialize front and back elbow motors
        elbowf = new SparkMax(Constants.ElevatorConstants.elbowFrontCanId, MotorType.kBrushless);
        elbowb = new SparkMax(Constants.ElevatorConstants.elbowBackCanId, MotorType.kBrushless);

        SparkMaxConfig frontConfig = new SparkMaxConfig();
        SparkMaxConfig backConfig = new SparkMaxConfig();

        // Set brake mode and current limits
        frontConfig.idleMode(IdleMode.kBrake);
        frontConfig.smartCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT_A);

        // Configure back motor to follow front motor
        backConfig.follow(Constants.ElevatorConstants.elbowFrontCanId);
        backConfig.idleMode(IdleMode.kBrake);

        // Apply configurations or persist settings
        elbowf.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elbowb.configure(backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize PositionableSubsystem
        super.init(elbowf);
        super.setMaxSpeed(Constants.IntakeConstants.ELBOW_MAX_SPEED);
    }

    /** Singleton access method */
    public static ElbowSubsystem getInstance() {
        if (self == null) {
            self = new ElbowSubsystem();
        }
        return self;
    }

    /** Move elbow motor with speed, applying optional safety */
    @Override
    public void move(double speed) {
        elbowf.set(restrictSpeedForMinMax(speed));
    }

    /** Stop the elbow motor */
    @Override
    public void stop() {
        move(0);
    }

    /** Restrict speed to prevent exceeding min/max positions */
    public double restrictSpeedForMinMax(double speed) {
        if (!APPLY_SAFETY) {
            return speed;
        }

        int sign = (int) Math.signum(speed);
        double curPosition = getPosition();

        if (sign == MotorDirectionForUP && curPosition >= Highest_Elbow_Position) {
            speed = 0;
        }

        if (sign != MotorDirectionForUP && curPosition <= Lowest_Elbow_Position) {
            speed = 0;
        }

        return speed;
    }

    /** Get current elbow position */
    @Override
    public long getPosition() {
        return super.getPosition();
    }
}
