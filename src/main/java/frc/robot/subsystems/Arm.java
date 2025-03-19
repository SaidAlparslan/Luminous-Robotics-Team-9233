package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final SparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final ProfiledPIDController pidController;
    private boolean manualOverride = false;
    private ArmState targetState;

    public enum ArmState {
        NOTRINTAKEARM(-3),
        L1_L2_L3ARM(-15),
        L4ARM(-20.800),
        NETARM(-75),
        PROCESSORARM(-127.000);


        private final double position;

        ArmState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    private static final double POSITION_CONVERSION_FACTOR = 10; 
    private static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0; 

    // Trapezoid Profil Sınırları
    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(Constants.ARM_TP[0],Constants.ARM_TP[1]);

    public Arm() {
        armMotor = new SparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                      .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);
        
        pidController = new ProfiledPIDController(Constants.ARM_PID[0],Constants.ARM_PID[1],Constants.ARM_PID[2], constraints);
        pidController.setTolerance(1.0);

        targetState = ArmState.NOTRINTAKEARM;
    }

    public void setTargetState(ArmState state) {
        manualOverride = false;
        targetState = state;
        pidController.setGoal(state.getPosition());
    }

    public ArmState getTargetState() {
        return targetState;
    }

    public double getCurrentPosition() {
        return armEncoder.getPosition();
    }

    @Override
    public void periodic() {
        double currentPosition = getCurrentPosition();
        SmartDashboard.putString("Arm State", targetState.name());
        SmartDashboard.putNumber("Arm Encoder (Degrees)", currentPosition);
        SmartDashboard.putNumber("Arm Target Position", targetState.getPosition());
        if (!manualOverride) {
            if (targetState.getPosition()==-1&&Math.abs(targetState.getPosition()-currentPosition)<0.5){
                stop();
            }
            if (targetState.getPosition()==-125&&Math.abs(targetState.getPosition()-currentPosition)<0.5){ 
                stop();
            }
            else if (Math.abs(targetState.getPosition() - currentPosition) > 0.5){
            double pidOutput = pidController.calculate(currentPosition);
            pidOutput = Math.max(-0.45, Math.min(0.45, pidOutput));
            armMotor.set(pidOutput);
            }
        }
    }

    public void manualMove(double speed) {
        manualOverride = true;
        armMotor.set(speed);
    }

    public void calistir(double speed){
        armMotor.set(speed);
    }

    public void stop() {
        manualOverride = false;
        armMotor.set(0);
    }
    @Override
    public void simulationPeriodic() {armEncoder.setPosition(targetState.getPosition());}
}
