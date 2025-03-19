package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorTrapezoidProfile extends SubsystemBase {
    public enum ElevatorTrapezoidState {
        NOTRL1(-0.2), 
        L2(-3.5), 
        L3(-11.5),
        L4(-24),
        NET(-26.1);

        private final double targetHeight;
        ElevatorTrapezoidState(double targetHeight) { this.targetHeight = targetHeight; }
        public double getTargetHeight() { return targetHeight; }
    }

    private final SparkMax m_motorLeftLead;
    private final SparkMax m_motorRight;
    private final RelativeEncoder m_encoder;
    
    private static final double GEAR_RATIO = 30.0;  
    private static final double PULLEY_DIAMETER_CM = 5.0;
    private static final double POSITION_CONVERSION_FACTOR = ((Math.PI * PULLEY_DIAMETER_CM) / (GEAR_RATIO));
    private static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;
    private static final double MIN_HEIGHT = -26.7;
    private static final double MAX_HEIGHT = 0.0;

    private final SparkMaxConfig globalConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_motorLeftConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_motorRightConfig = new SparkMaxConfig();

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.ELEVATOR_TP[0],Constants.ELEVATOR_TP[1]);
    private final ProfiledPIDController controller = new ProfiledPIDController(Constants.ELEVATOR_PID[0], Constants.ELEVATOR_PID[1], Constants.ELEVATOR_PID[2], constraints);

    private ElevatorTrapezoidState currentState = ElevatorTrapezoidState.NOTRL1;

    public ElevatorTrapezoidProfile() {
        m_motorLeftLead = new SparkMax(Constants.ELEVATOR_MOTOR_IDS[1], MotorType.kBrushless);
        m_motorRight = new SparkMax(Constants.ELEVATOR_MOTOR_IDS[0], MotorType.kBrushless);

        globalConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).openLoopRampRate(0.2).voltageCompensation(12);
        m_motorLeftConfig.apply(globalConfig).inverted(false);
        m_motorLeftConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR).velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        m_motorRightConfig.apply(globalConfig).inverted(false).follow(m_motorLeftLead);

        m_encoder = m_motorLeftLead.getEncoder();
        m_motorLeftLead.configure(m_motorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorRight.configure(m_motorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetEncoder();
    }

    public void resetEncoder() { 
        m_encoder.setPosition(0); 
    }

    public void setHeight(double targetHeight) {
        double clampedHeight = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, targetHeight));
        controller.setGoal(clampedHeight);
    }

    public void setState(ElevatorTrapezoidState state) {
        this.currentState = state;
        setHeight(state.getTargetHeight());
    }

    public double getCurrentPosition() { 
        return m_encoder.getPosition(); 
    }
    public ElevatorTrapezoidState getCurrentState() {
        return currentState;
    }

    public void stop() {
        m_motorLeftLead.set(0);
    }

    public void calistir(double percent){
        m_motorLeftLead.set(percent);
    }

    public double swervehizikatsayisi() {
        if (currentState==ElevatorTrapezoidState.L2){
            return 0.7;
        }
        else if (currentState==ElevatorTrapezoidState.L3){
            return 0.5;
        }
        else if (currentState==ElevatorTrapezoidState.L4){
            return 0.4;
        }
        else if (currentState==ElevatorTrapezoidState.NET){
            return 0.4;
        }
        else{
            return 0.9;
        }
    }
    

    @Override
    public void periodic() {
        double currentPosition = m_encoder.getPosition();
        if (currentState.getTargetHeight()==-0.2 && Math.abs(currentState.getTargetHeight()-currentPosition)<0.6){
            stop();
        }
        else if (!controller.atGoal()) {
            double pidOutput = controller.calculate(currentPosition);
            double ffOutput = -0.05;
            double output = pidOutput + ffOutput;
            if (currentState.getTargetHeight()>currentPosition){
                output = Math.max(-0.4, Math.min(0.4, output));    
            }
            else{
                output = Math.max(-0.55, Math.min(0.55, output));
            }
            m_motorLeftLead.set(output);
        }
        else if (Math.abs(currentState.getTargetHeight()-currentPosition)<0.5){
            m_motorLeftLead.set(-0.05);
        }
        
        SmartDashboard.putNumber("Elevator Current Position", currentPosition);
        SmartDashboard.putString("Elevator Target Position", currentState.name());
        
    }

    @Override
    public void simulationPeriodic() {
        double appliedOutput = m_motorLeftLead.get();
        double simulatedPosition = getCurrentPosition() + (appliedOutput);
        m_encoder.setPosition(simulatedPosition);
//m_encoder.setPosition(currentState.getTargetHeight());
    }
}
