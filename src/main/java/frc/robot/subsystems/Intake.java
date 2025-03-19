package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.Arm.ArmState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
    private SparkMax intakeMotor;
    private DigitalInput limitswitchcoral;
    private boolean isRunningCoralIntake = false;
    private LED m_led;
    private Arm arm;



    public Intake(LED led, Arm arm) {
        intakeMotor = new SparkMax(Constants.INTAKE_MOTOR_ID,MotorType.kBrushless);
        limitswitchcoral = new DigitalInput(Constants.INTAKECORAL_LIMIT_SWITCH_ID);
        this.m_led = led;
        this.arm= arm;
    }

    public void ToggleCoralIntake(){
        if (isRunningCoralIntake) {
            stopIntake();
        } else if (!isCoralDetected()){
            startCoralIntake();
        }
    }

    

    public void startCoralIntake() {
        if (!isCoralDetected()) {
            intakeMotor.set(-0.2);
            isRunningCoralIntake = true;
        } 
    }



    
    public void AlgIntake(double speed){
        System.out.println("AlgIntake Calisiyor");
        intakeMotor.set(0.8);

    }



    public void stopIntake(){
        intakeMotor.set(0);
        isRunningCoralIntake = false;
    }
    
    
    public void CoralIOandAlgOuttake(double speed){
        System.out.println("CoralIOandAlgOuttake Calisiyor");
        intakeMotor.set(-0.8);
    }


    public boolean isCoralDetected(){
        return !limitswitchcoral.get();
    }



    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral LS", isCoralDetected());
        if (isRunningCoralIntake && isCoralDetected()) {
            stopIntake();
            if (arm != null){
                new ArmCommand(arm, ArmState.L1_L2_L3ARM).schedule();
            }
            

        }
        if (!m_led.isBlueOn()){
            if (isCoralDetected()) {
                m_led.setGreen(); // Coral içerideyse yeşil
            } else if (isRunningCoralIntake) {
                m_led.blinkRed(); // Coral alınırken kırmızı yanıp sönsün
            } else {
                m_led.setRed(); // Boştayken kırmızı yanar
            }
        }
        
    }
}
