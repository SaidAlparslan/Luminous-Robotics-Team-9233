package frc.robot;

public class Constants {

    
    // Elevator Motor ID'leri
    public static final int[] ELEVATOR_MOTOR_IDS = {9, 12};// // Neo motorları

    // Intake Motor ID ve Limit Switch ID
    public static final int INTAKE_MOTOR_ID = 11;
    public static final int INTAKECORAL_LIMIT_SWITCH_ID = 0;//

    //Arm ID
    public static final int ARM_MOTOR_ID = 10;
    
    // PID Değerleri
    public static final double[] ELEVATOR_PID = {0.06, 0.0, 0.008};
    public static final double[] ELEVATOR_TP = {65,90};
    public static final double[] ARM_PID = {0.019, 0.0, 0.00005};
    public static final double[] ARM_TP = {70,100};

    
}
