package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
    private VictorSPX leaderMotor;
    private VictorSPX followerMotor;
    private boolean hanging = false;

    public Hang() {
        leaderMotor = new VictorSPX(0);
        followerMotor = new VictorSPX(3);
        
        leaderMotor.setNeutralMode(NeutralMode.Brake);
        followerMotor.setNeutralMode(NeutralMode.Brake);

    }

    public void setSpeed1(double speed) {
        leaderMotor.set(ControlMode.PercentOutput, speed);
        followerMotor.set(ControlMode.PercentOutput, speed);
    }

    public void toggleHanging(){
        hanging=!hanging;
    }

    public double hanghizkatsayisi() {
        if (hanging){
            return 0.3;
        }
        else {
            return 1;
        }
    }

    public void stop() {
        leaderMotor.set(ControlMode.PercentOutput, 0);
        followerMotor.set(ControlMode.PercentOutput,0);
    }
}
