package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class shoot extends SubsystemBase {
    private TalonFX shoot = new TalonFX(0);

public Command get(double speed){
   return startEnd(()->shoot.set(speed), ()->shoot.set(0)) ;
}
}