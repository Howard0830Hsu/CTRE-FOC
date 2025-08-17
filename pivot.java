package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class pivot extends SubsystemBase{
    private TalonFX right1 = new TalonFX(12);
    private TalonFX right2 = new TalonFX(11);
    private TalonFX left1 = new TalonFX(21);
    private TalonFX left2 = new TalonFX(22);
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    private MotionMagicVoltage m_mmReq =new MotionMagicVoltage(0)
    .withEnableFOC(true);
    boolean atSetPoint = false;   
    double tolerance = 0.005;
    public pivot() {
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;//方向錯了改成Clockwise_Positive

        right1.setControl(new Follower(left1.getDeviceID(), true));
        right2.setControl(new Follower(left1.getDeviceID(), true));
        left2.setControl(new Follower(left1.getDeviceID(), false));
        /* Configure current limits */
        MotionMagicConfigs mm = cfg.MotionMagic;
     
        mm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel 
        mm.MotionMagicJerk = 50;

        Slot0Configs slot0 = cfg.Slot0;
        
        slot0.kP = 5;//P
        slot0.kI = 0;//I
        slot0.kD = 0;//D
        slot0.kV = 15;
        slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 130;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = 130;
    
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 180;
        /* Retry config apply up to 5 times, report if failure */

        StatusCode status1 = StatusCode.StatusCodeNotInitialized;
        StatusCode status2 = StatusCode.StatusCodeNotInitialized;
        StatusCode status3 = StatusCode.StatusCodeNotInitialized;
        StatusCode status4 = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status1 = left1.getConfigurator().apply(cfg);
            status2 = left2.getConfigurator().apply(cfg);
           // status3 = right1.getConfigurator().apply(cfg);
            //status4 = right2.getConfigurator().apply(cfg);
        if (status1.isOK()  && status2.isOK()) break;
        }

        /* Make sure we start at 0 */
        left1.setPosition(0);
    } 
    public void periodic(){
        SmartDashboard.putNumber("pivot Position", left1.getPosition().getValueAsDouble());
    }

    public boolean atSetPoint(double goal){
        return Math.abs(goal-left1.getPosition().getValueAsDouble()) < tolerance;
    }

    public void stopElevator() {
        left1.set(0);
    }

    public void runPID(double position) {
        left1.setControl(m_mmReq.withPosition(position).withSlot(0));
    }
    public Command EX (){
        return startEnd(()->left1.set(0.1), ()->left1.set(0));
    }   
     public Command runCommand(double velocity) {
        return startEnd( () -> left1.set(velocity)
                        , () -> stopElevator() );
    }
    public Command Command(DoubleSupplier value) {
            return run(() -> left1.set(value.getAsDouble()));
    }
    public Command intakeCoral() {
        return run( () -> this.runPID(-0.035) )
                .until( () -> atSetPoint(-0.035))
                .andThen(() -> this.stopElevator());
    }
    public Command IntakeAlgaeGround() {
        return run( () -> this.runPID(-0.4) )
                .until( () -> atSetPoint(-0.4))
                .andThen(() -> this.stopElevator());
    }
    public Command Prepare() {
        return run( () -> this.runPID(-0.52) )
                .until( () -> atSetPoint(-0.52))
                .andThen(() -> this.stopElevator());
    }

    public Command Station() {
        return run( () -> this.runPID(-0.85) )
                .until( () -> atSetPoint(-0.85))
                .andThen(() -> this.stopElevator());
    }

         
}
