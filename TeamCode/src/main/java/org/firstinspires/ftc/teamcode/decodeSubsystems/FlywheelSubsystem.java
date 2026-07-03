package org.firstinspires.ftc.teamcode.decodeSubsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.BangBangController;

public class FlywheelSubsystem extends SubsystemBase {
    private final double gearRatio = 37.0 / 37; // 34:30
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(0.07, 0.00016305959, 0);
    private final PIDController flywheelPID = new PIDController(0.00133333, 0.000005, 0);
    private final BangBangController bangbang = new BangBangController(200);

    public final MotorEx flyWheelMotor;

    public FlywheelSubsystem(HardwareMap hm) {
        flyWheelMotor = new MotorEx(hm, "flyWheelMotor", Motor.GoBILDA.BARE);
        flyWheelMotor.encoder.setDirection(Motor.Direction.REVERSE); // for testing 2.7.26
    }

    public void setFlyWheelPower(double flyWheelPower) {
        flyWheelMotor.set(flyWheelPower);
    }

    public double getPower() {
        return flyWheelMotor.motorEx.getPower();
    }
    public void setFlyWheelRPM(double rpm) {
        if (abs(rpm) < 200) {
            flyWheelMotor.set(0);
            return;
        }

        double currentRPM = getRPM();

        if (bangbang.isAtSetpoint(currentRPM, rpm)) { // switching to pid for close range-error
            flyWheelMotor.set(calculateFlywheelCloseLoopOutput(currentRPM, rpm));
        } else {
            flyWheelMotor.set(bangbang.calculate(currentRPM, rpm));
        }
    }


    private double calculateFlywheelCloseLoopOutput(double currentRPM, double setpoint) {
        return flywheelPID.calculate(currentRPM, setpoint) + flywheelFeedforward.calculate(setpoint);
    }

    public void stayRPM(double rpm) {
        flyWheelMotor.set(calculateFlywheelCloseLoopOutput(getRPM(), rpm));
    }

    public double getRPM() {
        return -flyWheelMotor.getCorrectedVelocity() / flyWheelMotor.getCPR() * 60 * gearRatio;
    }

    public double getFlyWheelPower() {
        return flyWheelMotor.get();
    }
}
