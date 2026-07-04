package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurretSubsystem extends SubsystemBase {
    private double startAngle = 180;
    private final DcMotorEx turretMotor;

    private final double ticksPerDegree = 383.6 * (198.0 / 49.0) / 360.0;
    private final PIDController pid = new PIDController(-0.02, 0, 0);

    public TurretSubsystem(HardwareMap hm) {
        turretMotor = hm.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public double getAngle() {
        return 360 - (turretMotor.getCurrentPosition() / ticksPerDegree + startAngle);
    }

    public void setAngle(double angle) {
        startAngle += getAngle() - angle;
    }


    public void reset() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getAngularVelocity() {
        return turretMotor.getVelocity(AngleUnit.RADIANS) / (198.0 / 49.0);
    }

    public double getPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void stopPower() {
        setPower(0);
    }

    public void runToAngleDirect(double angle) {
        setPower(pid.calculate(getAngle(), angle));
    }


    public void setPower(double turretPower) {
        if (getAngle() > 340) {
            turretPower = Math.max(turretPower, 0);
        } else if (getAngle() < 20) {
            turretPower = Math.min(turretPower, 0);
        }
        turretMotor.setPower(turretPower);
    }
}
