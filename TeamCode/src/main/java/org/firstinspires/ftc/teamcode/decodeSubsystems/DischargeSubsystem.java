package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DischargeSubsystem extends SubsystemBase {

    private final DcMotorEx turretMotor;
    public final MotorEx flyWheelMotor;
    private final Servo rampServo;
    private final double kS = 0.096561046, kV = 0.00016767951, kP = 0.000833333, kI = 0.000005;
    private final double ticksPerDegree = 383.6 * (198.0/49.0) / 360.0;
    private final double startAngle;
    double integral;

    public DischargeSubsystem(HardwareMap hm) {
        flyWheelMotor = new MotorEx(hm, "flyWheelMotor", Motor.GoBILDA.BARE);
        flyWheelMotor.encoder.setDirection(Motor.Direction.REVERSE);
        turretMotor = hm.get(DcMotorEx.class, "turretMotor");
        rampServo = hm.get(Servo.class, "rampServo");
        startAngle = 180;
        integral = 0;
    }
    public void resetTurret(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setFlyWheelPower(double flyWheelPower) {
        flyWheelMotor.set(flyWheelPower);
//        turretMotor.setPower(-flyWheelPower);
    }

    public void setFlyWheelRPM(double rpm) {
        if(Math.abs(rpm) < 200){
            flyWheelMotor.set(0);
//            turretMotor.setPower(0);
            return;
        }
        double currentRPM = -flyWheelMotor.getVelocity() / flyWheelMotor.getCPR() * 60;
        if(currentRPM < rpm){
            flyWheelMotor.set(-1);
//            turretMotor.setPower(1);
        }else{
            flyWheelMotor.set(-kS * Math.signum(rpm) - kV * rpm + 0.04);
//            turretMotor.setPower(kS * Math.signum(rpm) + kV * rpm - 0.03);
        }

    }

    public double getRPM() {
        return -flyWheelMotor.getCorrectedVelocity() / flyWheelMotor.getCPR() * 60;
    }
    public double getFlyWheelPower(){
        return flyWheelMotor.get();
    }

    public void setTurretPower(double turretPower) {
        if(getTurretAngle() > 332){
            turretPower = Math.max(turretPower,0);
        } else if (getTurretAngle() < 24) {
            turretPower = Math.min(turretPower,0);
        }
        turretMotor.setPower(turretPower);
    }

    public double getTurretPosition() {
        return turretMotor.getCurrentPosition();
//        return 0;
    }

    public double getTurretAngle() {
        return 360 - (turretMotor.getCurrentPosition() / ticksPerDegree + startAngle);
//        return 0;
    }

    public double getRPS() {
        return turretMotor.getVelocity(AngleUnit.RADIANS) / (198.0/49.0);
    }

    public void setRampDegree(double rampDegree) {
        double pos = (62.5 - rampDegree) / (42.12);
//        double pos = -(34.13 - rampDegree)/(38.87);
        rampServo.setPosition(Math.min(pos * 0.85, 0.85));
    }
}
