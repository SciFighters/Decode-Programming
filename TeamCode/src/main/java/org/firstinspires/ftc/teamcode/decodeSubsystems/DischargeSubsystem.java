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
    private final double kS = 0.0866319, kV = 0.000168344, kP = 0.000833333, kI = 0.000005;
    private final double ticksPerDegree = 383.6 * 3.96 / 360.0;
    final double startAngle;
    double integral;

    public DischargeSubsystem(HardwareMap hm) {
        flyWheelMotor = new MotorEx(hm,"flyWheelMotor", Motor.GoBILDA.BARE);
        turretMotor = hm.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampServo = hm.get(Servo.class, "rampServo");
        startAngle = 180;
        integral = 0;
    }

    public void setFlyWheelPower(double flyWheelPower) {
        flyWheelMotor.set(flyWheelPower);
    }
    public void setFlyWheelRPM(double rpm){
        double currentRPM = flyWheelMotor.getVelocity() / flyWheelMotor.getCPR() * 60;
        integral += (rpm - currentRPM) * kI;
        integral = Range.clip(integral,-0.1,0.1);
        flyWheelMotor.set(kS * Math.signum(rpm) + kV * rpm + kP * (rpm - flyWheelMotor.getVelocity() / flyWheelMotor.getCPR() * 60) + integral);
    }
    public double getRPM(){
        return flyWheelMotor.getVelocity() / flyWheelMotor.getCPR() * 60;
    }
    public void setTurretPower(double turretPower) {
        turretMotor.setPower(turretPower);
    }

    public double getTurretPosition() {
        return turretMotor.getCurrentPosition();
//        return 0;
    }
    public double getTurretAngle(){
        return 360 - (turretMotor.getCurrentPosition() / ticksPerDegree + startAngle);
//        return 0;
    }
    public double getRPS(){
        return turretMotor.getVelocity(AngleUnit.RADIANS) / 3.96;
    }
    public void setRampDegree(double rampDegree) {
        double pos = (73 - rampDegree)/(38.87);
//        double pos = -(34.13 - rampDegree)/(38.87);
        rampServo.setPosition(pos);
    }
}
