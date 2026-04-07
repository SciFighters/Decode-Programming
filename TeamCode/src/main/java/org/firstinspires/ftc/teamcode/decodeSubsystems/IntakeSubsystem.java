package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.sql.Time;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx motor;
    private final Servo intakeServo1, intakeServo2;
    public final DigitalChannel leftSwitch, rightSwitch;
    private Thread sensorThread;
    public static boolean reversed = false;
    public static int count = 0;
    volatile boolean running = true;

    public IntakeSubsystem(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "intakeMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeServo1 = hm.get(Servo.class, "rightIntake");
        intakeServo2 = hm.get(Servo.class, "leftIntake");
        leftSwitch = hm.get(DigitalChannel.class, "leftSwitch");
        rightSwitch = hm.get(DigitalChannel.class, "rightSwitch");
        leftSwitch.setMode(DigitalChannel.Mode.INPUT);
        rightSwitch.setMode(DigitalChannel.Mode.INPUT);
        reversed = false;
        running = true;;
        startSensorThread();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    private void startSensorThread() {
        sensorThread = new Thread(() -> {

//            ElapsedTime time = new ElapsedTime();
//            time.reset();
//            double rightSensorTime = 0, leftSensorTime = 0;
            boolean currentRight, currentLeft;
            boolean lastRight = false, lastLeft = false;
//            long cTime;
            while (running) {
//                cTime = time.time(TimeUnit.MILLISECONDS);
                currentRight = !rightSwitch.getState();
                currentLeft = !leftSwitch.getState();
                if ((currentRight && !lastRight)) {
                    count += 1;
//                    rightSensorTime = cTime;
                }
                if ((currentLeft && !lastLeft)) {
                    count += 1;
//                    leftSensorTime = cTime;
                }

                lastLeft = currentLeft;
                lastRight = currentRight;
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        });

        sensorThread.setDaemon(true);
        sensorThread.start();
    }

    public void stopSensorThread() {
        running = false;
    }

    public void setPosition(double position) {
        intakeServo1.setPosition(position);
        intakeServo2.setPosition(1 - position);
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }


}
