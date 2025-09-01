package org.firstinspires.ftc.teamcode.needle.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;

@TeleOp
public class AxisMinPowerFinder extends LinearOpMode {
    ArmAxisSubsystem armAxisSubsystem;
    ElapsedTime time = new ElapsedTime();
    double power = 0.06;
    boolean moved = false;

    @Override
    public void runOpMode() throws InterruptedException {
        armAxisSubsystem = new ArmAxisSubsystem(hardwareMap);
        waitForStart();
        time.reset();
        while (opModeIsActive()){
            armAxisSubsystem.setAxisPower(power);
            if(time.seconds() >= 0.5 && !moved) {
                power += 0.002;
                time.reset();
            }
            if(armAxisSubsystem.getAngle() >= 1 && !moved){
                moved = true;
                telemetry.addData("correctPower",power);
                telemetry.update();
            }
        }
    }
}
