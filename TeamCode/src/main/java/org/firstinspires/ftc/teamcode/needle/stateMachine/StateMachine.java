package org.firstinspires.ftc.teamcode.needle.stateMachine;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

public class StateMachine {
    GamepadEx driver, system;
    SubsystemBase telescopicArmSubsystem, armAxisSubsystem, clawSubsystem;

    public StateMachine(GamepadEx driver, GamepadEx system, TelescopicArmSubsystem telescopicArmSubsystem, ArmAxisSubsystem armAxisSubsystem, ClawSubsystem clawSubsystem) {
        this.driver = driver;
        this.system = system;
        this.telescopicArmSubsystem = telescopicArmSubsystem;
        this.armAxisSubsystem = armAxisSubsystem;
        this.clawSubsystem = clawSubsystem;
    }

}
