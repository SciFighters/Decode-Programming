package org.firstinspires.ftc.teamcode.actions.groups;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.actions.ActionBase;

public class ParallelActionGroup {
    public ParallelAction action;
    public ParallelCommandGroup command;
    public ParallelActionGroup(ActionBase... a){
        action = new ParallelAction(a);
        command = new ParallelCommandGroup(a);
    }
}
