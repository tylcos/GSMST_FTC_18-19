package org.firstinspires.ftc.team2981;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team2981.systems.*;

public class Robot {
    public RobotDrive drive;
    public Intake intake;

    public Robot(HardwareMap map){
        drive = new RobotDrive(map);
        intake = new Intake(map);
    }
}
