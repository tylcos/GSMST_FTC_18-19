package org.firstinspires.ftc.team3819;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Brandaddy on 12/14/2017.
 */

@Autonomous(name="AutonPit")
public class AutonPit extends LinearOpMode {

    private Hardware robot = null;

    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void initialize() {
        robot = new Hardware(hardwareMap);

        //color = (ModernRoboticsI2cColorSensor) hardwareMap.get(ColorSensor.class, "color");
    }

    public void idler() {
        while (robot.isBusy() && opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Left: " + robot.left.getCurrentPosition());
            telemetry.addLine("Right: " + robot.right.getCurrentPosition());
            telemetry.update();
        }
        robot.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        robot.liftBack.setPower(.4);
        robot.liftFront.setPower(.4);

        waitForStart();

        robot.liftBack.setPower(.03);
        robot.liftFront.setPower(.03);
        waitCustom(5000);
        robot.liftFront.setPower(0);
        robot.liftBack.setPower(0); //lowers the bot
        waitCustom(1000);

        driveInches(.5, 24);
        driveInches(.5,-6);
        turn(90);
        driveInches(.5,38);
        turn(80);
        driveInches(.5, 42);
        robot.outtake();
        waitCustom(1000);
        robot.donttake();
        driveInches(.5,-6);
        turn(180);
        driveInches(.5,6*12);

    }

    public void driveInches(double pow, int in) {
        robot.resetEnc();
        int target = -1*(int)(in*robot.CPI);
        int dir = in >= 0 ? 1 : -1;
        robot.left.setPower(pow*dir);
        robot.right.setPower(pow*dir);
        while( (robot.left.getCurrentPosition()>target + 50||robot.left.getCurrentPosition()<target-50) &&
            (robot.right.getCurrentPosition()>target + 50||robot.right.getCurrentPosition()<target-50) &&
                opModeIsActive()) {
            telemetry.addLine("Left: " + robot.left.getCurrentPosition());
            telemetry.addLine("Right: " + robot.right.getCurrentPosition());
            telemetry.update();
        }
        robot.stop();
    }

    public void turn(double degrees) {
        robot.resetEnc();
        int powL = degrees >= 0 ? 1 : -1;
        int powR = powL * -1;

        int targetL = ((int) ((degrees / 360) * robot.CIRCUMFRENCE * robot.CPI)); //left gets a negative
        int targetR = targetL * -1;

        robot.left.setPower(powL*.25);
        robot.right.setPower(powR*.25);

        while ((robot.left.getCurrentPosition() > targetL + 10 || robot.left.getCurrentPosition() < targetL - 10) &&
                (robot.right.getCurrentPosition() > targetR + 10 || robot.right.getCurrentPosition() < targetR - 10) &&
                opModeIsActive()) {
            telemetry.addLine("Left: " + robot.left.getCurrentPosition());
            telemetry.addLine("Right: " + robot.right.getCurrentPosition());
            telemetry.update();
        }
        robot.stop();
    }

    public void slideUpEnc(){
        int target = 0;

        robot.slide.setPower(10);

        while( (robot.slide.getCurrentPosition()>target + 10||robot.slide.getCurrentPosition()<target-10) &&
            opModeIsActive()) {
            telemetry.addLine("Slide: " + robot.slide.getCurrentPosition());
            telemetry.update();
        }

    }

    public void slideDownEnc(){
        int target = robot.CPR;

        robot.slide.setPower(-50);

        while( (robot.slide.getCurrentPosition()>target + 10||robot.slide.getCurrentPosition()<target-10) &&
                opModeIsActive()) {
            telemetry.addLine("Slide: " + robot.slide.getCurrentPosition());
            telemetry.update();
        }

    }

    public void waitCustom(int ms) {
        time.reset();
        while(time.milliseconds()<ms) {}
    }

}