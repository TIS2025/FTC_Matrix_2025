package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class Matrix_Drive extends LinearOpMode {
    public static MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        waitForStart();
        while (opModeIsActive()) {
            drive.driveFieldCentric(-gamepad1.left_stick_x,  -gamepad1.left_stick_y , gamepad1.right_stick_x, drive.localizer.getPose().heading.toDouble());
            drive.updatePoseEstimate();
            if(gamepad1.right_stick_button){
                drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
                drive.updatePoseEstimate();
            }
            telemetry.addData("x", drive.localizer.getPose().position.x);
            telemetry.addData("y", drive.localizer.getPose().position.y);
            telemetry.addData("heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.update();
        }
    }
}
