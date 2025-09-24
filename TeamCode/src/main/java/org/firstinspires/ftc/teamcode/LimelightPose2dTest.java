package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.ejml.equation.Variable;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;


@Config
@TeleOp(group = "Limelight", name = "limelight Pose 2d")
public class LimelightPose2dTest extends LinearOpMode {
    public static Action traj1;
    private Limelight3A limelight;
    public static MecanumDrive drive;
    private double x_pose;
    private double y_pose;
    private double z_pose;
    private double robotYaw;
    public static boolean tagReceived = false;
    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Pose2d startPose = new Pose2d(0, 0, 0);


        drive = new MecanumDrive(hardwareMap, startPose);
        telemetry.setMsTransmissionInterval(50);
        tagReceived = false;
        drive.navxMicro.initialize();
        waitForStart();
        limelight.start();
        while(opModeIsActive()){
            drive.updatePoseEstimate();
            LLResult result = limelight.getLatestResult();
            if (result!= null){
                if (result.isValid() && !tagReceived ){
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        x_pose = metersToInches(fr.getRobotPoseFieldSpace().getPosition().x);
                        y_pose = metersToInches(fr.getRobotPoseFieldSpace().getPosition().y);
                        z_pose = metersToInches(fr.getRobotPoseFieldSpace().getPosition().z);
                        robotYaw = fr.getRobotPoseFieldSpace().getOrientation().getYaw(AngleUnit.DEGREES);

                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }
                }
                else{
                    telemetry.addLine("Result is invalid");
                }

                if(gamepad1.a){
                    tagReceived = true;
                    startPose = new Pose2d(x_pose, y_pose, toRadians(robotYaw));
                    drive = new MecanumDrive(hardwareMap, startPose);
                    buildTraj(startPose);
                    Actions.runBlocking(traj1);
                }
                else if (gamepad1.b){
                   tagReceived = false;
                }
            }
            else{
                telemetry.addLine("Result is null");
            }

            telemetry.addData("x_pose ", x_pose);
            telemetry.addData("y_pose ", y_pose);
            telemetry.addData("z_pose ", z_pose);
            telemetry.addData("drive x ", drive.pose.position.x);
            telemetry.addData("drive y ", drive.pose.position.y);
            telemetry.addData("drive yaw ", z_pose);
            telemetry.addData("robotYaw ", robotYaw);
            telemetry.addData("tagReceived ", tagReceived);
            telemetry.addData("heading ", Math.toDegrees(drive.pose.heading.toDouble()) );
            telemetry.update();
        }

    }

    public static double metersToInches(double meters) {
        return meters * 39.3701;
    }

    public static void buildTraj(Pose2d beginPose){
        traj1 = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(10, -47, toRadians(-168)), toRadians(0))
                .build();

    }

}
