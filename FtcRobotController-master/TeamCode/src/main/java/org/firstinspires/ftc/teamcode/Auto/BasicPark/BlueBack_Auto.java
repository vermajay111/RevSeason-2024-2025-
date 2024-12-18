package org.firstinspires.ftc.teamcode.Auto.BasicPark;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto_New")
public class BlueBack_Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .strafeRight(110)
                .forward(110)
                .strafeLeft(110)
                .back(110)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);
        Pose2d poseEstimate = drive.getPoseEstimate();
        while (!isStopRequested() && opModeIsActive()) ;
    }
}
