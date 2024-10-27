package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "RobotCentricMecnaum")
public class RobotCentricMecnaum extends LinearOpMode {
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        YawPitchRollAngles RobotOrientation;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        CRServo leftServo = hardwareMap.crservo.get("leftServo");
        CRServo rightServo = hardwareMap.crservo.get("rightServo");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        boolean safe_mode = true;
        double speedFactor = 1.0;
        while (opModeIsActive()) {
            double y = gamepad1.right_stick_y;
            double x = -gamepad1.right_stick_x;
            double rx = -gamepad1.left_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator);
            double backLeftPower = ((y - x + rx) / denominator);
            double frontRightPower = ((y - x - rx) / denominator);
            double backRightPower = ((y + x - rx) / denominator);

            if(gamepad1.a){

            }

            leftServo.setPower(0);
            rightServo.setPower(0);

            frontLeftMotor.setPower(-frontLeftPower * speedFactor);
            backLeftMotor.setPower(-backLeftPower * speedFactor);
            frontRightMotor.setPower(-frontRightPower * speedFactor);
            backRightMotor.setPower(-backRightPower * speedFactor);
            telemetry.update();
        }
    }
}