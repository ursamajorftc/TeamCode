package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear OpMode")

public class TeleOp extends LinearOpMode {

    private DcMotor intakeDrive = null;
    private Servo intakeServoLeft = null;
    private Servo intakeServoRight = null;
    private CRServo intakeCRSLeft = null;
    private CRServo intakeCRSRight = null;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeDrive  = hardwareMap.get(DcMotor.class, "intakeDrive");
        //intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));
//                if (gamepad1.a) {
//                    intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    intakeDrive.setTargetPosition(965);
//                    intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    intakeDrive.setPower(1);
//
//                }
//                if (gamepad1.b) {
//                    intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    intakeDrive.setTargetPosition(0);
//                    intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    intakeDrive.setPower(1);

//             }
                if (gamepad1.a) {
                    intakeServoLeft.setPosition(0);
                }



                drive.updatePoseEstimate();
                double LeftCurrentPosition = intakeServoLeft.getPosition();
                double RightCurrentPosition = intakeServoRight.getPosition();
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("intakePosition", intakeDrive.getCurrentPosition());
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.addData("check for explosion ", intakeDrive.getPower());
                telemetry.addData("intakeServoLeft", LeftCurrentPosition);
                telemetry.addData("intakeServoRight", RightCurrentPosition);
                telemetry.addData("CRSLeftPower", intakeCRSLeft.getPower());
                telemetry.addData("CRSRightPower", intakeCRSRight.getPower());

                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);


            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.addData("check for explosion ", intakeDrive.getPower());
                telemetry.addData("intakeServoLeft", intakeServoLeft.getPosition());
                telemetry.addData("intakeServoRight", intakeServoRight.getPosition());

                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }


}
