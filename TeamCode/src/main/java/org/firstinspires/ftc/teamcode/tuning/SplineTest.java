package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    public Pose2d corner(int angle){
        return new Pose2d(-6, -44, Math.toRadians(angle));
    }
    @Override
    public void runOpMode() throws InterruptedException {
        double pi = Math.PI;


        Pose2d beginPose = new Pose2d(-33, -62, Math.toRadians(0));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(33, -62, Math.toRadians(0)))
                            .splineToLinearHeading(corner(45), -pi/4)
                            .waitSeconds(1)

                            //code to drop off sample (top bucket)


//                        //face first sample
//                        .splineToLinearHeading(corner(76.79), -pi/8)
//                        .waitSeconds(1)
//
//                        //grab
//
//
//                        //rotate to bucket
//                        .splineToLinearHeading(new Pose2d(corner(45)), -pi/8)
//                        .waitSeconds(1)
//
//                        //drop sample
//
//
//                        //face second sample
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(93.98)), -pi/8)
//                        .waitSeconds(1)
//
//                        //grab
//
//
//                        //rotate to bucket
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), -pi/8)
//                        .waitSeconds(1)
//
//                        //drop sample
//
//
//                        //face third sample
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(111.345)), -pi/8)
//                        .waitSeconds(1)
//
//                        //grab
//
//
//                        //face bucket
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), -pi/8)
//
//                        //drop sample
//
//
//                        //park
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(100)), -pi/8)
//                        .splineTo(new Vector2d(-25, -11.5), 0)
                            .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
