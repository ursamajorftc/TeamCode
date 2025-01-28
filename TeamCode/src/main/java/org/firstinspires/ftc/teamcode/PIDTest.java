package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "mainTeleOp", group = "Linear OpMode")
public class PIDTest extends LinearOpMode {

	// motors declaration, we use the
	// Ex version as it has velocity measurements
	DcMotorEx outmoto1;
	DcMotorEx outmoto2;
	PIDController control = new PIDController(0,0,0);
	@Override
	public void runOpMode() throws InterruptedException {
		// the string is the hardware map name
		outmoto1 = hardwareMap.get(DcMotorEx.class, "outmoto1");
		outmoto2 = hardwareMap.get(DcMotorEx.class, "outmoto2");

		// use braking to slow the motors down faster
		outmoto1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		outmoto2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

		// disables the default velocity control
		// this does NOT disable the encoder from counting,
		// but lets us simply send raw motors power.
		outmoto1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		outmoto2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		waitForStart();

		int targetPosition = 2300;

		// loop that runs while the program should run.
		while (opModeIsActive()) {
			double command = control.update(targetPosition, outmoto1.getCurrentPosition());

			if ((gamepad1.right_trigger > 0.25)) {
				outmoto1.setPower(0.5);
				outmoto2.setPower(-outmoto1.getPower());
			}
		}
	}
}

