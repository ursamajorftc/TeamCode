package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "PIDTest", group = "Linear OpMode")
public class PIDTest extends LinearOpMode {
	// motors declaration, we use the Ex version as it has velocity measurements
	DcMotorEx outmoto1;
	DcMotorEx outmoto2;
	PIDController pid = new PIDController(0.04, 0, 0.005);

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

		int highBasket = 1100;
		int lowBasket = 530;
		int downPosition = 0;

		// loop that runs while the program should run.
		while (opModeIsActive()) {
			if (gamepad1.dpad_up) {
				pid.setTargetPosition(highBasket);
			} else if (gamepad1.dpad_left) {
				pid.setTargetPosition(lowBasket);
			} else if (gamepad1.dpad_down) {
				pid.setTargetPosition(0);
			}

			double power = pid.getPower(outmoto1.getCurrentPosition());


			if (outmoto1.getCurrentPosition()> pid.getTargetPosition()) {
				outmoto1.setPower(-0.2);
				outmoto2.setPower(0.2);
			} else {
				outmoto1.setPower(power);
				outmoto2.setPower(-power);
			}

			double currentPosition = outmoto1.getCurrentPosition();
			telemetry.addData("Target Position", pid.getTargetPosition());
			telemetry.addData("Current Position", currentPosition);
			telemetry.addData("Error", highBasket - currentPosition);
			telemetry.addData("Integral", pid.getIntegral());
			telemetry.update();
		}
	}
}

