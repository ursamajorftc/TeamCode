package org.firstinspires.ftc.teamcode;

public class PIDController {

	private double kP;
	private double kI;
	private double kD;

	private double targetPoint;
	private double lastError;
	private double integral;
	private double smoothedError = 0;
	private double lastSmoothedError = 0;
	private final double MAX_INTEGRAL = 100;

	public PIDController(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	public double update(double targetPoint, double processVariable) {
		this.targetPoint = targetPoint;

		double error = targetPoint - processVariable;
		integral += error;
		integral = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integral));

		double smoothingFactor = 0.8; // Between 0 (no smoothing) and 1 (very smooth)
		smoothedError = (smoothingFactor * smoothedError) + ((1 - smoothingFactor) * error); // Smooth the error
		double derivative = smoothedError - lastSmoothedError; // Calculate derivative using smoothed values
		lastSmoothedError = smoothedError; // Update smoothed error for next cycle

		double rawValue = kP * error + kI * integral + kD * derivative;

		return Math.max(-1, Math.min(1, rawValue));
	}
	public double getIntegral () {return this.integral;}
}

