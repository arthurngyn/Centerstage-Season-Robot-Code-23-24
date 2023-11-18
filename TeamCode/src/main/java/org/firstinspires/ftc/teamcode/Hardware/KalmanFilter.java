package org.firstinspires.ftc.teamcode.Hardware;

public class KalmanFilter {
    private double estimate;  // Current estimate
    private double estimateError;  // Error in the estimate
    private double processNoise;  // Process noise (how much the system is expected to change)
    private double measurementNoise;  // Measurement noise (error in sensor readings)

    public KalmanFilter(double initialEstimate, double initialEstimateError,
                        double processNoise, double measurementNoise) {
        this.estimate = initialEstimate;
        this.estimateError = initialEstimateError;
        this.processNoise = processNoise;
        this.measurementNoise = measurementNoise;
    }

    public void update(double measurement) {
        // Prediction Step
        double predictedEstimate = estimate;
        double predictedEstimateError = estimateError + processNoise;

        // Update Step (Kalman Gain Calculation)
        double kalmanGain = predictedEstimateError / (predictedEstimateError + measurementNoise);

        // Update Estimate and Estimate Error
        estimate = predictedEstimate + kalmanGain * (measurement - predictedEstimate);
        estimateError = (1 - kalmanGain) * predictedEstimateError;
    }

    public double getEstimate() {
        return estimate;
    }

    public void setEstimate(double estimate) {
        this.estimate = estimate;
    }

    public double getEstimateError() {
        return estimateError;
    }

    public void setEstimateError(double estimateError) {
        this.estimateError = estimateError;
    }
}

