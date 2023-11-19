package org.firstinspires.ftc.teamcode.util;

public class ErrorCalc {
    public double Error(double accepted, double experimental){
        return (Math.abs(accepted-experimental))/accepted;
    }
}
