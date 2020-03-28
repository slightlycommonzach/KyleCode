package org.firstinspires.ftc.teamcode;

/*
 * NOTE: Used in order to interact with Telemetry for
 * classes that are not OpModes/Cant Multiple Inherit
 * TODO: Look at Compositional Inheritance
 */
public class Mediator extends KyleSight {

    public <T> void handleTelemetry(String str, T x)   {
        telemetry.addData(str, x);
    }
}