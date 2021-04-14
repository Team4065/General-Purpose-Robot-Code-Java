// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import java.util.Vector;
import java.util.function.Supplier;


/** Add your docs here. */
public class Derivative {
    private static Vector<Derivative> derivatives = new Vector<Derivative>();

    Supplier<Double> m_getValue;
    double m_pastValue = 0;
    double m_derivative = 0;

    public Derivative(Supplier<Double> getValue){
        m_getValue = getValue;
        Derivative.derivatives.add(this);
    }

    public static void update(){
        for(Derivative d : derivatives){
            d.m_derivative = (d.m_getValue.get() - d.m_derivative) / (20. / 1000.);
            d.m_pastValue = d.m_getValue.get();
        }
    }

    public double get(){
        return m_derivative;
    }
}
