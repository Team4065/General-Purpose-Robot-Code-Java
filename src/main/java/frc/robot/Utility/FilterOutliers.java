// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Arrays;

/** Add your docs here. */
public class FilterOutliers {

    static Double[][] filterVelocityFeedForwardMeasurements(Double[] x,  Double[] y){
        return filterVelocityFeedForwardMeasurements((ArrayList<Double>)Arrays.asList(x), (ArrayList<Double>)Arrays.asList(y));
    }

    static Double[][] filterVelocityFeedForwardMeasurements(ArrayList<Double> x,  ArrayList<Double> y){
        ArrayList<Double> outliers = (ArrayList<Double>)getOutliers(y);
        //ArrayList<Double> x = (ArrayList<Double>)_x;
        //ArrayList<Double> y = (ArrayList<Double>)_y;

        System.out.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        System.out.println(outliers);
        System.out.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

        System.out.println(y.getClass().getName());

        for(int i = 0; i < outliers.size(); ++i){
            int indexOfOutlier = y.indexOf(outliers.get(i));
            y.remove(indexOfOutlier);
            x.remove(indexOfOutlier);
        }

        /*
        for(int i = 0; i < x.size(); ++i){
            System.out.printf("%.5f", x.get(i));
            System.out.print(",");
            System.out.printf("%.5f", y.get(i));
            System.out.println("");
        }
        */

        return new Double[0][0];
    }

    

    public static List<Double> getOutliers(List<Double> input) {
        List<Double> output = new ArrayList<Double>();
        List<Double> data1 = new ArrayList<Double>();
        List<Double> data2 = new ArrayList<Double>();

        if (input.size() % 2 == 0) {
            data1 = input.subList(0, input.size() / 2);
            data2 = input.subList(input.size() / 2, input.size());
        } else {
            data1 = input.subList(0, input.size() / 2);
            data2 = input.subList(input.size() / 2 + 1, input.size());
        }
        double q1 = getMedian(data1);
        double q3 = getMedian(data2);
        double iqr = q3 - q1;
        double lowerFence = q1 - 1.5 * iqr;
        double upperFence = q3 + 1.5 * iqr;
        for (int i = 0; i < input.size(); i++) {
            if (input.get(i) < lowerFence || input.get(i) > upperFence)
                output.add(input.get(i));
        }
        return output;
    }

    private static double getMedian(List<Double> data) {
        if (data.size() % 2 == 0)
            return (data.get(data.size() / 2) + data.get(data.size() / 2 - 1)) / 2;
        else
            return data.get(data.size() / 2);
    }
}
