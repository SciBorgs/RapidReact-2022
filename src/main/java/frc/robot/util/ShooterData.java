package frc.robot.util;

import java.util.ArrayList;

/**
 * methods to help with loading and interpolating between shooter data points
 * this will only be used if a quartic regression is unsuccessful
 */
public class ShooterData {
    
    // Converts CSV data into 2D array of doubles to pass into calcHoodAngle()
    public static double[][] getHoodAngleData(String fName) {
        ArrayList<String[]> lines = CSVUtils.newReader(fName).readData();
        double[][] data = new double[lines.size()-1][]; // Subtract 1 because we want to exclude the headers of CSV file
        for(int i = 1; i < lines.size(); i++) {
            double[] temp = { Double.valueOf(lines.get(i)[0]), Double.valueOf(lines.get(i)[1]) };
            data[i-1] = temp;
        }

        return data;
    }

    // Uses a 2D array of doubles (formatted as { angle, distance } for each point) and calculates
    public static double calcHoodAngle(double[][] data, double distance) {
        int entry1 = findClosest(data, distance);
        int entry2;

        if(distance < data[entry1][1]) {
            if(entry1 == 0) { // Special case for distances lower than the lowest logged distance
                entry2=entry1+1;
            }
            else {
                entry2 = entry1-1;
            }
        }
        else {
            if(entry1 == data.length-1) { // Special case for distances larger than the largest logged distance
                entry2 = entry1-1;
                
                // swapping entry points so initial value isn't incorrect (only for special case when a distance greater than last element in data array)
                int temp = entry1;
                entry1 = entry2;
                entry2 = temp;

            } 
            else {
                entry2 = entry1+1;
            }
        }

        return (data[entry2][0] - data[entry1][0]) / (data[entry2][1] - data[entry1][1]) + data[entry1][0];
    }

    // Helper function. Grabs index of the closest entry based on inputted distance value
    public static int findClosest(double arr[][], double target) {
        int n = arr.length;

        if (target <= arr[0][1])
            return 0;
        if (target >= arr[n - 1][1])
            return n - 1;

        int i = 0, j = n, mid = 0;
        while (i < j) {
            mid = (i + j) / 2;

            if (arr[mid][1] == target)
                return mid;

            if (target < arr[mid][1]) {

                if (mid > 0 && target > arr[mid - 1][1])
                    return getClosest(mid - 1,
                            mid, target, arr);

                j = mid;
            }

            else {
                if (mid < n - 1 && target < arr[mid + 1][1])
                    return getClosest(mid,
                            mid + 1, target, arr);
                i = mid + 1;
            }
        }

        return mid;
    }

    // Helper function for findClosest
    public static int getClosest(int val1, int val2, double target, double[][] data) {
        if (target - data[val1][1] >= data[val2][1] - target)
            return val2;
        else
            return val1;
    }
}
