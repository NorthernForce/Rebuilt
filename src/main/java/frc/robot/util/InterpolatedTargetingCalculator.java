package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import com.opencsv.CSVReader;
import com.opencsv.CSVWriter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotBase;

public class InterpolatedTargetingCalculator implements TargetingCalculator
{
    private InterpolatingDoubleTreeMap treeMap;
    private File file;
    private CSVReader csvReader;
    private CSVWriter csvWriter;

    public InterpolatedTargetingCalculator(String filePath)
    {
        treeMap = new InterpolatingDoubleTreeMap();
        if (!RobotBase.isSimulation())
        {
            file = new File(filePath);
            if (!file.exists())
            {
                try
                {
                    file.createNewFile();
                } catch (IOException e)
                {
                    e.printStackTrace();
                }
            }
            try
            {
                csvReader = new CSVReader(new FileReader(file));
                csvWriter = new CSVWriter(new FileWriter(file, true));
                csvReader.forEach(nextLine ->
                {
                    try
                    {
                        treeMap.put(Double.parseDouble(nextLine[0]), Double.parseDouble(nextLine[1]));
                    } catch (Exception e)
                    {
                        e.printStackTrace();
                    }
                });
            } catch (FileNotFoundException e)
            {
                e.printStackTrace();
            } catch (IOException e)
            {
                e.printStackTrace();
            } catch (NumberFormatException e)
            {
                e.printStackTrace();
            }
        }
    }

    @Override
    public double getValueForDistance(double distance)
    {
        distance = ((int) (distance * 100)) / 100.0;
        try
        {
            return treeMap.get(distance);
        } catch (Exception e)
        {
            e.printStackTrace();
        }
        return 0;
    }

    public void addData(double distance, double value)
    {
        distance = ((int) (distance * 100)) / 100.0;
        if (!RobotBase.isSimulation())
        {
            csvWriter.writeNext(new String[]
            { Double.toString(distance), Double.toString(value) });
            try
            {
                csvWriter.flush();
            } catch (IOException e)
            {
                e.printStackTrace();
            }
        }
        treeMap.put(distance, value);
    }
}