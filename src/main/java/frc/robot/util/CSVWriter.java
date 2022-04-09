package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;

public class CSVWriter {
    private PrintWriter writer;

    public CSVWriter(String path) {
        try {
            this.writer = new PrintWriter(new FileOutputStream(
                    new File(path),
                    true /* append = true */));
        } catch (FileNotFoundException e) {
            this.writer = null;
        }

    }

    public void addData(Object... objects) {
        for (int i = 0; i < objects.length; i++) {
            this.writer.print(objects[i]);
            if (i != objects.length - 1) {
                this.writer.print(",");
            } else {
                this.writer.println();
            }
        }
        flush();
    }

    public void flush() {
        this.writer.flush();
    }

    public void close() {
        this.writer.close();
    }
}
