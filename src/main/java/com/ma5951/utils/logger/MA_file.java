package com.ma5951.utils.logger;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class MA_file {
    private File file;
    private FileWriter fileWriter;

    public MA_file(String fileName) {
        file = new File(fileName);
        try {
            file.createNewFile();
            fileWriter = new FileWriter(fileName);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void wirte(String value) {
        try {
            fileWriter.write(value);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void close() {
        try {
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
