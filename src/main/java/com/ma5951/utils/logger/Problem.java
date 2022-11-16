package com.ma5951.utils.logger;

import java.util.function.Supplier;

public class Problem {
    private String problemName;
    private Supplier<Boolean> problem;

    public Problem(String problemName, Supplier<Boolean> problem) {
        this.problemName = problemName;
        this.problem = problem;
    }

    public String getProblemName() {
        return problemName;
    }

    public boolean getProblem() {
        return problem.get();
    }
}