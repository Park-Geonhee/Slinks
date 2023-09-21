package com.nemo.neplan.service;

import com.nemo.neplan.model.Plan;

import java.util.List;

public interface PlanService {
    Plan getPlanById(long planId);
    List<Plan> getPlansByUserId(long userId);
    Plan createPlan(Plan plan);
    Plan updatePlan(Plan plan);
    void deletePlan(long planId);
    List<Plan> getAllPlans();

}
