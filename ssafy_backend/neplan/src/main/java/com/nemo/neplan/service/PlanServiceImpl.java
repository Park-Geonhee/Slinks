package com.nemo.neplan.service;

import com.nemo.neplan.model.Plan;
import com.nemo.neplan.repository.PlanRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class PlanServiceImpl implements PlanService {
    private final PlanRepository planRepository;

    @Autowired
    public PlanServiceImpl(PlanRepository planRepository) {
        this.planRepository = planRepository;
    }

    @Override
    public Plan getPlanById(long planId) {
        return planRepository.findById(planId).orElse(null);
    }

    @Override
    public List<Plan> getPlansByUserId(long userId) {
        return planRepository.findByUserId(userId);
    }

    @Override
    public Plan createPlan(Plan plan) {
        return planRepository.save(plan);
    }

    @Override
    public Plan updatePlan(Plan plan) {
        return planRepository.save(plan);
    }

    @Override
    public void deletePlan(long planId) {
        planRepository.deleteById(planId);
    }

    @Override
    public List<Plan> getAllPlans() {
        return planRepository.findAll();
    }


}
