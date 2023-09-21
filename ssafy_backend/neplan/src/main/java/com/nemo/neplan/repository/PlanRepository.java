package com.nemo.neplan.repository;

import com.nemo.neplan.model.Plan;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;

public interface PlanRepository extends JpaRepository<Plan, Long>{
    List<Plan> findByUserId(long userId);
}
