package com.nemo.neplan.repository;

import com.nemo.neplan.model.Plan;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
public interface PlanRepository extends JpaRepository<Plan, Long>{
    List<Plan> findByUserId(long userId);
}
