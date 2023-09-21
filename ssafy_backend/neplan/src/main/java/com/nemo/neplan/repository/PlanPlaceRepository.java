package com.nemo.neplan.repository;

import com.nemo.neplan.model.Plan;
import com.nemo.neplan.model.PlanPlace;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;

public interface PlanPlaceRepository extends JpaRepository<PlanPlace, Long> {

    List<PlanPlace> findByPlanId(Long id);
    List<PlanPlace> findByPlaceId(Long id);
    int countByPlanId(Long planId);

}
