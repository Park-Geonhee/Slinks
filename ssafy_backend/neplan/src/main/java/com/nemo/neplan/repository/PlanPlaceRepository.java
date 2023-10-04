package com.nemo.neplan.repository;

import com.nemo.neplan.model.Plan;
import com.nemo.neplan.model.PlanPlace;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
public interface PlanPlaceRepository extends JpaRepository<PlanPlace, Long> {

    List<PlanPlace> findByPlanId(Long id);
    List<PlanPlace> findByPlaceId(Long id);
    int countByPlanId(Long planId);

}
