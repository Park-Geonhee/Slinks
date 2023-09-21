package com.nemo.neplan.controller;

import com.nemo.neplan.model.Plan;
import com.nemo.neplan.service.PlanService;
import io.swagger.annotations.Api;
import io.swagger.annotations.ApiOperation;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@Api(tags = "Plan 관련 API", description = "계획 관리 API")
@RequestMapping("/plans")
public class PlanController {

    @Autowired
    private PlanService planService;

    @GetMapping("/view/{planId}")
    @ApiOperation("계획 조회")
    public ResponseEntity<Plan> getPlan(@PathVariable long planId) {
        Plan plan = planService.getPlanById(planId);
        if (plan != null) {
            return ResponseEntity.ok(plan);
        } else {
            return ResponseEntity.notFound().build();
        }
    }

    @GetMapping("/user/{userId}")
    @ApiOperation("사용자 계획 조회")
    public ResponseEntity<List<Plan>> getUserPlans(@PathVariable long userId) {
        List<Plan> plans = planService.getPlansByUserId(userId);
        return ResponseEntity.ok(plans);
    }

    @GetMapping("/getAll")
    @ApiOperation(value = "모든 플랜 조회", notes = "모든 플랜을 조회합니다.")
    public ResponseEntity<List<Plan>> getAllPlans() {
        List<Plan> plans = planService.getAllPlans();
        return ResponseEntity.ok(plans);
    }

    @PostMapping("/add")
    @ApiOperation("계획 생성")
    public ResponseEntity<Plan> createPlan(@RequestBody Plan plan) {
        Plan createdPlan = planService.createPlan(plan);
        return ResponseEntity.status(HttpStatus.CREATED).body(createdPlan);
    }

    @PutMapping("/edit")
    @ApiOperation("계획 수정")
    public ResponseEntity<Plan> updatePlan(@RequestBody Plan plan) {
        Plan existingPlan = planService.getPlanById(plan.getId());
        if (existingPlan != null) {
            plan.setId(plan.getId());
            Plan updatedPlan = planService.updatePlan(plan);
            return ResponseEntity.ok(updatedPlan);
        } else {
            return ResponseEntity.notFound().build();
        }
    }

    @DeleteMapping("/delete/{planId}")
    @ApiOperation("계획 삭제")
    public ResponseEntity<Void> deletePlan(@PathVariable long planId) {
        Plan existingPlan = planService.getPlanById(planId);
        if (existingPlan != null) {
            planService.deletePlan(planId);
            return ResponseEntity.noContent().build();
        } else {
            return ResponseEntity.notFound().build();
        }
    }
}
