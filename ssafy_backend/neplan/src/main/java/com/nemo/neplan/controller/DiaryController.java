package com.nemo.neplan.controller;

import com.nemo.neplan.model.Diary;
import com.nemo.neplan.service.DiaryService;
import io.swagger.annotations.Api;
import io.swagger.annotations.ApiOperation;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.format.annotation.DateTimeFormat;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.time.LocalDateTime;
import java.util.List;
@RestController
@Api(tags = "Diary 관련 API", description = "일기 관리 API")
@RequestMapping("/diaries")
public class DiaryController {

    private final DiaryService diaryService;

    @Autowired
    public DiaryController(DiaryService diaryService) {
        this.diaryService = diaryService;
    }

    @GetMapping
    public ResponseEntity<List<Diary>> getAllDiaries() {
        List<Diary> diaries = diaryService.getAllDiaries();
        return ResponseEntity.ok(diaries);
    }

    @GetMapping("/user/{userId}/date/{date}")
    public ResponseEntity<List<Diary>> getDiariesByUserAndDateRange(
            @PathVariable Long userId,
            @PathVariable @DateTimeFormat(pattern = "yyyy-MM-dd") String date) {
        List<Diary> diaries = diaryService.getDiariesByUserAndCreatedDate(userId, date);
        return ResponseEntity.ok(diaries);
    }

    @GetMapping("/user/{userId}/place/{placeId}")
    public ResponseEntity<List<Diary>> getDiariesByUserAndPlace(
            @PathVariable Long userId,
            @PathVariable Long placeId) {
        // userId와 placeId를 이용하여 User와 Place 객체 가져오기
        List<Diary> diaries = diaryService.getDiariesByUserAndPlace(userId, placeId);
        return ResponseEntity.ok(diaries);
    }

    @PostMapping("/add")
    public ResponseEntity<Diary> createDiary(@RequestBody Diary diary) {
        Diary createdDiary = diaryService.createDiary(diary);
        return ResponseEntity.status(HttpStatus.CREATED).body(createdDiary);
    }

    @PutMapping("/edit/{id}")
    public ResponseEntity<Diary> updateDiary(
            @PathVariable Long id,
            @RequestBody Diary diary) {
        Diary updatedDiary = diaryService.updateDiary(diary);
        return ResponseEntity.ok(updatedDiary);
    }

    @DeleteMapping("/delete/{id}")
    public ResponseEntity<Void> deleteDiary(@PathVariable Long id) {
        diaryService.deleteDiary(id);
        return ResponseEntity.noContent().build();
    }
}
