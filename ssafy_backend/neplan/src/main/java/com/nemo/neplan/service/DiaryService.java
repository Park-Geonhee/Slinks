package com.nemo.neplan.service;

import com.nemo.neplan.model.Diary;

import java.util.List;
public interface DiaryService {

    List<Diary> getAllDiaries();

    List<Diary> getDiariesByUserAndCreatedDate(Long userId, String created_date);

    List<Diary> getDiariesByUserAndPlace(Long userId, Long placeId);

    Diary createDiary(Diary diary);

    Diary updateDiary( Diary diary);

    List<Diary> getDiaryByUserId(Long userId);

    void deleteDiary(Long id);

}
