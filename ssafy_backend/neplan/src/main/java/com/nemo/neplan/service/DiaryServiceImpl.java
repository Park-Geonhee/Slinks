package com.nemo.neplan.service;

import com.nemo.neplan.model.Diary;
import com.nemo.neplan.repository.DiaryRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.Optional;
@Service
public class DiaryServiceImpl implements DiaryService {

    private final DiaryRepository diaryRepository;

    @Autowired
    public DiaryServiceImpl(DiaryRepository diaryRepository) {
        this.diaryRepository = diaryRepository;
    }

    @Override
    public List<Diary> getAllDiaries() {
        return diaryRepository.findAll();
    }

    @Override
    public List<Diary> getDiariesByUserAndCreatedDate(Long userId, String created_date) {
        List<Diary> temp=diaryRepository.findByUserIdAndCreatedDate(userId, created_date);
        return temp;
    }

    @Override
    public List<Diary> getDiariesByUserAndPlace(Long userId, Long placeId) {
        List<Diary> temp=diaryRepository.findByUserIdAndPlaceId(userId, placeId);
        return temp;
    }


    @Override
    public Diary createDiary(Diary diary) {
        return diaryRepository.save(diary);
    }

    @Override
    public Diary updateDiary(Diary diary) {
        Diary existingDiary = diaryRepository.findById(diary.getId()).orElse(null);

        if (existingDiary != null) {
            existingDiary.setPlace(diary.getPlace());
            existingDiary.setContent(diary.getContent()); // 내용 변경 추가
            return diaryRepository.save(existingDiary);
        }

        return null; // 존재하지 않는 일기라면 예외 처리 혹은 null 반환
    }


    @Override
    public void deleteDiary(Long id) {
        diaryRepository.deleteById(id);
    }
}
