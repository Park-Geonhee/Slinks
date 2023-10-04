package com.nemo.neplan.service;


import com.nemo.neplan.repository.FileRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.core.io.Resource;
import org.springframework.core.io.UrlResource;
import org.springframework.stereotype.Service;
import org.springframework.util.StringUtils;
import org.springframework.web.multipart.MultipartFile;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.MalformedURLException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.List;
import java.util.UUID;


@Service
public class FileServiceImpl implements FileService {

    @Autowired
    private FileRepository fileRepository;

    @Override
    public Long uploadFiles(MultipartFile file) throws IOException {
        // 랜덤한 파일 이름 생성
        String fileName = StringUtils.cleanPath(file.getOriginalFilename());
        String extension = fileName.substring(fileName.lastIndexOf("."));

        UUID uuid = UUID.randomUUID();
        String storedFileName = uuid.toString() + extension;

        // 파일을 저장할 경로 설정
        Path uploadPath = Path.of("/usr/local/lib/upload-dir/" + storedFileName);

        // 파일 처리 및 리소스 자동 닫기
        try (var inputStream = file.getInputStream()) {
            Files.copy(inputStream, uploadPath, StandardCopyOption.REPLACE_EXISTING);
        }

        // 데이터베이스에 파일 정보 저장
        com.nemo.neplan.model.File uploadedFile = new com.nemo.neplan.model.File(storedFileName, fileName, uploadPath.toString());
        fileRepository.save(uploadedFile);


        // 새로 생성된 파일의 ID 값을 반환
        return uploadedFile.getId();
    }

    @Override
    public void deleteFile(String filePath) throws Exception {

        //파일 저장 경로를 이용해서 파일 객체 생성
        File deleteFile=new File(filePath);

        if(deleteFile.exists()){
            deleteFile.delete();
           System.out.println("파일 삭제 완료");
        }
        else{
            System.out.println("파일이 없습니다.");
        }
    }

    @Override
    public com.nemo.neplan.model.File getFileById(long id) {
        return fileRepository.getReferenceById(id);
    }
}
