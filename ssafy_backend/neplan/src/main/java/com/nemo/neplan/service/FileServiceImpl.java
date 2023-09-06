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

    @Override
    public String uploadFiles(String uploadPath, String originalName, byte[] fileData) throws Exception {

        //랜덤한 id값 생성
        UUID uuid=UUID.randomUUID();

        // .이후의 문자열은 제거
        String extension=originalName.substring(originalName.lastIndexOf("."));

        //랜덤 id값 + 파일 확장자
        String saveFileName=uuid.toString()+extension;
        //파일 업로드 경로= 업로드경로/랜덤 id값.파일 확장자
        String fileUploadFullUrl=uploadPath+"/"+saveFileName;

        //파일이 저장된 위치와 파일 이름을 파라미터로 넣어서 파일에 쓸 파일 출력 스트림 생성
        FileOutputStream fos=new FileOutputStream(fileUploadFullUrl);
        fos.write(fileData);
        fos.close();

        //saveFile(랜덤 id+파일 확장자) 반환
        return saveFileName;

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
}
