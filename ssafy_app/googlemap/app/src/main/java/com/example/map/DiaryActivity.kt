package com.example.map

import android.app.Activity
import android.content.ContentValues
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.drawable.BitmapDrawable
import android.net.Uri
import android.os.Build
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.os.Environment
import android.provider.MediaStore
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.ImageView
import android.widget.Toast
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import okhttp3.MediaType
import okhttp3.MultipartBody
import okhttp3.RequestBody
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStream
import java.net.URLEncoder
import java.text.SimpleDateFormat

class DiaryActivity : AppCompatActivity() {

    private lateinit var cameraBtn:Button
    private lateinit var makeDiary:Button
    private val PERMISSION_CAMERA = 1001 // 원하는 숫자로 권한 코드를 지정합니다.
    private val CAMERA = android.Manifest.permission.CAMERA // CAMERA 상수 정의

    private val REQUEST_IMAGE_CAPTURE = 1
    private lateinit var imagePreview: ImageView // 이미지를 표시할 ImageView 변수를 선언합니다.


    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_diary)

        // imagePreview 초기화
        imagePreview = findViewById(R.id.imagePreview) // R.id.imagePreview는 XML에서 정의한 ImageView의 ID입니다.


        // 카메라 버튼 클릭 리스너 구현
        cameraBtn = findViewById(R.id.buttonCamera)
        cameraBtn.setOnClickListener {
//            requirePermissions(arrayOf(CAMERA), PERMISSION_CAMERA)

            val cameraPermissionCheck = ContextCompat.checkSelfPermission(
                this@DiaryActivity,
                android.Manifest.permission.CAMERA
            )
            if (cameraPermissionCheck != PackageManager.PERMISSION_GRANTED) { // 권한이 없는 경우
                ActivityCompat.requestPermissions(
                    this,
                    arrayOf(android.Manifest.permission.CAMERA),
                    1000
                )
            } else { //권한이 있는 경우
                val REQUEST_IMAGE_CAPTURE = 1
                Intent(MediaStore.ACTION_IMAGE_CAPTURE).also { takePictureIntent ->
                    takePictureIntent.resolveActivity(packageManager)?.also {
                        startActivityForResult(takePictureIntent, REQUEST_IMAGE_CAPTURE)
                    }
                }
            }


        }


       makeDiary=findViewById(R.id.buttonMakediary)
      makeDiary.setOnClickListener{
          // 이동하고자 하는 DiaryActivity를 시작하는 Intent를 생성
          val intent = Intent(this@DiaryActivity, MainActivity::class.java)
          val bitmapFromImageView = (imagePreview.drawable as BitmapDrawable).bitmap

          uploadImageToServer(bitmapFromImageView,"멀티캠퍼스")
          // DiaryActivity로 이동
          startActivity(intent)


      }


    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == 1000) {
            if (grantResults[0] != PackageManager.PERMISSION_GRANTED) { //거부
                Toast.makeText(this@DiaryActivity, "권한을 허용해주세요", Toast.LENGTH_SHORT).show()
            }
        }
    }

    // onActivityResult 메서드를 오버라이드합니다.
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == REQUEST_IMAGE_CAPTURE && resultCode == Activity.RESULT_OK) {

            // 카메라로부터 사진을 가져왔을 때의 처리 코드입니다.
            val imageBitmap = data?.extras?.get("data") as Bitmap // 카메라로 찍은 사진을 Bitmap으로 가져옵니다.
            imagePreview.setImageBitmap(imageBitmap) // 가져온 사진을 ImageView에 표시합니다.
        }
    }

    private fun prepareFilePart(partName: String, bitmap: Bitmap): MultipartBody.Part {
        val file = bitmapToFile(bitmap) // Bitmap을 파일로 변환합니다.
        val requestFile = RequestBody.create(MultipartBody.FORM, file)
        return MultipartBody.Part.createFormData(partName, file.name, requestFile)
    }

    private fun bitmapToFile(bitmap: Bitmap): File {
        val filesDir = applicationContext.filesDir
        val file = File(filesDir, "image.png")
        val os: OutputStream
        try {
            os = FileOutputStream(file)
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, os)
            os.flush()
            os.close()
        } catch (e: Exception) {
            e.printStackTrace()
        }
        return file
    }

    private fun uploadImageToServer(imageBitmap: Bitmap, keyword: String) {

        Log.d("dd","사진 업로드 시작")
        val filePart = prepareFilePart("file", imageBitmap) // 이미지 파일을 MultipartBody.Part로 변환합니다.
//        val userId = RequestBody.create(MultipartBody.FORM, "1") // userId를 RequestBody로 변환합니다.
        val userIdString = "1"
        val userIdRequestBody = RequestBody.create(MediaType.parse("text/plain"), userIdString)
//        val userIdPart = MultipartBody.Part.createFormData("userId", userIdString, userIdRequestBody)

        val keywordString = "멀티캠퍼스"
        val keywordRequestBody = RequestBody.create(MediaType.parse("text/plain"), keywordString)
//        val keywordPart = MultipartBody.Part.createFormData("keyword", keywordString,keywordRequestBody)
//        val keywordd = RequestBody.create(MultipartBody.FORM, "멀티캠퍼스")
        // Retrofit을 사용하여 서버로 요청을 보냅니다.
        GlobalScope.launch(Dispatchers.IO) {
            try {
                Log.d("일단 날리기전","ㅇㅇ")
                val response = RetrofitClient.getRetrofitService.makeDiary(keywordRequestBody, filePart, userIdRequestBody )
                Log.d("일단 날림",response.toString())
                if (response.isSuccessful) {

                    Log.d("dd","사진 업로드 성공")
                    // 서버로부터 응답을 성공적으로 받았을 때의 처리
                    val diary = response.body()
                    // diary 객체를 사용하거나 필요한 처리를 수행합니다.
                } else {
                    // 서버로부터 오류 응답을 받았을 때의 처리
                    Log.d("dd","사진 업로드 실패")
                    // response.errorBody() 등을 사용하여 오류 응답을 처리할 수 있습니다.
                }
            } catch (e: Exception) {
                // 네트워크 오류 등 예외 발생 시의 처리
                Log.d("dd","사진 오류ㅠㅠㅠ")
                e.printStackTrace()
            }
        }
    }








//
//
//
//
//    private val apiService: ApiService by lazy {
//        val retrofit = Retrofit.Builder()
//            .baseUrl("YOUR_BASE_URL") // 서버의 기본 URL을 입력해주세요
//            .addConverterFactory(GsonConverterFactory.create())
//            .build()
//
//        retrofit.create(ApiService::class.java)
//    }
//
//    private fun uploadImageToServer(imageBitmap: Bitmap, keyword: String) {
//        val filePart = prepareFilePart("file", imageBitmap) // 이미지 파일을 MultipartBody.Part로 변환합니다.
//        val userIdPart = RequestBody.create(MultipartBody.FORM, "1") // userId를 RequestBody로 변환합니다.
//
//        // Retrofit을 사용하여 서버로 요청을 보냅니다.
//        GlobalScope.launch(Dispatchers.IO) {
//            try {
//                val response = apiService.makeDiary(keyword, filePart, userIdPart)
//                if (response.isSuccessful) {
//                    // 서버로부터 응답을 성공적으로 받았을 때의 처리
//                    val diary = response.body()
//                    // diary 객체를 사용하거나 필요한 처리를 수행합니다.
//                } else {
//                    // 서버로부터 오류 응답을 받았을 때의 처리
//                    // response.errorBody() 등을 사용하여 오류 응답을 처리할 수 있습니다.
//                }
//            } catch (e: Exception) {
//                // 네트워크 오류 등 예외 발생 시의 처리
//                e.printStackTrace()
//            }
//        }
//    }
//
//    private fun prepareFilePart(partName: String, bitmap: Bitmap): MultipartBody.Part {
//        val file = bitmapToFile(bitmap) // Bitmap을 파일로 변환합니다.
//        val requestFile = RequestBody.create(MultipartBody.FORM, file)
//        return MultipartBody.Part.createFormData(partName, file.name, requestFile)
//    }
//
//    private fun bitmapToFile(bitmap: Bitmap): File {
//        val filesDir = applicationContext.filesDir
//        val file = File(filesDir, "image.png")
//        val os: OutputStream
//        try {
//            os = FileOutputStream(file)
//            bitmap.compress(Bitmap.CompressFormat.PNG, 100, os)
//            os.flush()
//            os.close()
//        } catch (e: Exception) {
//            e.printStackTrace()
//        }
//        return file
//    }


}