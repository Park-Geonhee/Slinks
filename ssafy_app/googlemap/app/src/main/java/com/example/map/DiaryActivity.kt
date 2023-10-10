package com.example.map

import android.content.ContentValues
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.os.Environment
import android.provider.MediaStore
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.Toast
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat

class DiaryActivity : AppCompatActivity() {

    private lateinit var cameraBtn:Button
    private lateinit var makeDiary:Button
    private val PERMISSION_CAMERA = 1001 // 원하는 숫자로 권한 코드를 지정합니다.
    private val CAMERA = android.Manifest.permission.CAMERA // CAMERA 상수 정의

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_diary)


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


       makeDiary=findViewById(R.id.buttonCamera)
      makeDiary.setOnClickListener{
          // 이동하고자 하는 DiaryActivity를 시작하는 Intent를 생성
          val intent = Intent(this@DiaryActivity, MainActivity::class.java)
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


//    /**자식 액티비티에서 권한 요청 시 직접 호출하는 메서드
//     * @param permissions 권한 처리를 할 권한 목록
//     * @param requestCode 권한을 요청한 주체가 어떤 것인지 구분하기 위함.
//     * */
//    fun requirePermissions(permissions: Array<String>, requestCode: Int) {
//        Log.d("권한 요청")
//        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.M) {
//            permissionGranted(requestCode)
//        } else {
//            // isAllPermissionsGranted : 권한이 모두 승인 되었는지 여부 저장
//            // all 메서드를 사용하면 배열 속에 들어 있는 모든 값을 체크할 수 있다.
//            val isAllPermissionsGranted =
//                permissions.all { checkSelfPermission(it) == PackageManager.PERMISSION_GRANTED }
//            if (isAllPermissionsGranted) {
//                permissionGranted(requestCode)
//            } else {
//                // 사용자에 권한 승인 요청
//                ActivityCompat.requestPermissions(this, permissions, requestCode)
//            }
//        }
//    }
//
//    /** 사용자가 권한을 승인하거나 거부한 다음에 호출되는 메서드
//     * @param requestCode 요청한 주체를 확인하는 코드
//     * @param permissions 요청한 권한 목록
//     * @param grantResults 권한 목록에 대한 승인/미승인 값, 권한 목록의 개수와 같은 수의 결괏값이 전달된다.
//     * */
//    override fun onRequestPermissionsResult(
//        requestCode: Int,
//        permissions: Array<out String>,
//        grantResults: IntArray
//    ) {
//        if (grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
//            permissionGranted(requestCode)
//        } else {
//            permissionDenied(requestCode)
//        }
//    }
//
//    private fun permissionGranted(requestCode: Int) {
//        when (requestCode) {
//            PERMISSION_CAMERA -> openCamera()
//        }
//    }
//
//    private fun permissionDenied(requestCode: Int) {
//        when (requestCode) {
//            PERMISSION_CAMERA -> Toast.makeText(
//                this,
//                "카메라 권한을 승인해야 카메라를 사용할 수 있습니다.",
//                Toast.LENGTH_LONG
//            ).show()
//        }
//    }

//    private fun openCamera() {
//        val intent = Intent(MediaStore.ACTION_IMAGE_CAPTURE)
//
//        createImageUri(newFileName(), "image/jpg")?.let { uri ->
//            realUri = uri
//            // MediaStore.EXTRA_OUTPUT을 Key로 하여 Uri를 넘겨주면
//            // 일반적인 Camera App은 이를 받아 내가 지정한 경로에 사진을 찍어서 저장시킨다.
//            intent.putExtra(MediaStore.EXTRA_OUTPUT, realUri)
//            startActivityForResult(intent, REQUEST_CAMERA)
//        }
//    }
//
//    private fun newFileName(): String {
//        val sdf = SimpleDateFormat("yyyyMMdd_HHmmss")
//        val filename = sdf.format(System.currentTimeMillis())
//        return "$filename.jpg"
//    }
//
//    private fun createImageUri(filename: String, mimeType: String): Uri? {
//        var values = ContentValues()
//        values.put(MediaStore.Images.Media.DISPLAY_NAME, filename)
//        values.put(MediaStore.Images.Media.MIME_TYPE, mimeType)
//        return this.contentResolver.insert(MediaStore.Images.Media.EXTERNAL_CONTENT_URI, values)
//    }
//
//    /** 카메라 및 앨범 Intent 결과
//     * */
//    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
//        super.onActivityResult(requestCode, resultCode, data)
//
//        if (resultCode == RESULT_OK) {
//            when (requestCode) {
//                REQUEST_CAMERA -> {
//                    realUri?.let { uri ->
//                        imageView.setImageURI(uri)
//                    }
//                }
//            }
//        }
//    }







}