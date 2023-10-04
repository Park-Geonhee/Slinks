package com.example.map

import android.Manifest
import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.content.pm.PackageManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Toast
import androidx.core.content.ContextCompat
import com.example.map.databinding.ActivityMainBinding
import com.google.android.gms.maps.CameraUpdateFactory
import com.google.android.gms.maps.GoogleMap
import com.google.android.gms.maps.OnMapReadyCallback
import com.google.android.gms.maps.SupportMapFragment
import com.google.android.gms.maps.model.LatLng
import com.google.android.gms.maps.model.MarkerOptions
import com.google.android.material.datepicker.CalendarConstraints
import com.google.android.material.datepicker.DateValidatorPointForward
import com.google.android.material.datepicker.MaterialDatePicker
import java.util.Calendar
import java.util.Date
import kotlinx.coroutines.*


class MainActivity : AppCompatActivity(){

    private lateinit var binding: ActivityMainBinding


    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding=ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
//        binding.textView.text="안녕"





//        CoroutineScope(Dispatchers.IO).launch {
//            try {
//                val apiService = RetrofitClient.apiService // Retrofit 인스턴스를 통해 API 서비스를 가져옵니다.
//                val users = apiService.getAllUsers() // API 요청 (suspend 함수이므로 코루틴 내부에서 호출해야 합니다.)
//
//                // UI 업데이트를 위해 메인 스레드로 전환
//                withContext(Dispatchers.Main) {
//                    // users 리스트를 이용하여 UI 업데이트 작업 수행
//                    for (user in users) {
//
//                        print(user.name)
//                        print(user.name)
//                        print("hiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii")
//
//                        binding.textViewUsername.text = "Username: ${user.name}"
//                        binding.textViewAddress.text = "Address: ${user.address}"
//                    }
//                }
//            } catch (e: Exception) {
//                Log.e("API Error", e.message ?: "Unknown error")
//            }
//        }





        binding.planList.setOnClickListener(View.OnClickListener {
            // 이동하고자 하는 DiaryActivity를 시작하는 Intent를 생성
            val intent = Intent(this@MainActivity, PlanListActivity::class.java)

            // 필요한 경우, Intent에 데이터를 추가할 수 있음
            // intent.putExtra("key", value)

            // DiaryActivity로 이동
            startActivity(intent)

        })

        binding.aiDiary.setOnClickListener(View.OnClickListener {
            var intent=Intent(this@MainActivity,DiaryActivity::class.java )

            //ai diary페이지로 이동
            startActivity(intent)
        })

        binding.travelPlan.setOnClickListener(View.OnClickListener {
            var intent=Intent(this@MainActivity,MapActivity::class.java )

            //여행 플랜 만드는 페이지로 이동
            startActivity(intent)
        })


        //sharedPreference를 이용한 기기에 선택한 날짜 데이터 저장
        val sharedPreference = getSharedPreferences("CreateProfile", Context.MODE_PRIVATE)
        val editor: SharedPreferences.Editor = sharedPreference.edit()

        binding.calanderButtonCreate.setOnClickListener {
            val calendarConstraintBuilder = CalendarConstraints.Builder()
            calendarConstraintBuilder.setValidator(DateValidatorPointForward.now())

            val builder = MaterialDatePicker.Builder.datePicker()
                .setTitleText("날짜를 선택하세요:)")
                .setSelection(MaterialDatePicker.todayInUtcMilliseconds())
                .setCalendarConstraints(calendarConstraintBuilder.build())

            val datePicker = builder.build()

            datePicker.addOnPositiveButtonClickListener {
                val calendar = Calendar.getInstance()
                calendar.time = Date(it)
                val calendarMilli = calendar.timeInMillis
                binding.calanderButtonCreate.text =
                    "${calendar.get(Calendar.MONTH) + 1}/${calendar.get(Calendar.DAY_OF_MONTH)}/${calendar.get(Calendar.YEAR)}"

                // SharedPreference
                editor.putLong("Die_Millis", calendarMilli)
                editor.apply()
                 Log.d("Die_Millis", "$calendarMilli") // Log 사용을 위해 추가해야 함
            }
            datePicker.show(supportFragmentManager, datePicker.toString())
        }


    }




}



//프래그먼트 일 경우
/*
* class BlankFragment : Fragment() {

    private var _binding: FragmentBlankBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        _binding = FragmentBlankBinding.inflate(inflater, container, false)
        val view = binding.root
        return view
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding.textView.text = "안녕"
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}
*
*
* */

