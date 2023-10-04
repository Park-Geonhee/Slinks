package com.example.map

import android.app.TimePickerDialog
import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.TimePicker
import com.example.map.databinding.ActivityMainBinding
import com.example.map.databinding.ActivityPlanDetailBinding
import com.google.android.material.datepicker.CalendarConstraints
import com.google.android.material.datepicker.DateValidatorPointForward
import com.google.android.material.datepicker.MaterialDatePicker
import java.text.SimpleDateFormat
import java.util.Calendar
import java.util.Date
import java.util.Locale

class PlanDetailActivity : AppCompatActivity() , TimePicker.OnTimeChangedListener{

    private lateinit var binding: ActivityPlanDetailBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityPlanDetailBinding.inflate(layoutInflater)
        setContentView(binding.root)

        binding.btnPrevious.setOnClickListener(View.OnClickListener {


            val intent = Intent(this@PlanDetailActivity, PlanListActivity::class.java)

            // 필요한 경우, Intent에 데이터를 추가할 수 있음
            // intent.putExtra("key", value)

            // 이동
            startActivity(intent)

        })


        //현재 시간을 가져와 저장
        var cal = Calendar.getInstance()
        val hour = cal.get(Calendar.HOUR_OF_DAY)
        val minute = cal.get(Calendar.MINUTE)


        // 현재 시간을 가져와서 timesettingBtn에 표시
        updateButtonTime(hour, minute)
        binding.timesettingBtn.setOnClickListener {
            // TimePickerDialog를 표시하여 시간을 선택할 수 있도록 함
            val timePickerDialog = TimePickerDialog(
                this,
                TimePickerDialog.OnTimeSetListener { _: TimePicker, selectedHour: Int, selectedMinute: Int ->
                    // 사용자가 선택한 시간을 timesettingBtn에 반영
                    updateButtonTime(selectedHour, selectedMinute)
                },
                hour,
                minute,
                false
            )
            timePickerDialog.show()
        }
    }



    override fun onTimeChanged(p0: TimePicker?, p1: Int, p2: Int) {
        TODO("Not yet implemented")
    }

        fun updateButtonTime(hour: Int, minute: Int) {
            // 선택한 시간을 timesettingBtn에 표시
            val timeFormat = SimpleDateFormat("a hh:mm", Locale.getDefault())
            val calendar = Calendar.getInstance()
            calendar.set(Calendar.HOUR_OF_DAY, hour)
            calendar.set(Calendar.MINUTE, minute)
            val formattedTime = timeFormat.format(calendar.time)
            binding.timesettingBtn.text = formattedTime
        }


}
