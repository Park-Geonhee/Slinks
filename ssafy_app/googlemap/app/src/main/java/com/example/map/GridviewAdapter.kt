package com.example.map

import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.BaseAdapter
import android.widget.TextView
import retrofit2.Callback

class GridviewAdapter(val context: Context, val text_list:ArrayList<String>):BaseAdapter() {
    override fun getCount(): Int {
       return text_list.size
    }

    override fun getItem(p0: Int): Any {
        return 0
    }

    override fun getItemId(p0: Int): Long {
       return 0
    }

    override fun getView(p0: Int, p1: View?, p2: ViewGroup?): View {
        val view:View=LayoutInflater.from(context).inflate(R.layout.gridview_item, null)

        //gridview_item xml에 있는 gridview_text에 text_list의 값을 대입하고 싶어
        val textView: TextView = view.findViewById(R.id.gridview_text)

        // text_list의 해당 위치(position)에 있는 값을 가져와 textView에 설정합니다.
        textView.text = text_list[p0]

        return view
    }

}