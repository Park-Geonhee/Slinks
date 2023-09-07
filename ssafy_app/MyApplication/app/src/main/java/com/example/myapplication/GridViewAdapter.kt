package com.example.myapplication

import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.BaseAdapter
import android.widget.ImageView
import android.widget.TextView



class GridViewAdapter(val context: Context, val img_list: Array<Int>, val text_list:Array<String>) :BaseAdapter() {

    override fun getView(p0: Int, p1: View?, p2: ViewGroup?): View {
        val view : View = LayoutInflater.from(context).inflate(R.layout.gridview_item, null)

        // TextView와 ImageView를 참조
        val textView = view.findViewById<TextView>(R.id.gridview_text)
        val imageView = view.findViewById<ImageView>(R.id.gridview_img)
        // 데이터를 설정하거나 뷰의 속성을 변경
        textView.text = text_list[p0]
        imageView.setImageResource(img_list[p0])

//        view.gridview_text.text=text_list[p0]
//        view.gridview_img.setImageResource(img_list[p0])



        return view
    }
    override fun getItem(p0: Int): Any {
        return 0
    }

    override fun getItemId(p0: Int): Long {
        return 0
    }

    //넘길 이미지의 개수 설정
    override fun getCount(): Int {
        return img_list.size
    }






}