package com.nemo.neplan.model;

public enum PlaceType {

        MT1("대형마트"),
        CS2("편의점"),
        PS3("어린이집, 유치원"),
        SC4("학교"),
        AC5("학원"),
        PK6("주차장"),
        OL7("주유소, 충전소"),
        SW8("지하철역"),
        BK9("은행"),
        CT1("문화시설"),
        AG2("중개업소"),
        PO3("공공기관"),
        AT4("관광명소"),
        AD5("숙박"),
        FD6("음식점"),
        CE7("카페"),
        HP8("병원"),
        PM9("약국"),
        ETC("기타");

        private final String value;

        PlaceType(String value) {
                this.value = value;
        }

        public String getValue() {
                return value;
        }

        public static PlaceType fromString(String text) {
                for (PlaceType placeType : PlaceType.values()) {
                        if (placeType.getValue().equalsIgnoreCase(text)) {
                                return placeType;
                        }
                }
                return ETC; // 기본적으로 ETC로 설정하거나 예외 처리를 할 수 있습니다.
        }


}
