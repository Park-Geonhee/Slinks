
## Class IMGParser
	Camera Subscribe
	Odometry Subscribe
	LanePath Publish

	while
		mask_roi
		BEVTransform.warp_bev_img
		binarize
		BEVTransform.warp_inv_img
		BEVTransform.recon_lane_pts

		CurveFit.fit_curve
		CurveFit.set_vehicle_status
		CurveFit.write_path_msg
		BEVTransform.project_lane2img
		draw_lane_img

	def odom_callback      : odometry topic callback
	def callback           :  camera topic callback
	def binarize	       : 이미지 내 흰색/노란색 영역 만 선택
	def maskroi	           : 이미지 내 특정 영역 선택
	def draw_lane_img	   : 이미지 내 검출된 차선 표시

---
## Class BEVTransform
	theta 		 : cam Pitch radian
	width 		 : cam width
	height 		 : cam height
	x 		     : cam X coord
	h		     : cam Z + 0.34
	n		     : float(width)
	m		     : float(height)
	alpha_r		 : cam FOV/2 radian
	alpha_c		 : arctan (width/2, fc_y)
	fc_y		 : focal length y
	fc_x		 : focal length x
	RT_b2g		 : Rotation MAP, BEV -> 3차원 ground
	poject_mat	 : 2D 사영 위한 매트릭스
	
	
	def calc_Xv_Yu	      : 2D 이미지 픽셀 좌표를 3차원 좌표계 변경
	def build_tf	      : BEV(Bird Eye View), BEV 복구 매트릭스 생성
	def warp_bev_img      : 이미지 -> BEV 변경
	def warp_inv_img      : BEV -> 이미지 변경
	def recon_lane_pts    : 실질적 차선 검출, INV 거친 이미지 내 1인 영역 검출
	def project_lane2img  : 3차원 좌표 -> 2차원 이미지 좌표 변경, 이미지 크기만큼 slice
	def project2img_mtx   : cam params 기반 2D 사영 매트릭스
	def translationMtx    : x,y,z 이동 매트릭스
	def rotationMtx       : y,p,r 회전 매트릭스

---
## Class CURVEFit:
	order		  : 다항식 차수, 높으면 오버피팅 가능성 높다
	lane_width	  : 가상 차선의 너비, 계속 업데이트
	y_margin	  : 예측 차선 노이즈 범위
	x_range		  : 차선의 범위
	min_pts		  : RANSAC Regressor 최소 샘플 개수
	dx		      : 차선 범위 간격
	lane_path	  : 최종 lane path
	ransac_left	  : 왼쪽 차선 검출 모델
	ransac_right  : 오른쪽 차선 검출 모델

	
	def init_model	       : 모델 초기 설정
	def preprocess_pts	   : 샘플링, 예측, 차선 검출
	def fit_curve 	       : 최소 샘플 개수보다 적다면 다시 preprocess,
			               : x_range, dx 간격으로 모델에 적용하여 예측값 산출
	def update_lane_width  : 예측값이 특정 값보다 작거나 크다면 lane_width 업데이트
	def write_path_msg     : Path() 메세지 작성
	def set_vehicle_status : 현재 차량 orientation, x,y 등 값 갱신
	
	



	
