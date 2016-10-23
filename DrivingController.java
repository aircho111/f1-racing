public class DrivingController {	
	public class DrivingCmd{
		public double steer;
		public double accel;
		public double brake;
		public int backward;
	};
	
	public double track_last_angle = 0.0;  // 이전 트랙 angle 보관
	public double curr_whole_track_dist_straight = 0.0;  // 현재 직진 트랙의 전체 트랙 길이 보관
	public double track_last_dist_straight = 0.0;  // 이전 직진 트랙의 남은 길이 보관	
	public boolean isLogger = true;  // 테스트 로그용 출력
	
	public DrivingCmd controlDriving(double[] driveArray, double[] aicarArray, double[] trackArray, double[] damageArray, int[] rankArray, int trackCurveType, double[] trackAngleArray, double[] trackDistArray, double trackCurrentAngle){
		DrivingCmd cmd = new DrivingCmd();
		
		////////////////////// input parameters
		double toMiddle     = driveArray[DrivingInterface.drvie_toMiddle    ];
		double angle        = driveArray[DrivingInterface.drvie_angle       ];
		double speed        = driveArray[DrivingInterface.drvie_speed       ];

		double toStart				 = trackArray[DrivingInterface.track_toStart		];
		double dist_track			 = trackArray[DrivingInterface.track_dist_track		];
		double track_width			 = trackArray[DrivingInterface.track_width			];
		double track_dist_straight	 = trackArray[DrivingInterface.track_dist_straight	];
		int track_curve_type		 = trackCurveType;

		double[] track_forward_angles	= trackAngleArray;
		double[] track_forward_dists	= trackDistArray;
		double track_current_angle		= trackCurrentAngle;
		
		double[] dist_cars = aicarArray;
		
		double damage		 = damageArray[DrivingInterface.damage];
		double damage_max	 = damageArray[DrivingInterface.damage_max];

		int total_car_num	 = rankArray[DrivingInterface.rank_total_car_num	];
		int my_rank			 = rankArray[DrivingInterface.rank_my_rank			];
		int opponent_rank	 = rankArray[DrivingInterface.rank_opponent_rank	];		
		////////////////////// END input parameters
		
		// To-Do : Make your driving algorithm
		System.out.println("====================== [start] =============================");
		
		double steer_angle = 0.0;
		double streer_coeff = 0.3;
		double corr_toMiddle = 0.0;
		double forward_ai_dist = 100.0;
		double emer_turn_yn = -1.0;
		double[] corr_route = new double[3]; 
		double corr_break = 0.0;
		double corr_accel = 0.25;
		double track_curve_level = 0.0;
		
		double[] user_change_dist = new double[2];
		
		// 직전 트랙Angle과 현재 트랙Angle과의 차이로 커브의 수준 파악을 위함.
		track_curve_level = Math.abs(track_current_angle - track_last_angle);
		if(track_last_dist_straight == 0.0 && track_dist_straight > 0.0) {
			curr_whole_track_dist_straight = track_dist_straight;
		} else {
			if(track_dist_straight == 0.0) {
				curr_whole_track_dist_straight = 0.0;
			}
		}
		
		// 변경할 angle 계산
		double user_chagne_angle = angle;
		
		double[] forward_track_info = new double[2];
		forward_track_info = this.getForwardTrackInfo(speed, track_forward_angles, track_forward_dists, track_current_angle, toStart, angle, track_dist_straight, track_curve_type);		
		user_chagne_angle  = forward_track_info[0];
		
		if(isLogger) {			
			System.out.println("toStart : " + toStart);			
			System.out.println("track_width : " + track_width);
			System.out.println("track_last_angle : " + track_last_angle);
			System.out.println("track_curr_angle : " + track_current_angle);
			System.out.println("track_curve_level : " + track_curve_level);
			System.out.println("speed : " + speed);  // 수치가 이상하게 들어옴...
			System.out.println("toMiddle : " + toMiddle); // 음수이면 중앙에서 오른쪽, 양수이면 왼쪽
			System.out.println("angle : " + angle);
			System.out.println("user_chagne_angle : " + user_chagne_angle);
			System.out.println("track_dist_straight : " + track_dist_straight); 
			System.out.println("track_last_dist_straight : " + track_last_dist_straight); 
			System.out.println("curr_whole_track_dist_straight : " + curr_whole_track_dist_straight);
			
			System.out.println("---------------------------------------------------");
		}
		
		angle = user_chagne_angle;
		
		// 차량이 이동할 트랙의 상대위치 (+값은 오른쪽, -값은 왼쪽), 장애물로 인해 이동이 불가한 경우 -100 리턴
		//corr_toMiddle = this.getCorrToMiddle(dist_cars, toMiddle, speed, angle, track_width, track_dist_straight, track_curve_type);
		corr_route[0] = -1.0;
		corr_route[1] = 0.0;
		corr_route[2] = 100.0;
		corr_route = this.getCorrToMiddle(dist_cars, toMiddle, speed, angle, track_width, track_dist_straight, track_curve_type);
		
		emer_turn_yn = corr_route[0];   // 장애물 피하기 위한 경로 조정인 경우 0보다 큰값
		corr_toMiddle = corr_route[1];  // 이동할 경로 (내차 중심으로 부터 횡 간격)
		forward_ai_dist = corr_route[2];  // 이동할 경로 (내차 중심으로 부터 횡 간격)
		
		if(emer_turn_yn > 1.0) {
			user_change_dist[0] = this.getBestDist(toMiddle, track_width, track_curve_type);
			user_change_dist = this.getChangeDist(dist_cars, toMiddle, user_change_dist[0], speed, track_width, track_dist_straight, track_curve_type);
		
			corr_toMiddle = user_change_dist[0];
			forward_ai_dist = user_change_dist[1];
		}
		
		double user_best_speed  = 100;
		
		/* ------------ 속도 제어 함수 이관 -------------- */	
		// 속도 제어 함수  <-- 이관함수
		
		user_best_speed = this.getBestSpeed(angle, forward_ai_dist, speed, track_dist_straight, track_curve_type);
		
		// 브레이크, 엑셀 제어 <-- 이관함수
		double[] user_speed_ctl = this.getSpeedCtl(speed, user_best_speed, track_dist_straight);
		
		double user_accelCtl = user_speed_ctl[0];  
		double user_breakCtl = user_speed_ctl[1];  
		/* ------------ 속도 제어 함수 이관 -------------- */	
		
		
		// 브레이크, 엑셀 제어 <-- 자체함수(임시로 전방 커브 10M 전에 차량의 속도가 110K 이상인 경우 브레이킹...커브에서는 커브경사도별로 구분함)
		double[] user_speed_ctl2 = this.getSpeedCtl2(speed, user_best_speed, track_dist_straight, track_curve_level);
		double user_accelCtl2 = user_speed_ctl2[0];  
		double user_breakCtl2 = user_speed_ctl2[1]; 
		/*-----------------------------------------*/
		
		// 전방 장애물이 있는 경우 속도제어 함수에 따라 조절
		if(emer_turn_yn > 1.0) {
			corr_accel = user_accelCtl; 
			corr_break = user_breakCtl;
		} else {
			corr_accel = user_accelCtl2; 
			corr_break = user_breakCtl2;
		}
		
		/* --- 트랙조건에 따른 angle계수 추가 함수 필요 : 우열책임 -- */
		// angle값에 대한 계수 계산(속도, 트랙의 조건에 따라 계산)
//		streer_coeff = this.getSteerCoeff(speed, track_dist_straight);
		streer_coeff = this.getSteerCoeff2(track_current_angle, track_forward_angles, track_curve_type);
		
		// 전방 장애물이 있는 경우 
		if(emer_turn_yn > 0.0) {
			streer_coeff = 1.0;
		}
		
		/* --- 가속/감속 추가 함수 필요 : 진희책임 -- */		
		// 차량의 break 조건 처리 등
		cmd.backward = DrivingInterface.gear_type_forward;
		
		if(toStart!=0 && speed<1){
			System.out.println("시작점이 아니고 스피드가 1이하인경우");
			
			if(Math.abs(toMiddle) > track_width/2){
				System.out.println("트랙밖에 있는 경우");
				if(toMiddle>0){
					System.out.println("중앙선 오른쪽에 있는 경우");
					corr_toMiddle = 0;
					streer_coeff -= 0.5;
					corr_accel += 0.5;
					
				}else{
					System.out.println("중앙선 왼쪽에 있는 경우");
					corr_toMiddle = 0;
					streer_coeff += 0.5;
					corr_accel += 0.5;
				}
				
			}else{
				System.out.println("트랙안에 있는 경우");	
				if(toMiddle<0){
					System.out.println("중앙선 오른쪽에 있는 경우");
					if(dist_cars[0]<10){
						System.out.println("간격이 10 이하");
						corr_toMiddle = corr_toMiddle + 4;
						user_best_speed = 100;
						speed += 20;
						streer_coeff -= 0.5;
						cmd.backward += 1;
						corr_accel += 1;
					}else{
						System.out.println("간격이 10 이상");
						cmd.backward = 0;
					}
					
					
					
				}else{
					System.out.println("중앙선 왼쪽에 있는 경우");
					if(dist_cars[0]<10){
						System.out.println("간격이 10 이하");
						corr_toMiddle = corr_toMiddle - 4;
						user_best_speed = 100;
						speed += 20;
						streer_coeff += 0.5;
						cmd.backward += 1;
						corr_accel += 1;
						
					}else{
						System.out.println("간격이 10 이상");
						//cmd.backward = 0;
					}
					
					
				}
			}
			
		}
		
		if(isLogger) {
			System.out.println("emergency turn yn : " + emer_turn_yn);
		}
		// 차량이 회전할 angle값 계산
		steer_angle = this.getSteerAngle(angle, corr_toMiddle, track_width, streer_coeff);
		if(isLogger) {
			System.out.println("steer_angle : " + steer_angle);
			System.out.println("curr damage : " + damage + "(" + damage_max + ")");
		}
		this.track_last_angle = track_current_angle;
		this.track_last_dist_straight = track_dist_straight;
		/*-----------------------------------------*/
		
		////////////////////// output values		
		cmd.steer = steer_angle;
		cmd.accel = corr_accel; 
		cmd.brake = corr_break;
		
		////////////////////// END output values
		System.out.println("====================== [end] =============================");
		return cmd;
	}
	
	public static void main(String[] args) {
		DrivingInterface driving = new DrivingInterface();
		DrivingController controller = new DrivingController();
		
		double[] driveArray = new double[DrivingInterface.INPUT_DRIVE_SIZE];
		double[] aicarArray = new double[DrivingInterface.INPUT_AICAR_SIZE];
		double[] trackArray = new double[DrivingInterface.INPUT_TRACK_SIZE];
		double[] damageArray = new double[DrivingInterface.INPUT_DAMAGE_SIZE];
		int[] rankArray = new int[DrivingInterface.INPUT_RANK_SIZE];
		int[] trackCurveType = new int[1];
		double[] trackAngleArray = new double[DrivingInterface.INPUT_FORWARD_TRACK_SIZE];
		double[] trackDistArray = new double[DrivingInterface.INPUT_FORWARD_TRACK_SIZE];
		double[] trackCurrentAngle = new double[1];
				
		// To-Do : Initialize with your team name.
		int result = driving.OpenSharedMemory();
		
		if(result == 0){
			boolean doLoop = true;
			while(doLoop){
				result = driving.ReadSharedMemory(driveArray, aicarArray, trackArray, damageArray, rankArray, trackCurveType, trackAngleArray, trackDistArray, trackCurrentAngle);
				switch(result){
				case 0:
					DrivingCmd cmd = controller.controlDriving(driveArray, aicarArray, trackArray, damageArray, rankArray, trackCurveType[0], trackAngleArray, trackDistArray, trackCurrentAngle[0]);
					driving.WriteSharedMemory(cmd.steer, cmd.accel, cmd.brake, cmd.backward);
					break;
				case 1:
					break;
				case 2:
					// disconnected
				default:
					// error occurred
					doLoop = false;
					break;
				}
			}
		}
	}
	
	

	/**
	 * 전방 트랙정보 이용 함수 
	 * @param curr_track_forward_angles
	 * @param curr_track_forward_dists
	 * @param curr_track_current_angle
	 * @param curr_toStart
	 * @param curr_angle
	 * @param curr_track_dist_straight
	 * @param curr_track_curve_type
	 * @return
	 */
	private double[] getForwardTrackInfo(double curr_speed, double[] curr_track_forward_angles, double[] curr_track_forward_dists, double curr_track_current_angle, double curr_toStart, double curr_angle, double curr_track_dist_straight, double curr_track_curve_type){
		double[] forward_track_info = new double[2];
		forward_track_info[0] =  curr_angle; // 변경할 전방 angle 정보
		forward_track_info[1] =  0; // 추가 사용 예정

		double user_forward_dists = 0;
		
		double track_chage_angle  = 0;
		double user_chagne_angle  = 0;
		long start_time = System.nanoTime();
		for(int i=0;i<20;i++){
			/* 상대거리 계산 */
			user_forward_dists = curr_track_forward_dists[i] - curr_toStart;
//			System.out.println("curr_speed["+i+"]   = " + curr_speed);
//			
//			System.out.println("track_forward_dists["+i+"]   = " + user_forward_dists);
//			System.out.println("track_current_angle     = " + track_current_angle*180/3.14);
//			System.out.println("track_forward_angles["+i+"] = " + track_forward_angles[i]);
//			System.out.println("track_forward_angles["+i+"]  = " + track_forward_angles[i]*180/3.14); 
			
			
			track_chage_angle = curr_track_current_angle - curr_track_forward_angles[i];			
				
//			System.out.println("track_chage_angle["+i+"]      = " + ( track_chage_angle*180/3.14 ));
//			System.out.println("track_chage_angle 파이["+i+"]   = " + track_chage_angle);
//			System.out.println("track_current_angle 파이["+i+"]  = " + ( curr_track_current_angle ));
//			System.out.println("track_forward_angles 파이["+i+"] = " + curr_track_forward_angles[i]); 
			
			if(i == 0 && user_forward_dists < 3) // 전방 1m 이내인 경우(트랙이 다수 변경되는 점)
			{
				user_chagne_angle = curr_angle + track_chage_angle;
				forward_track_info[0] = user_chagne_angle;
//				System.out.println("user_chagne_angle 파이["+i+"]    = " + user_chagne_angle);				
//				System.out.println();
				
			}else{
				forward_track_info[0] = curr_angle;
			}
			long end_time = System.nanoTime();
//			System.out.println("user_chagne_angle time " + (end_time-start_time));
			
			break; // 현재 angle 정보만 처리하기때문에 한번만 읽음
		}
		
		
		return forward_track_info;		
	}
	
	/**
	 * 핸들 Angle 산출
	 * @param curr_angle
	 * @param curr_toMiddle
	 * @param curr_track_width
	 * @param streer_coeff
	 * @return
	 */
	private double getSteerAngle(double curr_angle, double curr_toMiddle, double curr_track_width, double streer_coeff) {
		double steer_angle = 0.0;
		
		steer_angle = streer_coeff * (curr_angle - curr_toMiddle/curr_track_width);
		
		return steer_angle;
	}
	
	/**
	 * 핸들 Angle 산출식 계수
	 * @param curr_angle
	 * @param curr_toMiddle
	 * @param curr_track_width
	 * @param streer_coeff
	 * @return
	 */
	private double getSteerCoeff(double curr_speed, double curr_track_dist_straight) {
		double steer_coeff = 1.0;
		
		if(curr_track_dist_straight > 30) {
			if(curr_speed > 28.0) { 
				steer_coeff = 0.1;
			} else if( curr_speed > 25 && curr_speed <= 28.0) {
				steer_coeff = 0.3;
			} else if( curr_speed > 10 && curr_speed <= 25.0) {
				steer_coeff = 0.5;
			} else {
				steer_coeff = 1.0;
			}
		} else {
			steer_coeff = 1.0;
		}

		return steer_coeff;
	}
	
	private double getSteerCoeff2(double track_current_angle, double[] track_forward_angles, int track_curve_type){
		System.out.println("+++++++++++++++++++++++++++++++++++[steer coeff START]+++++++++++++++++++++++++++++++++++");
		double best_user_steer_coeff = 0.541052; // 핸들계수(트랙정보에 따라 셋팅)
		
		double currRemain = 0.0;
		double currRemainAndForwardAngleSum = 0.0;
		double realRemain = 0.0;
		
		//우회전인 경우
		if(track_curve_type == 1){
			currRemain = ((double)3.14) - Math.abs(track_current_angle);
			currRemainAndForwardAngleSum = Math.abs(track_forward_angles[0]) + currRemain;
			realRemain = ((double)3.14) - currRemainAndForwardAngleSum;
			
//			if(isLogger){
//				System.out.println("현재 트랙angle을 뺀 나머지                                      : " + currRemain);
//				System.out.println("전방 첫번째 angle                                               : " + Math.abs(track_forward_angles[0]));
//				System.out.println("현재 트랙angle을 뺀 나머지와 전방 첫번째 angle의 합             : " + currRemainAndForwardAngleSum);
//				System.out.println("현재 트랙angle을 뺀 나머지와 전방 첫번째 angle의 합을 뺀 나머지 : " + realRemain);
//			}
		}
		//좌회전인 경우
		else if(track_curve_type == 2){
			currRemain = Math.abs(track_current_angle);
			currRemainAndForwardAngleSum = (((double)3.14) - Math.abs(track_forward_angles[0])) + currRemain;
			realRemain = ((double)3.14) - currRemainAndForwardAngleSum;
			
//			if(isLogger){
//				System.out.println("현재 트랙angle                                                  : " + currRemain);
//				System.out.println("전방 첫번째 angle                                               : " + Math.abs(track_forward_angles[0]));
//				System.out.println("현재 트랙angle와 전방 첫번째 angle을 뺀 나머지의 합             : " + currRemainAndForwardAngleSum);
//				System.out.println("현재 트랙angle와 전방 첫번째 angle을 뺀 나머지의 합을 뺀 나머지 : " + realRemain);
//			}
		}
		
		//계수 세팅 방법 : 현재 트랙과 전방 첫번째 트랙 간의 각도가 클수록 급커브로 판단하고 계수를 작게 세팅
		//현재 트랙과 전방 첫번째 트랙이 같은 방향인 경우.. 라고 생각됨
		if(realRemain >= 0.0){
			if(realRemain < 0.1){
				best_user_steer_coeff = 1.0;
			}
			else if(realRemain < 0.2){
				best_user_steer_coeff = 0.9;
			}
			else if(realRemain < 0.3){
				best_user_steer_coeff = 0.8;
			}
			else if(realRemain < 0.4){
				best_user_steer_coeff = 0.7;
			}
			else if(realRemain < 0.5){
				best_user_steer_coeff = 0.6;
			}
			else if(realRemain < 0.6){
				best_user_steer_coeff = 0.5;
			}
			else if(realRemain < 0.7){
				best_user_steer_coeff = 0.4;
			}
			else if(realRemain < 0.8){
				best_user_steer_coeff = 0.3;
			}
			else if(realRemain < 0.9){
				best_user_steer_coeff = 0.2;
			}
			else if(realRemain < 1.0){
				best_user_steer_coeff = 0.1;
			}
			else{
				best_user_steer_coeff = 0.05;
			}
		}
		//현재 트랙과 전방 첫번째 트랙이 다른 방향인 경우.. 라고 생각됨
		else{
			if(realRemain < -1.0){
				best_user_steer_coeff = 0.05;
			}
			else if(realRemain < -0.9){
				best_user_steer_coeff = 0.1;
			}
			else if(realRemain < -0.8){
				best_user_steer_coeff = 0.2;
			}
			else if(realRemain < -0.7){
				best_user_steer_coeff = 0.3;
			}
			else if(realRemain < -0.6){
				best_user_steer_coeff = 0.4;
			}
			else if(realRemain < -0.5){
				best_user_steer_coeff = 0.5;
			}
			else if(realRemain < -0.4){
				best_user_steer_coeff = 0.6;
			}
			else if(realRemain < -0.3){
				best_user_steer_coeff = 0.7;
			}
			else if(realRemain < -0.2){
				best_user_steer_coeff = 0.8;
			}
			else if(realRemain < -0.1){
				best_user_steer_coeff = 0.9;
			}
			else{
				best_user_steer_coeff = 1.0;
			}
		}
		
		System.out.println("best_user_steer_coeff : " + best_user_steer_coeff);
		System.out.println("+++++++++++++++++++++++++++++++++++[steer coeff end]+++++++++++++++++++++++++++++++++++");
		
		return best_user_steer_coeff;
	}
	
	/**
	 * 최적 경로 식별
	 * @param curr_aicars
	 * @param curr_toMiddle
	 * @param curr_speed
	 * @param curr_angle
	 * @param curr_track_width
	 * @param curr_track_dist_straight
	 * @param curr_track_curve_type
	 * @return
	 */
	private double[] getCorrToMiddle(double[] curr_aicars, double curr_toMiddle, double curr_speed, double curr_angle, double curr_track_width, double curr_track_dist_straight, double curr_track_curve_type) {
		double corr_toMiddle = 0.0;
		double emer_turn_yn = 0.0;
		double tmp_ai_dist = 0.0;
		double fst_ai_dist = 0.0;
		double tmp_pre_ai_dist = 0.0;
		double tmp_ai_toMiddle = 0.0;
		double ai_car_width = 2.0;
		double ai_car_length = 4.0;
		double my_car_width = 2.0;
		double my_car_length = 4.5;
		double forward_dist_min = 2.0;
		double forward_dist_max = 80.0;
		double backward_dist_max = -10.0;
		double forward_ai_dist = 100.0;
		
		double[] ret_corr_route = new double[3];
		
		int[] tmp_r_ai = new int[10];
		int[] tmp_l_ai = new int[10];
		int[] tmp_c_ai = new int[10];
		int[] tmp_b_ai = new int[10];
		int tmp_r_ai_cnt = 0;
		int tmp_l_ai_cnt = 0;
		int tmp_c_ai_cnt = 0;
		int tmp_b_ai_cnt = 0;
		
		for(int i=0 ; i<10 ; i++){
			tmp_r_ai[i] = -1;
			tmp_l_ai[i] = -1;
			tmp_c_ai[i] = -1;
			tmp_b_ai[i] = -1;
		}
		
		ret_corr_route[0] = -1.0;
		ret_corr_route[1] = 0.0;
		ret_corr_route[2] = 100.0;
		
		// 장애물 차량 배영로 부터 전방, 좌측, 우측 별 ai 차량 배열 생성
		for(int i=1 ; i<curr_aicars.length ; i+=2) {
			//
			// 내차 기준 장애차량과의 간격
			tmp_ai_dist = this.getAiSideDist(curr_toMiddle, curr_aicars[i]);
						
			if(curr_aicars[i-1] > -100.0 && curr_aicars[i-1] < 100.0) {
				System.out.println("AI Car #" + (i+1)/2 + " : " + curr_aicars[i-1] + ", " + tmp_ai_dist);
			}
			
			
			// 처음 출발시는 모두 0.0이므로 제외
			if(curr_speed == 0.0){
				continue;
			}
			
			// 내 차보다 10M 뒤 80M 앞에 있는 ai차량은 일단 제외
			if(curr_aicars[i-1] <= backward_dist_max || curr_aicars[i-1] >= forward_dist_max) {
				continue;
			}
			
			/*================ 내차의 오른쪽 왼쪽 전방 AI 차량 배열 생성 ====================*/
			// 내차의 전방 충돌 위치 ai 차량 수
			// 전방 4~50M, 좌측우측 9M폭 사이에 있는 차량은 전방 차량으로 간주
			if((curr_aicars[i-1] > forward_dist_min && curr_aicars[i-1] < forward_dist_max) 
					&& (tmp_ai_dist > -(my_car_width + 0.5) && tmp_ai_dist < (my_car_width + 0.5))) {
				tmp_c_ai[tmp_c_ai_cnt] = i;
				tmp_c_ai_cnt++;
//				System.out.println("   --> 전방 ai 차량수 : " + tmp_c_ai_cnt);
			} else {
				
				// 내 차의 왼쪽에 위치하는 ai차량을  거리순으로 array에 저장
				if(tmp_ai_dist < 0.0) {
					if(tmp_l_ai_cnt == 0) { // 첫번째 왼쪽 ai차량의 배열 인덱스
						tmp_l_ai[0] = i;
					} else {
						
						for(int j=0 ; j<10 ; j++) {
							if(tmp_l_ai[j] < 0) {
								tmp_l_ai[j] = i;
								break;
							} else {
								tmp_pre_ai_dist = this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_l_ai[j]]);
								if(tmp_pre_ai_dist < tmp_ai_dist) {
									for(int k=j ; k < tmp_l_ai_cnt ; k++) {
										tmp_l_ai[k+1] = tmp_l_ai[k];
									}
									
									tmp_l_ai[j] = i;
								}
							}
							
							
						}
					}
					
					tmp_l_ai_cnt++;
					
//					System.out.println("   --> 좌측 ai 차량수 : " + tmp_l_ai_cnt);
				
				// 내 차의 오른쪽에 위치하는 ai차량을 거리순으로 array에 저장	
				} else if (tmp_ai_dist >= 0.0) {
					if(tmp_r_ai_cnt == 0) { // 첫번째 왼쪽 ai차량의 배열 인덱스
						tmp_r_ai[0] = i;
					} else {
						
						for(int j=0 ; j<10 ; j++) {
							if(tmp_r_ai[j] < 0) {
								tmp_r_ai[j] = i;
								break;
							} else {
								
								tmp_pre_ai_dist = this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_r_ai[j]]);
								
								if(tmp_pre_ai_dist < tmp_ai_dist) {
									for(int k=j ; k < tmp_r_ai_cnt ; k++) {
										tmp_r_ai[k+1] = tmp_r_ai[k];
									}
									
									tmp_r_ai[j] = i;
								}
							}
							
						}
					}
					
					tmp_r_ai_cnt++;
					
//					System.out.println("   --> 우측 ai 차량수 : " + tmp_r_ai_cnt);
				}
			}
			
		} /* for문 끝 */
		/*================ 내차의 오른쪽 왼쪽 바로 앞쪽 AI 차량 배열 생성 끝 ====================*/
//		System.out.println("---------------------------------------------------");
		
		/*================ 경로 결정 ====================*/
		// ai 차량이 왼쪽에 있고 오른쪽에 없는 경우
		if(tmp_r_ai_cnt == 0 && tmp_l_ai_cnt > 0) {
			if(isLogger) {
				System.out.println("Left ai car : " + tmp_l_ai_cnt + "," + tmp_c_ai_cnt);
			}
			
			// 전방에 ai 차량이 있는 경우 오른방향으로
			//if(tmp_c_ai_cnt > 0  || curr_aicars[tmp_l_ai[0]] < 3.0) {
			if(tmp_c_ai_cnt > 0) {
				//corr_toMiddle = (curr_track_width/2 + curr_toMiddle)/2;
				corr_toMiddle = 3.0 + this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_c_ai[0]]);  // 전방 장애물차의 차폭을 2.5M라고 가정
				
				emer_turn_yn = 1.0;
				// 트랙 바깥으로 벗어나는 경우 트랙까지만 경로 셋팅...전방에 있는 장애물과 부딪힐 경우도 생각해야함...
				if((curr_track_width/2 + curr_toMiddle) < corr_toMiddle ) {
					corr_toMiddle = curr_track_width/2  + curr_toMiddle;
					emer_turn_yn = 2.0;
				}
				
				
			} else {
				
				corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
			}
		
	    // ai 차량이 오른쪽에 있고 왼쪽에 없는 경우
		} else if (tmp_r_ai_cnt > 0 && tmp_l_ai_cnt == 0) {
			if(isLogger) {
				System.out.println("Right ai car : " + tmp_r_ai_cnt + "," + tmp_c_ai_cnt);
			}
			
			// 바로 앞에 ai 차량이 있는 경우 왼쪽방향으로
			//if(tmp_c_ai_cnt > 0 || curr_aicars[tmp_r_ai[0]] > -3.0) {
			if(tmp_c_ai_cnt > 0) {
				//corr_toMiddle = (-curr_track_width/2 + curr_toMiddle)/2;
				corr_toMiddle = -3.0 + this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_c_ai[0]]);
				
				System.out.println("  이동할 거리 : " + corr_toMiddle + ", 트랙까지 거리 : " + (-curr_track_width/2 + curr_toMiddle));
				emer_turn_yn = 1.0;
				// 트랙 바깥인 경우
				if((-curr_track_width/2 + curr_toMiddle) > corr_toMiddle ) {
					corr_toMiddle = -curr_track_width/2  + curr_toMiddle;
					
					emer_turn_yn = 2.0;
				}
				
				
				
			} else {
				
				corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
			}
		
		// 양쪽 모두 차량이 있는 경우
		} else if (tmp_r_ai_cnt > 0 && tmp_l_ai_cnt > 0) {
			if(isLogger) {
				System.out.println("Left and Right ai car : " + tmp_l_ai_cnt + "," + tmp_r_ai_cnt + "," + tmp_c_ai_cnt);
			}
			
			double tmp_left_width = 0.0;
			double tmp_right_width = 0.0;
			// 전방 차량이 있는 경우
			if(tmp_c_ai_cnt > 0) {
				
				// 좌우, 전방 모두 차량이 있는 경우 좌/우 중 간격이 큰 방향의 중간으로 진행
				tmp_left_width = this.getAiSideDist(curr_aicars[tmp_l_ai[0]],curr_aicars[tmp_c_ai[0]]);
				tmp_right_width = this.getAiSideDist(curr_aicars[tmp_c_ai[0]],curr_aicars[tmp_r_ai[0]]);
				
				emer_turn_yn = 1.0;
				
				if(tmp_left_width > tmp_right_width) {
					
					//간격이 내 차량이 지나가기에 충분한 경우만 진행
					if(tmp_left_width > my_car_width) {
						corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + tmp_left_width/2;
						
					//간격이 내 차량이 지나가기에 충분하지 않은 경우  양쪽 차량의 바깥쪽으로 진행(간격이 작은 쪽 우선 체크)  
					} else {
						// 차량이 바로 옆에 있는지 체크(바로 옆에 있음...충돌)
						if(curr_aicars[tmp_r_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_r_ai[0]]) + my_car_width;
						} else if(curr_aicars[tmp_l_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + my_car_width;
						} else {
							// 양쪽이 모두 차량으로 막혀 있는 경우 전방 차량이 7M 앞에 올때까지는 현재 진행경로 유지
							if(curr_aicars[tmp_c_ai[0]-1] > 7.0) {
								corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
							// 전방, 양쪽이 모두 막힐때 브레이킹
							} else {
								
								corr_toMiddle = -100.0;
								emer_turn_yn = 2.0;
							}
						}
						
					}
				} else {
					//간격이 내 차량이 지나가기에 충분한 경우만 진행
					if(tmp_right_width > my_car_width) {
						corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_r_ai[0]]) - tmp_right_width/2;
						
					//간격이 내 차량이 지나가기에 충분하지 않은 경우  양쪽 차량의 바깥쪽으로 진행(간격이 작은 쪽 우선 체크)  	
					} else {
						// 차량이 바로 옆에 있는지 체크(바로 옆에 있음...충돌)
						if(curr_aicars[tmp_l_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + my_car_width;
						} else if(curr_aicars[tmp_r_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_r_ai[0]]) + my_car_width;
						} else {
							// 양쪽이 모두 차량으로 막혀 있는 경우 전방 차량이 7M 앞에 올때까지는 현재 진행경로 유지
							if(curr_aicars[tmp_c_ai[0]-1] > 7.0) {
								corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
							// 전방, 양쪽이 모두 막힐때 브레이킹
							} else {
								corr_toMiddle = -100.0;
								emer_turn_yn = 2.0;
							}
						}
					}
				}
				

			// 전방에 장애물차량이 없는 경우 현재 경로 유지
			} else {
				
				//corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + this.getAiSideDist(curr_aicars[tmp_l_ai[0],curr_aicars[tmp_r_ai[0]])/2;
				corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
			}
			
		// 양쪽에 ai차량이 없는 경우	
		} else {
			
			double tmp_fst_forward_width = 0.0;
			
			// 전방에 ai차량이 있는 경우
			if(tmp_c_ai_cnt > 0) {
				if(isLogger) {
					System.out.println("Forward ai car : " + tmp_c_ai_cnt);
				}
				
				tmp_fst_forward_width = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_c_ai[0]]);
				
				
				if ( tmp_fst_forward_width < 0.0) { // ai 차량 중심이 왼편에 있을때
				    //corr_toMiddle = (curr_track_width/2 + curr_toMiddle)/2;
					
					if(curr_angle < 0.0) {
						corr_toMiddle = tmp_fst_forward_width + my_car_width + 2.0;
					} else {
						corr_toMiddle = tmp_fst_forward_width + my_car_width + 1.0;
					}
					
					// 트랙을 벗어나는 경우 반대방향으로
					if( ((curr_track_width-2)/2 + curr_toMiddle) < corr_toMiddle ) {
						
						// 직선주로에서 
						if (curr_track_dist_straight > 0.0 ) {
							corr_toMiddle = tmp_fst_forward_width - my_car_width - 2.0;
						} else {
							corr_toMiddle = (curr_track_width-2)/2 + curr_toMiddle;
						}
						//corr_toMiddle = curr_toMiddle;
					}
					
					
				} else {  // ai 차량 중심이 오른편에 있을때

					
					if(curr_angle > 0.0) {
						corr_toMiddle = tmp_fst_forward_width - my_car_width - 2.0;
					} else {
						corr_toMiddle = tmp_fst_forward_width - my_car_width - 1.0;
					}

					// 직선주로에서 트랙을 벗어나는 경우 반대으로
					if((-(curr_track_width-2)/2 + curr_toMiddle) > corr_toMiddle ) {
						
						// 직선주로에서 
						if (curr_track_dist_straight > 0.0 ) {
							corr_toMiddle = tmp_fst_forward_width + my_car_width + 2.0;
						} else {
							corr_toMiddle = -(curr_track_width-2)/2 + curr_toMiddle;
						}
	
						//corr_toMiddle = curr_toMiddle;
					}

				}
				
				emer_turn_yn = 1.0;

			} else {
				if(isLogger) {
					System.out.println("No ai car forward. Go Go!!!");
					System.out.println("track_curve_type : " + curr_track_curve_type);
					System.out.println("track_dist_straight : " + curr_track_dist_straight);
					System.out.println("track_whole_dist : " + curr_whole_track_dist_straight);
				}
				
				// 우회전 코스
				if(curr_track_curve_type == 1.0) {
					if(isLogger) {
						System.out.println("Right Curve " + curr_track_dist_straight + " forward.");
					}
					
					//전방 10M 전까지는 좌측으로 주행
					if(curr_track_dist_straight > 15.0 && curr_whole_track_dist_straight > 50.0) {
						// 현재 내 차의 위치가 중앙선 좌측에 있는 경우만 좌측으로 우측에 있는 경우는 중앙선으로
						if(curr_toMiddle > 0) {
							corr_toMiddle = (-(curr_track_width-3)/2 + curr_toMiddle)/5;
							
							// 트랙 바깥인 경우
							if((-curr_track_width/2 + 1.5 + curr_toMiddle) > corr_toMiddle ) {
								corr_toMiddle = -curr_track_width/2  + curr_toMiddle + 2.0;
							}
							
						} else {
							corr_toMiddle = curr_toMiddle/5;
						}
					} else {
						if (curr_track_dist_straight > 2.0) {
							corr_toMiddle = ((curr_track_width - 2.0)/2 + curr_toMiddle)/curr_track_dist_straight;
						} else {
							corr_toMiddle = ((curr_track_width - 2.0)/2 + curr_toMiddle)/2;
						}
					}
					
				} else if(curr_track_curve_type == 2.0){
					if(isLogger) {
						System.out.println("Left Curve " + curr_track_dist_straight + " forward.");
					}
					
					//전방 10M 전까지는 우측으로 주행
					if(curr_track_dist_straight > 15.0  && curr_whole_track_dist_straight > 50.0) {
						
						// 현재 내 차의 위치가 중앙선 우측에 있는 경우만 우측으로 좌측에 있는 경우는 중앙선으로
						if(curr_toMiddle < 0) {
							corr_toMiddle = ((curr_track_width-3)/2 + curr_toMiddle)/5;
							
							// 트랙 바깥으로 벗어나는 경우 트랙까지만 경로 셋팅...전방에 있는 장애물과 부딪힐 경우도 생각해야함...
							if((curr_track_width/2 - 1.5 + curr_toMiddle) < corr_toMiddle ) {
								corr_toMiddle = curr_track_width/2  + curr_toMiddle - 2.0;
							}
							
						} else {
							corr_toMiddle = curr_toMiddle/5;
						}
						
					} else {
						if (curr_track_dist_straight > 2.0) {
							corr_toMiddle = (-(curr_track_width - 2.0)/2 + curr_toMiddle)/curr_track_dist_straight ;
						} else {
							corr_toMiddle = (-(curr_track_width - 2.0)/2 + curr_toMiddle)/2;
						}
					}
				} else {
				
					corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
				}
			}
		}

		
		ret_corr_route[0] = emer_turn_yn;
		ret_corr_route[1] = corr_toMiddle;
		if(tmp_c_ai_cnt > 0) {
			ret_corr_route[2] = curr_aicars[tmp_c_ai[0]-1]; // 제일 근접한 전방 ai 차량 거리
		}
		if(isLogger) {
			System.out.println("corr_toMiddle : " + corr_toMiddle);
		}

		return ret_corr_route;
	}
	
	/**
	 * 차량 간 Mid값으로 서로의 간격 구하기 (내차 기준)
	 * @param curr_toMiddle
	 * @param curr_aiMiddle
	 * @return
	 */
	private double getAiSideDist (double curr_toMiddle, double curr_aiMiddle) {
		double ret_dist = 0.0;
		
		ret_dist = curr_toMiddle - curr_aiMiddle ;
		
		return ret_dist;
	}
	
	/**
	 * Mid값으로 트랙 사이드까지 간격 구하기 (내차 기준)
	 * @param curr_toMiddle
	 * @param curr_track_width
	 * @param side
	 * @return
	 */
	private double getTrackSideDist (double curr_toMiddle, double curr_track_width, int side) {
		double ret_dist = 0.0;
		
		if(side == 1) { // 오른쪽 사이드까지 거리
			ret_dist = curr_track_width/2 + curr_toMiddle;
		} else if(side == 2) { // 왼쪽 사이드까지 거리
			ret_dist = -(curr_track_width/2) + curr_toMiddle;
		}
		
		return ret_dist;
	}
	
	/**
	 * 현재 진행 경로를 유지하기 위한 값 (내차 기준)
	 * 0값을 셋팅하면 현재 경로가 유지되나 계산시 미세보정
	 * @param curr_toMiddle
	 * @param curr_track_width
	 * @return
	 */
	private double getKeepTrackSideDist (double curr_toMiddle, double curr_track_width) {
		double ret_corr_toMiddle = 0.0;
		double tmp_r_track_side = this.getTrackSideDist(curr_toMiddle, curr_track_width, 1);
		double tmp_l_track_side = this.getTrackSideDist(curr_toMiddle, curr_track_width, 2);
		
		//전방에 장애물 차량이 없는 경우는 그냥 가던 길로
		
		if(tmp_r_track_side < 0.0 || tmp_l_track_side > 0.0) {
			if(tmp_r_track_side < 0.0) {
				ret_corr_toMiddle = tmp_r_track_side - 1;
			}
			
			if(tmp_l_track_side > 0.0) {
				ret_corr_toMiddle = tmp_l_track_side + 1;
			}
			
		} 
		
		return ret_corr_toMiddle;
	}
	
	
	/**
	 *  최적 속도 계산
	 * @param curr_speed
	 * @param curr_track_dist_straight
	 * @param curr_track_curve_type
	 * @return
	 */
	private double getBestSpeed(double curr_angle, double curr_dist_aicar, double curr_speed, double curr_track_dist_straight, double curr_track_curve_type){
		double user_best_speed = 100;
		
		float user_c_coeff      = (float)2.772;
		float user_d_coeff		= (float)-0.693;
		
		double curr_max_speed = 100;
		
		// 90 ~ 130 마일 일경우  초당 40.2~58.1m 이동 : 
		// 70 ~  90  마일 일경우 초당 31.2~40.2m 이동 : 
		//    ~  70  마일 일경우 초당 0   ~31.2m 이동 : 
				
		if(curr_track_dist_straight > 100){
			curr_max_speed = 50;
			
			user_c_coeff = (float)2.772;
			user_d_coeff = (float)-0.693;
		}else if (curr_track_dist_straight > 80 && curr_track_dist_straight <= 100) {
			curr_max_speed = 40;
			
			user_c_coeff = (float)2.0;
			user_d_coeff = (float)-0.9;
		}else if (curr_track_dist_straight > 50 && curr_track_dist_straight <= 80) {
			curr_max_speed = 30;
			
			user_c_coeff = (float)1.5;
			user_d_coeff = (float)-1.2;
		}else{
			curr_max_speed = 25; // 90도 이상일때 최적 속도(88~90km/h)
			
			user_c_coeff = (float)1.0;
			user_d_coeff = (float)-1.5;
		}
		
		
		
		double curr_angle_abs = Math.abs(curr_angle*180/3.14);
		
		if(curr_angle_abs <= 10){
			curr_max_speed = curr_max_speed*1.3;
		}else if(curr_angle_abs > 10 && curr_angle_abs <= 30 ){
			curr_max_speed = curr_max_speed*1;
		}else if(curr_angle_abs > 30 && curr_angle_abs <= 45 ){
			curr_max_speed = curr_max_speed*0.7;
		}else if(curr_angle_abs > 45 && curr_angle_abs <= 90 ){		
			curr_max_speed = curr_max_speed*0.6;
		}else if(curr_angle_abs > 90 && curr_angle_abs <= 135 ){
			curr_max_speed = curr_max_speed*0.5;
		}else if(curr_angle_abs > 135 && curr_angle_abs <= 180 ){
			curr_max_speed = curr_max_speed*0.2;
		}else{
			curr_max_speed = curr_max_speed*0.1;
		}
		 
//		/*테스트 셋팅 */
		//curr_max_speed = 25;
//		/*테스트 셋팅 */
		
		user_best_speed = curr_max_speed * (1 - Math.exp(-user_c_coeff/curr_max_speed * curr_dist_aicar - user_d_coeff));
		if(isLogger) {
			System.out.println("+++++++++++++++++ 최적 속도 계산[start] ++++++++++++++++++++++");
			System.out.println("curr_max_speed          ="+curr_max_speed);
			System.out.println("user_best_speed         ="+user_best_speed);
			System.out.println("curr_speed              ="+curr_speed + " m/s");
			System.out.println("curr_speed              ="+curr_speed*3.6 + " km/h");
			System.out.println("curr_angle              ="+curr_angle);
			System.out.println("curr_angle_abs          ="+curr_angle_abs + " 도");
			System.out.println("curr_track_dist_straight="+curr_track_dist_straight);
			System.out.println("+++++++++++++++++ 최적 속도 계산[end] ++++++++++++++++++++++");
		}
		return user_best_speed;
	}
	
	/**
	 * 브레이크, 엑셀 제어 함수
	 * @param curr_speed
	 * @param curr_best_speed
	 * @param curr_track_dist_straight
	 * @return
	 */
	private double[] getSpeedCtl(double curr_speed, double curr_best_speed, double curr_track_dist_straight){
		double[] user_speed_ctl = new double[2];
		user_speed_ctl[0] = 0.2; // accel
		user_speed_ctl[1] = 0.0; // brake

		if(curr_speed > curr_best_speed) {
			if(isLogger) {
				System.out.println("+++++++++++++++++ 브레이크, 엑셀 제어 함수[start] ++++++++++++++++++++++");
				System.out.println("curr_speed               = "+curr_speed);
				System.out.println("curr_best_speed          = "+curr_best_speed);
				System.out.println("curr_track_dist_straight = "+curr_track_dist_straight);
			}
			user_speed_ctl[0] = 0.1;
			
			if(curr_track_dist_straight < 20){
				user_speed_ctl[1] = 0.2;
			}else{
				user_speed_ctl[1] = 0.2;
			}
			if(isLogger) {
				System.out.println("user_brakeCtl="+user_speed_ctl[1]);
				System.out.println("+++++++++++++++++ 브레이크, 엑셀 제어 함수[end] ++++++++++++++++++++++");
			}
			
		}else{
			user_speed_ctl[0] = 0.4;
		}		

		
		return user_speed_ctl;		
	}
	
	/**
	 * 브레이크, 엑셀 제어 함수(자체)
	 * @param curr_speed
	 * @param curr_best_speed
	 * @param curr_track_dist_straight
	 * @return
	 */
	private double[] getSpeedCtl2(double curr_speed, double curr_best_speed, double curr_track_dist_straight, double track_curve_level){
		double corr_break = 0.0;
		double corr_accel = 0.2;
		double[] user_speed_ctl = new double[2];
		user_speed_ctl[0] = 0.2; // accel
		user_speed_ctl[1] = 0.0; // brake

		//if(curr_speed > curr_best_speed) {
			if(isLogger) {
				System.out.println("+++++++++++++++++ 브레이크, 엑셀 제어 함수2[start] ++++++++++++++++++++++");
				System.out.println("curr_speed               = "+curr_speed);
				System.out.println("curr_best_speed          = "+curr_best_speed);
				System.out.println("curr_track_dist_straight = "+curr_track_dist_straight);
			}
			
			if(curr_track_dist_straight > 0.0 && curr_track_dist_straight < 15.0){
				if( curr_speed > 35.0) {
					corr_break = 0.4;
					corr_accel = 0.1;
				} else if( curr_speed > 30.0  && curr_speed <= 35.0) {
					corr_break = 0.3;
					corr_accel = 0.1;
				} else if ( curr_speed > 23.0 && curr_speed <= 30.0){
					corr_break = 0.2;
					corr_accel = 0.1;
				} else if ( curr_speed > 14.0 && curr_speed <= 23.0){
					corr_break = 0.1;
					corr_accel = 0.1;
				} else {
					corr_break = 0.1;
					corr_accel = 0.2;				
				}
			} else if(curr_track_dist_straight == 0.0) {
				if( curr_speed > 35.0) {
					if(track_curve_level > 0.055) {
						corr_break = 0.5;
						corr_accel = 0.1;
					} else if(track_curve_level > 0.045 && track_curve_level <= 0.055) {
						corr_break = 0.4;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.035 && track_curve_level <= 0.045) {
						corr_break = 0.3;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.030 && track_curve_level <= 0.035) {
						corr_break = 0.2;
						corr_accel = 0.1;
					} else {
						corr_break = 0.1;
						corr_accel = 0.1;
					}
					
				} else if( curr_speed > 30.0  && curr_speed <= 35.0) {
					if(track_curve_level > 0.055) {
						corr_break = 0.4;
						corr_accel = 0.1;
					} else if(track_curve_level > 0.045 && track_curve_level <= 0.055) {
						corr_break = 0.3;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.035 && track_curve_level <= 0.045) {
						corr_break = 0.2;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.030 && track_curve_level <= 0.035) {
						corr_break = 0.1;
						corr_accel = 0.1;
					} else {
						corr_break = 0.1;
						corr_accel = 0.2;
					}
				} else if ( curr_speed > 23.0 && curr_speed <= 30.0){
					
					if(track_curve_level > 0.055) {
						corr_break = 0.3;
						corr_accel = 0.1;
					} else if(track_curve_level > 0.045 && track_curve_level <= 0.055) {
						corr_break = 0.2;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.035 && track_curve_level <= 0.045) {
						corr_break = 0.1;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.030 && track_curve_level <= 0.035) {
						corr_break = 0.0;
						corr_accel = 0.1;
					} else {
						corr_break = 0.0;
						corr_accel = 0.2;
					}
				} else if ( curr_speed > 14 && curr_speed <= 23.0){
					if(track_curve_level > 0.055) {
						corr_break = 0.2;
						corr_accel = 0.1;
					} else if(track_curve_level > 0.045 && track_curve_level <= 0.055) {
						corr_break = 0.1;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.035 && track_curve_level <= 0.045) {
						corr_break = 0.0;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.030 && track_curve_level <= 0.035) {
						corr_break = 0.0;
						corr_accel = 0.2;
					} else {
						corr_break = 0.0;
						corr_accel = 0.3;
					}
					
				} else {
					if(track_curve_level > 0.055) {
						corr_break = 0.1;
						corr_accel = 0.1;
					} else if(track_curve_level > 0.045 && track_curve_level <= 0.055) {
						corr_break = 0.1;
						corr_accel = 0.2;
					} else if (track_curve_level > 0.035 && track_curve_level <= 0.045) {
						corr_break = 0.0;
						corr_accel = 0.2;
					} else if (track_curve_level > 0.030 && track_curve_level <= 0.035) {
						corr_break = 0.0;
						corr_accel = 0.3;
					} else {
						corr_break = 0.0;
						corr_accel = 0.4;
					}
									
				}
				
			} else {
				if( curr_speed > 35.0) {
					if(curr_track_dist_straight > 100) {
						corr_break = 0.0;
						corr_accel = 0.4;
					} else {
						corr_break = 0.0;
						corr_accel = 0.3;
					}
				} else if ( curr_speed > 23 && curr_speed <= 35.0){
					if(curr_track_dist_straight > 100) {
						corr_break = 0.0;
						corr_accel = 0.5;
					} else {
						corr_break = 0.0;
						corr_accel = 0.4;
					}
				} else {
					if(curr_track_dist_straight > 100) {
						corr_break = 0.0;
						corr_accel = 0.6;
					} else {
						corr_break = 0.0;
						corr_accel = 0.5;
					}
				}
			}
			
			user_speed_ctl[0] = corr_accel;
			user_speed_ctl[1] = corr_break;
			if(isLogger) {
				System.out.println("user_accelCtl="+user_speed_ctl[0]);
				System.out.println("user_brakeCtl="+user_speed_ctl[1]);
				System.out.println("+++++++++++++++++ 브레이크, 엑셀 제어 함수2[end] ++++++++++++++++++++++");
			}
			
		//}else{
		//	user_speed_ctl[0] = 0.4;
		//}		

		
		return user_speed_ctl;		
	}
	
	/**
	 * 최적 이동 함수(기본)
	 * @param curr_toMiddle
	 * @param curr_track_width
	 * @param curr_track_curve_type
	 * @return
	 */
	private double getBestDist(double curr_toMiddle, double curr_track_width, double curr_track_curve_type){
		double user_change_dist = 0.0;		
		
		if(curr_track_curve_type == 1){ // 우회전 
			user_change_dist = curr_toMiddle + ( curr_track_width/3 );  
		}else if(curr_track_curve_type == 2){ // 좌회전
			user_change_dist = curr_toMiddle - ( curr_track_width/3 );
		}else{
			user_change_dist = curr_toMiddle;
		}		
		
		return user_change_dist;		
	}	
	
	/**
	 * 장애물 처리 함수
	 * @param curr_aicars
	 * @param change_toMiddle
	 * @param curr_speed
	 * @param curr_track_width
	 * @param curr_track_dist_straight
	 * @param curr_track_curve_type
	 * @return
	 */
	private double[] getChangeDist(double[] curr_aicars, double curr_toMiddle, double change_toMiddle, double curr_speed, double curr_track_width, double curr_track_dist_straight, double curr_track_curve_type) {
		

		
		double user_check_forward = 100;
		double user_car_width = 2; // 전폭 2m
		double user_car_width_half = user_car_width/2; // 전폭 2m
		double ai_car_length      = 4.8; // 전장 4.8
		double ai_car_length_half = ai_car_length/2; // 전장 4.8
		
		if(curr_speed >= 30){
			user_check_forward = 100;
		}else if(curr_speed < 30 && curr_speed >= 20){
			user_check_forward = 50;
		}else{
			user_check_forward = 10;
		}		
		
		System.out.println("user_check_forward         = "+user_check_forward);
		
//		/*테스트 셋팅 */
//		user_check_forward = 20;
//		/*테스트 셋팅 */

		
		double[] user_change_dist = new double[2]; // change_toMiddle;
		user_change_dist[0] = change_toMiddle; // 변경할 거리 (초기는 최적화된 코스로 셋팅)
		user_change_dist[1] = 100; // 전방 장애물 거리
		
		// 전방 차량 모두 검증
		double[] cars_dist = new double[2];
		cars_dist[0] = 2; // 차량 실거리
		cars_dist[1] = 2; // 차량 중심거리(좌우 판단 가능)
		
		for(int i=0;i<5;i++){
			System.out.println("dist_cars_dist[" + i + "]    = " + curr_aicars[i]);
			System.out.println("dist_cars_middle[" + i + "]  = " + curr_aicars[i + 1]);
			
			if(i == 0 && curr_aicars[i] == 100){
				System.out.println("전방 장애물차량 없음");
				break;
			}else{
				if(curr_aicars[i] == 100 || curr_aicars[i] > user_check_forward) {
					System.out.println("전방 차량[" + i + "] 번 무시");
					continue;
				}
			}
			
			
			
			cars_dist = this.getCarsDist(curr_toMiddle, change_toMiddle, curr_aicars[2*i + 1], curr_track_curve_type, user_car_width, ai_car_length);
			
			
			// 차량 중심거리가 음수 이면 차량이 충돌하는 것으로 판단하여 우회
			// TODO : 로직 변경 필요 차량 실거리
			if(cars_dist[0] <=  1.2 ){
				System.out.println("+++++++++++++++++ 전방 장애물 발견 우회 하세요 ++++++++++++++++++++++");
				System.out.println("전방 차량[" + i + "] 번 회피");
				if(cars_dist[1] > 0){ // 장애물 차가 우측에 존재(왼쪽으로 기본 이동)
					change_toMiddle = change_toMiddle + ( Math.abs(cars_dist[0]) + user_car_width );
				}else if(cars_dist[1] < 0){ // 장애물 차가 좌측에 존재(오른쪽으로 기본 이동)
					change_toMiddle = change_toMiddle - ( Math.abs(cars_dist[0]) + user_car_width );
				}else{
					if(curr_track_curve_type == 1 ){ // 우회전 코스(오른쪽으로 기본 이동)
						change_toMiddle = change_toMiddle - ( Math.abs(cars_dist[0]) + user_car_width );
					}else{ // 좌회전 코스(왼쪽으로 기본 이동)
						change_toMiddle = change_toMiddle + ( Math.abs(cars_dist[0]) + user_car_width );
					}
				}
				user_change_dist[1] = curr_aicars[i];
				break;
			}else if(cars_dist[0] > 1.2 && cars_dist[0] <= 1.5) {
				System.out.println("+++++++++++++++++ 전방 장애물 발견 유지 하세요 ++++++++++++++++++++++");
				System.out.println("좌우 차량[" + i + "] 확인 필요");
				if(cars_dist[1] > 0){ // 좌측에 존재
					change_toMiddle = change_toMiddle + Math.abs(cars_dist[0]);
				}else if(cars_dist[1] < 0){ // 우측에 존재
					change_toMiddle = change_toMiddle - Math.abs(cars_dist[0]);
				}else{
					if(curr_track_curve_type == 1 ){ // 우회전 코스
						change_toMiddle = change_toMiddle + Math.abs(cars_dist[0]);
					}else{ // 좌회전 코스
						change_toMiddle = change_toMiddle - Math.abs(cars_dist[0]);
					}
				}
			}			
		}
		
		user_change_dist[0] = change_toMiddle;
		
		System.out.println("change_toMiddle         = "+change_toMiddle);
		System.out.println("user_change_middle      = "+user_change_dist[0]);
		System.out.println("user_ai_car_dist        = "+user_change_dist[1]);
		
				
		return user_change_dist;
	}
	
	/**
	 * 차량간 실거리 계산
	 * @param user_car_toMiddle
	 * @param ai_car_toMiddle
	 * @param curr_track_curve_type
	 * @param user_car_width_half
	 * @param ai_car_length_half
	 * @return
	 */
	private double[] getCarsDist(double curr_toMiddle, double user_change_toMiddle, double ai_car_toMiddle, double curr_track_curve_type, double user_car_width, double ai_car_length){
		double[] cars_dist = new double[2];
		cars_dist[0] =  0; // 차량간 실거리
		cars_dist[1] =  0; // 차량 중심거리(좌우 판단 가능)
		
		double cars_middle_dist = curr_toMiddle - ai_car_toMiddle; // 차량과의 중심 거리
		cars_dist[1] = cars_middle_dist;
		System.out.println("cars_middle_dist(중심거리) = "+cars_middle_dist); 
		
		// 차량간 실거리 (차폭을 고려하여 계산)
		cars_dist[0] = Math.abs(cars_middle_dist) - user_car_width/2 - ai_car_length/2;		
		System.out.println("cars_dist(실거리)         = "+cars_dist[0]);
		
		return cars_dist;		
	}
	
			
}